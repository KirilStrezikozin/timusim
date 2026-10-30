"""V4 hinge jig: three sensors on an arm that turns between two stops, the box flat, tilted or on
edge. Session 1 fits the geometry (mounts, hinge axis, arm direction); every reported capture is
replayed through the chip models and scored on closure error, gyro scale, lever-arm residual,
mount agreement and static decomposition, with the RESEARCH.md acceptance rule."""

import dataclasses
import pathlib
import typing

import numpy as np
import tomllib

from timusim import capture, rig, sim, v1, v2

ORIENTATIONS = ("hinge_horizontal", "tilted", "hinge_vertical")
# The arm's rest angle between the stops, per base orientation (the author's measurements): with
# the hinge horizontal the arm sinks about 5 degrees past first contact under its own weight.
THETA_REST_DEG = {"hinge_horizontal": 176.0, "tilted": 171.0, "hinge_vertical": 171.0}
# In sweeps the arm rebounds off the stops. With the hinge vertical gravity cannot place the rests,
# and tilted the rocking moves gravity's projected angle, so there they are taken as apart by the
# mean of this first measurement and the LSM6DSL's integrated span (the author's choice).
SWEEP_REST_MEASURED_DEG = 168.0
GYRO_REST_ORIENTATIONS = ("tilted", "hinge_vertical")
# Where gravity is normal to the hinge, the reference for the per-direction gyro gain.
GAIN_ORIENTATION = "hinge_horizontal"
# Stop contacts are left out of the radius fit.
CONTACT_ALPHA_RAD_S2 = 10.0
MOVING_DPS = 10.0
SETTLE_DPS = 5.0
SETTLE_S = 0.2
CONTACT_DEG = 3.0
TOLERANCE = {"closure_deg": 1.0, "gyro_scale_deg": 1.0, "tilt_deg": 1.0}
# palm_lsm9 was taken off the arm for its V2 and put back before this session: from here on its
# mount comes from this session's base rotation.
REMOUNT_SESSION = 2
RELATIVE = {"lever_residual_g": 0.25, "mount_rms_dps": 0.25}


def classify(e: rig.Entry) -> tuple[str, str]:
    """(orientation, kind) from the capture notes; kind is hold_a, hold_b, sweeps or base_rotation."""
    n = e.notes
    kind = (
        "base_rotation" if n.startswith("base rotation")
        else "hold_a" if "hold at stop A" in n
        else "hold_b" if "hold at stop B" in n
        else "sweeps" if "sweeps" in n
        else "other"
    )
    if kind == "base_rotation":
        return "", kind
    orientation = (
        "tilted" if n.startswith("tilted")
        else "hinge_vertical" if n.startswith("hinge vertical")
        else "hinge_horizontal" if n.startswith("hinge horizontal")
        else ""
    )
    return orientation, kind


def by_kind(es: list[rig.Entry]) -> dict[tuple[int, str, str], list[rig.Entry]]:
    out: dict[tuple[int, str, str], list[rig.Entry]] = {}
    for e in es:
        o, k = classify(e)
        out.setdefault((e.session, o, k), []).append(e)
    return out


def rate(s: capture.SensorStream) -> np.ndarray:
    return s.gyro_dps - rig.zero_level(s.gyro_dps, s.odr_hz)


def hold_up(s: capture.SensorStream, params: dict | None = None) -> np.ndarray:
    """Up in the sensor's frame from a hold; with params, corrected by its accel zero level and
    gain (V2)."""
    a = s.accel_g[s.longest_run(s.t_s >= v2.SETTLE_S)].mean(axis=0)
    if params is not None:
        a = np.linalg.solve(np.array(params.get("accel_gain", np.eye(3))), a - np.array(params.get("accel_offset", np.zeros(3))))
    return a / np.linalg.norm(a)


def accel_params(data_dir: pathlib.Path) -> dict[str, dict]:
    return {n: tomllib.loads((data_dir.parent / rig.PARAMS[n]).read_text()) for n in rig.SENSORS}


@dataclasses.dataclass(frozen=True)
class Geometry:
    mount: dict[str, np.ndarray]  # sensor frame -> palm_lsm9 frame
    hinge: dict[str, np.ndarray]  # unit, in each sensor's frame, positive from A towards B
    radial: dict[str, np.ndarray]  # unit, from the pivot outwards along the arm, in each frame
    # Integrated over true hinge angle, A->B and B->A, per sensor.
    gyro_gain: dict[str, tuple[float, float]]
    # The arm rocks out of its plane at the stops: per sensor and orientation, the turn (axis,
    # deg) in the sensor's frame from up at B predicted by the hinge alone to session 1's measured
    # one.
    stop_b: dict[tuple[str, str], tuple[np.ndarray, float]]
    sweep_rest_deg: dict[str, float]  # rest-to-rest angle in sweeps, where gravity cannot tell


def sweep_rest_deg(geometry: Geometry, orientation: str) -> float:
    return geometry.sweep_rest_deg.get(orientation, THETA_REST_DEG[orientation])


def fit_mounts(cap: capture.Capture) -> dict[str, np.ndarray]:
    ref = rig.stream(cap, "palm_lsm9")
    w_ref = rate(ref)
    out = {"palm_lsm9": np.eye(3)}
    for name in ("palm", "finger"):
        s = rig.stream(cap, name)
        w = rate(s)
        w_l9 = rig.resample(rig.board_time(ref), w_ref, rig.board_time(s))
        moving = np.linalg.norm(w, axis=1) > MOVING_DPS
        out[name] = rig.fit_rotation(w_l9[moving], w[moving])
    return out


def remount(geometry: Geometry, cap: capture.Capture) -> Geometry:
    """The geometry after palm_lsm9 turned on the arm: the turn from its old frame to its new one,
    fitted on a base rotation with palm and finger as the reference, moves every mount and
    palm_lsm9's own directions."""
    ref = rig.stream(cap, "palm_lsm9")
    old, new = [], []
    for name in ("palm", "finger"):
        s = rig.stream(cap, name)
        w = rate(s)
        moving = np.linalg.norm(w, axis=1) > MOVING_DPS
        old.append(w[moving] @ geometry.mount[name].T)
        new.append(rig.resample(rig.board_time(ref), rate(ref), rig.board_time(s))[moving])
    turn = rig.fit_rotation(np.concatenate(new), np.concatenate(old))
    own = lambda d: {n: turn @ v if n == "palm_lsm9" else v for n, v in d.items()}
    stop_b = {k: (turn @ axis if k[0] == "palm_lsm9" else axis, deg) for k, (axis, deg) in geometry.stop_b.items()}
    return dataclasses.replace(
        geometry, mount={n: turn @ m for n, m in geometry.mount.items()} | {"palm_lsm9": np.eye(3)},
        hinge=own(geometry.hinge), radial=own(geometry.radial), stop_b=stop_b,
    )


def remounted(geometry: Geometry, data_dir: pathlib.Path, *, log: rig.Log = print) -> Geometry:
    """The geometry sessions from REMOUNT_SESSION on are predicted with; session 1's if that
    session has no base rotation yet."""
    rot = by_kind([e for e in rig.entries(data_dir, "V4") if e.session == REMOUNT_SESSION]).get((REMOUNT_SESSION, "", "base_rotation"))
    if not rot:
        return geometry
    moved = remount(geometry, capture.read_capture(rot[-1].path))
    turn = moved.mount["finger"] @ geometry.mount["finger"].T
    log(f"palm_lsm9 re-mounted: turned {np.degrees(np.arccos(np.clip((np.trace(turn) - 1) / 2, -1, 1))):.2f} deg on the arm (session {REMOUNT_SESSION} base rotation)")
    return moved


def fit_hinge(sweeps: list[capture.Capture]) -> np.ndarray:
    """Principal direction of palm_lsm9's rate while the arm moves."""
    w = np.concatenate([rate(rig.stream(c, "palm_lsm9")) for c in sweeps])
    w = w[np.linalg.norm(w, axis=1) > MOVING_DPS]
    _, vecs = np.linalg.eigh(w.T @ w)
    return vecs[:, -1]


def fit_radial(cap: capture.Capture, name: str, hinge: np.ndarray, up_a: np.ndarray) -> tuple[np.ndarray, float]:
    """From hinge-vertical sweeps: the in-plane accel, less gravity turned with the arm from up at
    A (the hinge is a few degrees off vertical, which leaves 0.07-0.08 g in the plane), is
    -w^2 c + alpha (h x c) + zero level, with c the radius vector. Stop contacts are left out.
    Returns its direction and length (m)."""
    s = rig.stream(cap, name)
    t = rig.board_time(s)
    w_dps = rate(s) @ hinge
    w = np.radians(w_dps)
    alpha = np.gradient(w, t)
    theta = np.cumsum(w_dps) / s.odr_hz
    first = rig.still(s.gyro_dps, s.odr_hz)[0]
    theta -= theta[first[0] : first[1]].mean()
    a = (s.accel_g - rig.rotate(hinge, -theta, np.broadcast_to(up_a, (len(t), 3)))) * rig.G
    a_in = a - np.outer(a @ hinge, hinge)
    moving = (np.abs(w) > np.radians(MOVING_DPS)) & (np.abs(alpha) < CONTACT_ALPHA_RAD_S2)
    hx = np.array([[0.0, -hinge[2], hinge[1]], [hinge[2], 0.0, -hinge[0]], [-hinge[1], hinge[0], 0.0]])
    rows = (-(w[moving] ** 2))[:, None, None] * np.eye(3) + alpha[moving][:, None, None] * hx
    # Plus a constant for the accel zero level.
    rows = np.concatenate([rows, np.broadcast_to(np.eye(3), rows.shape)], axis=2)
    x, *_ = np.linalg.lstsq(rows.reshape(-1, 6), a_in[moving].reshape(-1), rcond=None)
    c = x[:3] - (x[:3] @ hinge) * hinge
    return c / np.linalg.norm(c), float(np.linalg.norm(c))


def fit_geometry(data_dir: pathlib.Path, *, log: rig.Log = print) -> Geometry:
    groups = by_kind([e for e in rig.entries(data_dir, "V4") if e.session == 1])
    rot = groups.get((1, "", "base_rotation"))
    if not rot:
        raise SystemExit("no V4 session-1 base-rotation capture in the manifest")
    mount = fit_mounts(capture.read_capture(rot[-1].path))
    sweeps = {o: [capture.read_capture(e.path) for e in groups.get((1, o, "sweeps"), [])] for o in ORIENTATIONS}
    h = fit_hinge([c for cs in sweeps.values() for c in cs])
    # Positive turns A towards B: world up, seen from the arm, turns the other way.
    a, b = groups.get((1, "hinge_horizontal", "hold_a")), groups.get((1, "hinge_horizontal", "hold_b"))
    if a and b:
        up_a = hold_up(rig.stream(capture.read_capture(a[-1].path), "palm_lsm9"))
        up_b = hold_up(rig.stream(capture.read_capture(b[-1].path), "palm_lsm9"))
        if rig.signed_angle(h, up_a, up_b) > 0:
            h = -h
    hinge = {n: mount[n].T @ h for n in rig.SENSORS}
    hold_caps = {(o, k): capture.read_capture(groups[(1, o, k)][-1].path) for o in ORIENTATIONS for k in ("hold_a", "hold_b") if (1, o, k) in groups}
    holds = {key: hold_up(rig.stream(c, "palm_lsm9")) for key, c in hold_caps.items()}
    radial = {}
    for n in rig.SENSORS:
        own = fit_hinge_one(sweeps, n)
        log(f"{n}: own hinge fit {np.degrees(np.arccos(min(1.0, abs(own @ hinge[n])))):.2f} deg from the mapped one")
        if sweeps["hinge_vertical"] and ("hinge_vertical", "hold_a") in holds:
            up_a = hold_up(rig.stream(hold_caps[("hinge_vertical", "hold_a")], n))
            radial[n], r = fit_radial(sweeps["hinge_vertical"][-1], n, hinge[n], up_a)
            log(f"{n}: fitted radius {100 * r:.1f} cm (measured {100 * rig.RADIUS_M[n]:.1f}, used)")

    stop_b = {}
    corrections = accel_params(data_dir)
    for o in ORIENTATIONS:
        if (o, "hold_a") not in holds or (o, "hold_b") not in holds:
            continue
        for n in rig.SENSORS:
            up_a, up_b = (hold_up(rig.stream(hold_caps[(o, k)], n), corrections[n]) for k in ("hold_a", "hold_b"))
            pred = rig.rotate(hinge[n], -THETA_REST_DEG[o], up_a)
            axis = np.cross(pred, up_b)
            deg = float(np.degrees(np.arctan2(np.linalg.norm(axis), pred @ up_b)))
            stop_b[(n, o)] = (axis / np.linalg.norm(axis), deg)
        log(f"{o}: stop B off the hinge-only prediction by "
            + ", ".join(f"{stop_b[(n, o)][1]:.1f}" for n in rig.SENSORS) + " deg (" + ", ".join(rig.SENSORS) + ")")

    partial = Geometry(mount, hinge, radial, {}, stop_b, {})
    gyro_gain = {}
    if sweeps[GAIN_ORIENTATION] and (GAIN_ORIENTATION, "hold_a") in holds:
        spans = [span_ratios(c, partial, GAIN_ORIENTATION, holds[(GAIN_ORIENTATION, "hold_a")]) for c in sweeps[GAIN_ORIENTATION]]
        for n in rig.SENSORS:
            gyro_gain[n] = tuple(float(np.mean([v for sp in spans for v in sp[n][d]])) for d in ("a_to_b", "b_to_a"))
            log(f"{n}: gyro gain against gravity A->B {gyro_gain[n][0]:.4f}, B->A {gyro_gain[n][1]:.4f}")
    rest = {}
    for o in GYRO_REST_ORIENTATIONS:
        if not sweeps[o] or "palm" not in gyro_gain:
            continue
        spans = []
        for c in sweeps[o]:
            l9 = rig.stream(c, "palm_lsm9")
            rs = rests(l9, hinge["palm_lsm9"], o, np.zeros(3), THETA_REST_DEG[o])
            m = sweep_metrics(rig.stream(c, "palm"), 0.0, hinge["palm"], None, 0.0, rs)
            spans += [v / g for d, g in zip(("a_to_b", "b_to_a"), gyro_gain["palm"]) for v in m.gyro_scale_deg[d]]
        rest[o] = (float(np.mean(spans)) + SWEEP_REST_MEASURED_DEG) / 2
        log(f"{o}: sweep rests {rest[o]:.1f} deg apart (LSM6DSL {np.mean(spans):.1f}, measured {SWEEP_REST_MEASURED_DEG:.0f})")
    return Geometry(mount, hinge, radial, gyro_gain, stop_b, rest)


def span_ratios(cap: capture.Capture, geometry: Geometry, orientation: str, up_a: np.ndarray) -> dict[str, dict[str, list[float]]]:
    """Per sensor and direction, each half sweep's integrated angle over its span from gravity."""
    l9 = rig.stream(cap, "palm_lsm9")
    rs = rests(l9, geometry.hinge["palm_lsm9"], orientation, up_a, THETA_REST_DEG[orientation])
    spans = {"a_to_b": [], "b_to_a": []}
    for r0, r1 in zip(rs, rs[1:]):
        spans["b_to_a" if r0.at_b else "a_to_b"].append(abs(r1.target_deg - r0.target_deg))
    out = {}
    for n in rig.SENSORS:
        m = sweep_metrics(rig.stream(cap, n), 0.0, geometry.hinge[n], None, 0.0, rs)
        out[n] = {d: [v / t for v, t in zip(m.gyro_scale_deg[d], spans[d])] for d in spans}
    return out


def gyro_params(geometry: Geometry, name: str, base: str) -> str:
    """The parameter file with the per-direction gain on the axis nearest the hinge; it goes
    before the first table."""
    if name not in geometry.gyro_gain:
        return base
    h = geometry.hinge[name]
    axis = int(np.argmax(np.abs(h)))
    a_to_b, b_to_a = geometry.gyro_gain[name]
    # A->B turns positive about the hinge: on this axis, that is a positive rate when h points along it.
    positive, negative = (a_to_b, b_to_a) if h[axis] > 0 else (b_to_a, a_to_b)
    gain = np.eye(3)
    gain[axis, axis] = positive
    neg = np.ones(3)
    neg[axis] = negative / positive
    lines = [ln for ln in base.splitlines() if not ln.startswith(("gyro_gain", "gyro_negative_gain"))]
    first_table = next((i for i, ln in enumerate(lines) if ln.startswith("[")), len(lines))
    rows = ", ".join("[" + ", ".join(f"{c:.6g}" for c in row) + "]" for row in gain)
    lines[first_table:first_table] = [f"gyro_gain = [{rows}]", f"gyro_negative_gain = [{', '.join(f'{c:.6g}' for c in neg)}]"]
    return "\n".join(lines) + "\n"


def write_params(geometry: Geometry, work_dir: pathlib.Path, data_dir: pathlib.Path) -> dict[str, pathlib.Path]:
    out = {}
    for n in rig.SENSORS:
        base = (data_dir.parent / rig.PARAMS[n]).read_text()
        out[n] = work_dir / f"params-{n}.toml"
        out[n].write_text(gyro_params(geometry, n, base))
    return out


def fit_hinge_one(sweeps: dict[str, list[capture.Capture]], name: str) -> np.ndarray:
    w = np.concatenate([rate(rig.stream(c, name)) for cs in sweeps.values() for c in cs])
    w = w[np.linalg.norm(w, axis=1) > MOVING_DPS]
    _, vecs = np.linalg.eigh(w.T @ w)
    return vecs[:, -1]


@dataclasses.dataclass(frozen=True)
class Rest:
    start: float  # board time, s
    end: float
    at_b: bool
    target_deg: float  # hinge angle the rest stands for
    level_deg: float  # integrated arm rate over the rest, for telling A from B without gravity


def rests(l9: capture.SensorStream, hinge: np.ndarray, orientation: str, up_a: np.ndarray, theta_rest: float) -> list[Rest]:
    """Still stretches of the arm, labelled A or B, each with its hinge angle: from its own gravity
    against the hold at A where gravity has an in-plane part, else 0 at A and theta_rest at B,
    B being the stop the integrated rate reaches turning positive. Consecutive stretches at the
    same stop merge."""
    t = rig.board_time(l9)
    theta = np.cumsum(rate(l9) @ hinge) / l9.odr_hz
    stills = rig.still(l9.gyro_dps, l9.odr_hz)
    levels = [float(theta[a:b].mean()) for a, b in stills]
    middle = (min(levels) + max(levels)) / 2 if levels else 0.0
    from_gravity = orientation not in GYRO_REST_ORIENTATIONS
    out: list[Rest] = []
    for (a, b), level in zip(stills, levels):
        if from_gravity:
            # Signed angles wrap at 180 and the stops are about that far apart: B may read -177.
            target = -rig.signed_angle(hinge, up_a, l9.accel_g[a:b].mean(axis=0)) % 360.0
            target = target - 360.0 if target > 270.0 else target
            at_b = target > theta_rest / 2
        else:
            at_b = level > middle
            target = theta_rest if at_b else 0.0
        if out and out[-1].at_b == at_b:
            out[-1] = dataclasses.replace(out[-1], end=float(t[b - 1]), level_deg=level)
        else:
            out.append(Rest(float(t[a]), float(t[b - 1]), at_b, float(target), level))
    return out


def hinge_angle(l9: capture.SensorStream, hinge: np.ndarray, rs: list[Rest]) -> np.ndarray:
    """The integrated arm rate, rescaled between consecutive rests so each rest reads its target."""
    t = rig.board_time(l9)
    raw = np.cumsum(rate(l9) @ hinge) / l9.odr_hz
    out = np.full(len(t), rs[0].target_deg if rs else 0.0)
    for r in rs:
        out[(t >= r.start) & (t <= r.end)] = r.target_deg
    for r0, r1 in zip(rs, rs[1:]):
        m = (t > r0.end) & (t < r1.start)
        i0, i1 = np.searchsorted(t, r0.end), np.searchsorted(t, r1.start)
        span = raw[i1] - raw[i0]
        if m.any() and abs(span) > 1e-6:
            out[m] = r0.target_deg + (raw[m] - raw[i0]) * (r1.target_deg - r0.target_deg) / span
    if rs:
        out[t > rs[-1].end] = rs[-1].target_deg
    return out


def sweep_up(s: capture.SensorStream, h: np.ndarray, t: np.ndarray, theta: np.ndarray, rs: list[Rest], params: dict) -> np.ndarray:
    """Up in the sensor's frame at times t: each rest's own corrected gravity, turned about the
    hinge between rests, with the out-of-plane change from one rest to the next spread over the
    turn (the loose arm settles and sinks as the sweeps go on)."""
    ts = rig.board_time(s)
    ups = [hold_up_of(s.accel_g[segment(ts, r.start, r.end)], params) for r in rs]
    out = np.empty((len(t), 3))
    out[:] = ups[0]
    for r, u in zip(rs, ups):
        out[(t >= r.start) & (t <= r.end)] = u
    for (r0, u0), (r1, u1) in zip(zip(rs, ups), zip(rs[1:], ups[1:])):
        m = (t > r0.end) & (t < r1.start)
        if not m.any():
            continue
        turned = rig.rotate(h, -(theta[m] - r0.target_deg), np.broadcast_to(u0, (int(m.sum()), 3)))
        end = rig.rotate(h, -(r1.target_deg - r0.target_deg), u0)
        axis = np.cross(end, u1)
        if np.linalg.norm(axis) > 1e-9:
            deg = np.degrees(np.arctan2(np.linalg.norm(axis), end @ u1))
            f = np.clip((theta[m] - r0.target_deg) / (r1.target_deg - r0.target_deg), 0.0, 1.0)
            turned = rig.rotate(axis, f * deg, turned)
        out[m] = turned
    out[t > rs[-1].end] = ups[-1]
    return out


def hold_up_of(accel_g: np.ndarray, params: dict) -> np.ndarray:
    a = np.linalg.solve(np.array(params.get("accel_gain", np.eye(3))), accel_g.mean(axis=0) - np.array(params.get("accel_offset", np.zeros(3))))
    return a / np.linalg.norm(a)


def sweep_inputs(
    cap: capture.Capture, geometry: Geometry, orientation: str, up_a_l9: np.ndarray, corrections: dict[str, dict]
) -> tuple[list[Rest], float, dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]]:
    """Each sensor's input for the replay, at palm_lsm9's samples from the earliest stream's start."""
    l9 = rig.stream(cap, "palm_lsm9")
    rs = rests(l9, geometry.hinge["palm_lsm9"], orientation, up_a_l9, sweep_rest_deg(geometry, orientation))
    t = rig.board_time(l9)
    theta = hinge_angle(l9, geometry.hinge["palm_lsm9"], rs)
    w = np.gradient(theta, t)
    alpha = np.radians(np.gradient(w, t))
    t_start = min(rig.board_time(rig.stream(cap, n))[0] for n in rig.SENSORS)
    inputs = {}
    for n in rig.SENSORS:
        h = geometry.hinge[n]
        up = sweep_up(rig.stream(cap, n), h, t, theta, rs, corrections[n])
        r = rig.RADIUS_M[n] * geometry.radial[n]
        kin = alpha[:, None] * np.cross(h, r) - (np.radians(w) ** 2)[:, None] * r
        inputs[n] = (t - t_start, up + kin / rig.G, w[:, None] * h)
    return rs, t_start, inputs


def segment(t: np.ndarray, a: float, b: float) -> slice:
    return slice(int(np.searchsorted(t, a)), int(np.searchsorted(t, b)))


@dataclasses.dataclass
class SweepMetrics:
    closure_deg: list[float] = dataclasses.field(default_factory=list)
    gyro_scale_deg: dict[str, list[float]] = dataclasses.field(default_factory=lambda: {"a_to_b": [], "b_to_a": []})
    lever_residual_g: list[float] = dataclasses.field(default_factory=list)


def sweep_metrics(s: capture.SensorStream, t_offset: float, hinge: np.ndarray, radial: np.ndarray | None,
                  radius_m: float, rs: list[Rest]) -> SweepMetrics:
    """Per sweep A->B->A: closure from the rest at A before to the rest at A after; per half sweep
    the integrated angle; with radial given, the RMS of radial accel + w^2 r, leaving out each stop
    contact until |w| stays under 5 dps for 0.2 s, and clipped samples. Rates are integrated
    against the zero level of the rest each stretch starts from."""
    t = rig.board_time(s) + t_offset
    w = s.gyro_dps @ hinge
    out = SweepMetrics()
    clipped = np.any(np.abs(s.accel_codes.astype(np.int32)) >= int(0.99 * 32768), axis=1)
    settle_n = max(1, int(SETTLE_S * s.odr_hz))

    def integrate(r0: Rest, r1: Rest) -> float:
        z = w[segment(t, r0.start, r0.end)].mean()
        return float(np.sum(w[segment(t, r0.end, r1.start)] - z) / s.odr_hz)

    for r0, r1 in zip(rs, rs[1:]):
        out.gyro_scale_deg["b_to_a" if r0.at_b else "a_to_b"].append(abs(integrate(r0, r1)))
    for r0, r1, r2 in zip(rs, rs[1:], rs[2:]):
        if r0.at_b or not r1.at_b or r2.at_b:
            continue
        out.closure_deg.append(integrate(r0, r2))
        if radial is None:
            continue
        seg = segment(t, r0.end, r2.start)
        z = w[segment(t, r0.start, r0.end)].mean()
        ws = w[seg] - z
        phi = np.cumsum(ws) / s.odr_hz
        keep = ~clipped[seg]
        b_start = int(np.searchsorted(t[seg], r1.start))
        b_end = int(np.searchsorted(t[seg], r1.end))
        for lo, arrival in ((0, b_start), (b_end, len(ws))):
            if arrival <= lo:
                continue
            near = lo + np.flatnonzero(np.abs(phi[lo:arrival] - phi[arrival - 1]) < CONTACT_DEG)
            if not len(near):
                continue
            calm = np.abs(ws) < SETTLE_DPS
            settled = next((j for j in range(near[0], len(ws) - settle_n) if calm[j : j + settle_n].all()), len(ws))
            keep[near[0] : settled] = False
        if keep.any():
            residual = s.accel_g[seg] @ radial + np.radians(ws) ** 2 * radius_m / rig.G
            out.lever_residual_g.append(float(np.sqrt(np.mean(residual[keep] ** 2))))
    return out


def mount_rms(cap: capture.Capture, geometry: Geometry, name: str) -> float:
    ref = rig.stream(cap, "palm_lsm9")
    s = rig.stream(cap, name)
    w = rate(s)
    w_l9 = rig.resample(rig.board_time(ref), rate(ref) @ geometry.mount[name], rig.board_time(s))
    moving = np.linalg.norm(w, axis=1) > MOVING_DPS
    return float(np.sqrt(np.mean(np.sum((w[moving] - w_l9[moving]) ** 2, axis=1))))


@dataclasses.dataclass(frozen=True)
class Score:
    sensor: str
    orientation: str
    metric: str
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool | None
    n_real: int


def score(sensor: str, orientation: str, metric: str, real: list[float], simulated: list[float], scored: bool = True) -> Score:
    r, s = np.array(real), np.array(simulated)
    sd = float(r.std(ddof=1)) if len(r) > 1 else 0.0
    base = metric.split(".")[0]
    tol = TOLERANCE[base] if base in TOLERANCE else RELATIVE[base] * abs(float(r.mean()))
    window = max(tol, 2 * sd)
    passed = bool(abs(s.mean() - r.mean()) <= window) if scored and len(s) else None
    return Score(sensor, orientation, metric, float(r.mean()), sd, float(s.mean()) if len(s) else np.nan, window, passed, len(r))


def run(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    *,
    report_sessions: typing.Callable[[int], bool] = lambda s: s > 1,
    log: rig.Log = print,
) -> list[Score]:
    geometry = fit_geometry(data_dir, log=log)
    work_dir.mkdir(parents=True, exist_ok=True)
    params = write_params(geometry, work_dir, data_dir)
    corrections = accel_params(data_dir)
    groups = by_kind([e for e in rig.entries(data_dir, "V4") if report_sessions(e.session)])
    sessions = sorted({k[0] for k in groups})
    session_1 = geometry
    moved = remounted(geometry, data_dir, log=log) if sessions and sessions[-1] >= REMOUNT_SESSION else geometry
    # (sensor, orientation, metric) -> real values, simulated values
    rows: dict[tuple[str, str, str], tuple[list[float], list[float]]] = {}

    def add(key: tuple[str, str, str], real: typing.Iterable[float], simulated: typing.Iterable[float]) -> None:
        r, s = rows.setdefault(key, ([], []))
        r.extend(real)
        s.extend(simulated)

    for session in sessions:
        geometry = moved if session >= REMOUNT_SESSION else session_1
        for o in ORIENTATIONS:
            a, b = groups.get((session, o, "hold_a")), groups.get((session, o, "hold_b"))
            if not a:
                continue
            cap_a = capture.read_capture(a[-1].path)
            up_a = hold_up(rig.stream(cap_a, "palm_lsm9"))
            # Static decomposition at B: each sensor's own hold at A, corrected by its V2 zero level
            # and gain, turned by the rest angle about its hinge axis and by its session-1 rocking.
            if b:
                cap_b = capture.read_capture(b[-1].path)
                for n in rig.SENSORS:
                    up = rig.rotate(geometry.hinge[n], -THETA_REST_DEG[o], hold_up(rig.stream(cap_a, n), corrections[n]))
                    if (n, o) in geometry.stop_b:
                        up = rig.rotate(*geometry.stop_b[(n, o)], up)
                    s = rig.stream(cap_b, n)
                    real = v2.array_hold_metrics(s.accel_g[s.longest_run(s.t_s >= v2.SETTLE_S)], up).tilt_deg
                    scen = work_dir / f"hold-s{session}-{o}-b-{n}.toml"
                    scen.write_text(v1.still_scenario(s.header, up, s.duration_s, session * 100 + len(rows), str(params[n].resolve())))
                    sim.run(scen, scen.with_suffix(".bin"), truth=False)
                    ss = rig.stream(capture.read_capture(scen.with_suffix(".bin")), "finger")
                    simulated = v2.array_hold_metrics(ss.accel_g[ss.longest_run(ss.t_s >= v2.SETTLE_S)], up).tilt_deg
                    add((n, o, "tilt_deg.b"), [real], [simulated])
            for k, e in enumerate(groups.get((session, o, "sweeps"), [])):
                cap = capture.read_capture(e.path)
                rs, t_start, inputs = sweep_inputs(cap, geometry, o, up_a, corrections)
                headers = {n: rig.stream(cap, n).header for n in rig.SENSORS}
                odrs = {n: rig.stream(cap, n).odr_hz for n in rig.SENSORS}
                sims = rig.simulate(work_dir, f"sweeps-s{session}-{o}-{k}", inputs, headers, odrs, session * 1000 + k, params)
                for n in rig.SENSORS:
                    radial = geometry.radial.get(n) if o == "hinge_vertical" else None
                    args = (geometry.hinge[n], radial, rig.RADIUS_M[n], rs)
                    real = sweep_metrics(rig.stream(cap, n), 0.0, *args)
                    simulated = sweep_metrics(sims[n], t_start, *args)
                    add((n, o, "closure_deg"), real.closure_deg, simulated.closure_deg)
                    for d in ("a_to_b", "b_to_a"):
                        add((n, o, f"gyro_scale_deg.{d}"), real.gyro_scale_deg[d], simulated.gyro_scale_deg[d])
                    if radial is not None:
                        add((n, o, "lever_residual_g"), real.lever_residual_g, simulated.lever_residual_g)
        for k, e in enumerate(groups.get((session, "", "base_rotation"), [])):
            cap = capture.read_capture(e.path)
            l9 = rig.stream(cap, "palm_lsm9")
            t = rig.board_time(l9)
            t_start = min(rig.board_time(rig.stream(cap, n))[0] for n in rig.SENSORS)
            w = rate(l9)
            inputs = {n: (t - t_start, l9.accel_g @ geometry.mount[n], w @ geometry.mount[n]) for n in rig.SENSORS}
            headers = {n: rig.stream(cap, n).header for n in rig.SENSORS}
            odrs = {n: rig.stream(cap, n).odr_hz for n in rig.SENSORS}
            sims = rig.simulate(work_dir, f"rotation-s{session}-{k}", inputs, headers, odrs, session * 1000 + 500 + k, params)
            scap = capture.Capture(cap.header, dict(enumerate(sims.values())), {}, [], 0, 0)
            # The re-mount is fitted on this capture, so here mount agreement is reported only.
            metric = "mount_rms_dps.remount_fit" if session == REMOUNT_SESSION else "mount_rms_dps"
            for n in ("palm", "finger"):
                add((n, "", metric), [mount_rms(cap, geometry, n)], [mount_rms(scap, geometry, n)])

    out = []
    for (n, o, metric), (real, simulated) in sorted(rows.items()):
        if not real:
            continue
        # Where the sweep rests come partly from the LSM6DSL's own integration, its gyro scale is
        # reported only.
        scored = not metric.endswith("remount_fit") and not (
            n == "palm" and o in GYRO_REST_ORIENTATIONS and metric.startswith("gyro_scale_deg")
        )
        out.append(score(n, o, metric, real, simulated, scored))
    for o in ORIENTATIONS + ("",):
        part = [s for s in out if s.orientation == o and s.passed is not None]
        if part:
            log(f"{o or 'base_rotation'}: {sum(bool(s.passed) for s in part)}/{len(part)} pass")
    return out


CSV_HEADER = "sensor,orientation,metric,real_mean,real_sd,sim_mean,window,pass,n_real"


def scores_csv(results: typing.Iterable[Score]) -> str:
    rows = [CSV_HEADER]
    for r in results:
        verdict = "" if r.passed is None else str(int(r.passed))
        rows.append(f"{r.sensor},{r.orientation},{r.metric},{r.real_mean:.6g},{r.real_sd:.6g},{r.sim_mean:.6g},{r.window:.6g},{verdict},{r.n_real}")
    return "\n".join(rows) + "\n"

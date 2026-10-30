"""V3 saturation: on the rig, the narrow sensors (lowest ranges) clip while a wide-range reference
on the same rigid arm sees the whole motion. The reference, mapped into each narrow sensor's frame
through the V4 mounts (palm_lsm9's refitted after its re-mount), moved to its position and
delayed, drives that sensor's chip model; clip level, wrap and gyro recovery of the simulated
events are scored against the real ones. Session 1 fits only the delays."""

import dataclasses
import pathlib
import typing

import numpy as np

from timusim import capture, rig, v4

REFERENCE = {"v3": "palm", "v3_board": "finger"}
NARROW = {"v3": ("finger", "palm_lsm9"), "v3_board": ("palm", "palm_lsm9")}
CLIP_CODE = int(0.99 * 32768)
MERGE_S = 0.5
RECOVERY_TOLERANCE = 2.0
# Half the 1% band an event's samples are counted in: the chips plateau a few codes to a few
# hundred below the largest code, where the model clips at it.
CLIP_LEVEL_TOLERANCE = 0.005 * 32768
DELAY_SEARCH_S = 0.02


@dataclasses.dataclass(frozen=True)
class Event:
    kind: str  # accel or gyro
    axis: int
    sign: int
    clip_level: float  # the code the output sits on while clipped
    wrapped: bool
    recovery: float  # samples, gyro only; nan when not measured


def events(s: capture.SensorStream, kind: str, ref_input: np.ndarray | None, full_scale: float) -> list[Event]:
    """Runs at or past 99% of full scale on one axis and sign, merged when under MERGE_S apart.
    ref_input is the reference's estimate of this sensor's input on its samples, in g or dps."""
    codes = (s.accel_codes if kind == "accel" else s.gyro_codes).astype(np.int32)
    out = []
    gap = int(MERGE_S * s.odr_hz)
    for axis in range(3):
        for sign in (1, -1):
            hit = sign * codes[:, axis] >= CLIP_CODE
            spans: list[list[int]] = []
            for a, b in rig.runs(hit):
                if spans and a - spans[-1][1] < gap:
                    spans[-1][1] = b
                else:
                    spans.append([a, b])
            for a, b in spans:
                clipped = codes[a:b, axis][sign * codes[a:b, axis] >= CLIP_CODE]
                past = None if ref_input is None else sign * ref_input[a:b, axis] > full_scale
                # A wrap is the output jumping to the opposite extreme while the input stays past
                # full scale; a shock at the stop can swing it through zero for a sample.
                wrapped = bool(past is not None and np.any(-sign * codes[a:b, axis][past] >= CLIP_CODE))
                recovery = np.nan
                if kind == "gyro" and ref_input is not None:
                    inside = np.flatnonzero(sign * ref_input[b - 1 :, axis] <= full_scale)
                    out_of = np.flatnonzero(sign * codes[b - 1 :, axis] < CLIP_CODE)
                    if len(inside) and len(out_of):
                        recovery = float(out_of[0] - inside[0])
                out.append(Event(kind, axis, sign, plateau(clipped, sign), wrapped, recovery))
    return out


def plateau(clipped: np.ndarray, sign: int) -> float:
    """The most frequent clipped code; a spike too short to repeat a code gives its extreme."""
    values, counts = np.unique(clipped, return_counts=True)
    if counts.max() > 1:
        return float(values[counts == counts.max()][-1 if sign > 0 else 0])
    return float(sign * np.max(sign * clipped))


@dataclasses.dataclass(frozen=True)
class Mapping:
    rotation: np.ndarray  # reference frame -> narrow frame
    delay_s: float  # narrow time = reference time + delay
    lever_m: np.ndarray  # narrow position minus reference position, in the reference frame


def reference_input(
    ref: capture.SensorStream, narrow: capture.SensorStream, m: Mapping, t: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray]:
    """The reference's estimate of the narrow sensor's specific force (g) and rate (dps) at board
    times t (default: the narrow sensor's samples)."""
    t = rig.board_time(narrow) if t is None else t
    t_ref = rig.board_time(ref) + m.delay_s
    w = ref.gyro_dps - rig.zero_level(ref.gyro_dps, ref.odr_hz)
    w_rad = np.radians(w)
    alpha = np.gradient(w_rad, t_ref, axis=0)
    lever = np.cross(alpha, m.lever_m) + np.cross(w_rad, np.cross(w_rad, m.lever_m))
    force = ref.accel_g + lever / rig.G
    return rig.resample(t_ref, force, t) @ m.rotation.T, rig.resample(t_ref, w, t) @ m.rotation.T


def fit_delay(ref: capture.SensorStream, narrow: capture.SensorStream, rotation: np.ndarray) -> float:
    """The delay minimising the RMS difference of the gyros over the narrow sensor's unclipped
    moving samples."""
    t = rig.board_time(narrow)
    w = narrow.gyro_dps - rig.zero_level(narrow.gyro_dps, narrow.odr_hz)
    ok = np.all(np.abs(narrow.gyro_codes.astype(np.int32)) < CLIP_CODE, axis=1) & (np.linalg.norm(w, axis=1) > 20)
    w_ref = (ref.gyro_dps - rig.zero_level(ref.gyro_dps, ref.odr_hz)) @ rotation.T
    delays = np.arange(-DELAY_SEARCH_S, DELAY_SEARCH_S, 1e-4)
    cost = [np.sqrt(np.mean((w[ok] - rig.resample(rig.board_time(ref) + d, w_ref, t[ok])) ** 2)) for d in delays]
    return float(delays[int(np.argmin(cost))])


def lever(geometry: v4.Geometry, ref: str, narrow: str) -> np.ndarray:
    """Narrow minus reference position in the reference's frame: both lie on the arm's radial line."""
    radial = geometry.radial[ref]
    return (rig.RADIUS_M[narrow] - rig.RADIUS_M[ref]) * radial


def mapping(geometry: v4.Geometry, ref: str, narrow: str, delay_s: float = 0.0) -> Mapping:
    rotation = geometry.mount[narrow].T @ geometry.mount[ref]
    return Mapping(rotation, delay_s, lever(geometry, ref, narrow))


@dataclasses.dataclass(frozen=True)
class Score:
    sensor: str
    config: str
    metric: str
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool | None  # None: reported, not scored
    n_real: int


def group(evs: list[Event]) -> dict[tuple[str, str, int], list[Event]]:
    out: dict[tuple[str, str, int], list[Event]] = {}
    for e in evs:
        out.setdefault((e.kind, "xyz"[e.axis], e.sign), []).append(e)
    return out


def score_events(sensor: str, config: str, real: list[Event], simulated: list[Event]) -> list[Score]:
    out = []
    rg, sg = group(real), group(simulated)
    for key in sorted(set(rg) | set(sg)):
        kind, axis, sign = key
        r, s = rg.get(key, []), sg.get(key, [])
        name = f"{kind}.{axis}{'+' if sign > 0 else '-'}"
        if not r or not s:
            out.append(Score(sensor, config, f"{name}.events", len(r), 0.0, len(s), 0.0, None, len(r)))
            continue
        rl, sl = np.array([e.clip_level for e in r]), np.array([e.clip_level for e in s])
        window = max(CLIP_LEVEL_TOLERANCE, 2 * float(rl.std()))
        out.append(Score(sensor, config, f"{name}.clip_level", float(np.median(rl)), float(rl.std()), float(np.median(sl)), window,
                         bool(abs(np.median(rl) - np.median(sl)) <= window), len(r)))
        rw, sw = any(e.wrapped for e in r), any(e.wrapped for e in s)
        out.append(Score(sensor, config, f"{name}.wrap", float(rw), 0.0, float(sw), 0.0, rw == sw, len(r)))
        rr = np.array([e.recovery for e in r if np.isfinite(e.recovery)])
        sr = np.array([e.recovery for e in s if np.isfinite(e.recovery)])
        if len(rr) and len(sr):
            sd = float(rr.std(ddof=1)) if len(rr) > 1 else 0.0
            window = max(RECOVERY_TOLERANCE, 2 * sd)
            # Accel recovery is reported, not scored.
            passed = bool(abs(sr.mean() - rr.mean()) <= window) if kind == "gyro" else None
            out.append(Score(sensor, config, f"{name}.recovery", float(rr.mean()), sd, float(sr.mean()), window, passed, len(rr)))
    return out


def full_scale(s: capture.SensorStream, kind: str) -> float:
    """Where the output stops: the largest code times the LSB, e.g. 286.7 dps at the 245 dps setting."""
    return 32768 * float(s.header["accel_lsb_g" if kind == "accel" else "gyro_lsb_dps"])


def run(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    *,
    report_sessions: typing.Callable[[int], bool] = lambda s: s > 1,
    log: rig.Log = print,
) -> list[Score]:
    geometry = v4.fit_geometry(data_dir, log=log)
    es = rig.entries(data_dir, "V3")
    work_dir.mkdir(parents=True, exist_ok=True)
    params = v4.write_params(geometry, work_dir, data_dir)
    delays: dict[tuple[str, str], float] = {}
    for e in (x for x in es if x.session == 1):
        cap = capture.read_capture(e.path)
        ref = REFERENCE[e.config]
        for narrow in NARROW[e.config]:
            m = mapping(geometry, ref, narrow)
            delays[(e.config, narrow)] = fit_delay(rig.stream(cap, ref), rig.stream(cap, narrow), m.rotation)
            log(f"{e.config} {narrow}: delay {1e3 * delays[(e.config, narrow)]:+.2f} ms behind {ref}")

    reported = [x for x in es if report_sessions(x.session)]
    session_1 = geometry
    moved = v4.remounted(geometry, data_dir, log=log) if any(x.session >= v4.REMOUNT_SESSION for x in reported) else geometry
    real: dict[tuple[str, str], list[Event]] = {}
    simulated: dict[tuple[str, str], list[Event]] = {}
    for k, e in enumerate(reported):
        geometry = moved if e.session >= v4.REMOUNT_SESSION else session_1
        cap = capture.read_capture(e.path)
        ref_name = REFERENCE[e.config]
        ref = rig.stream(cap, ref_name)
        inputs, headers, odrs = {}, {}, {}
        t_start = min(rig.board_time(rig.stream(cap, n))[0] for n in NARROW[e.config])
        for narrow in NARROW[e.config]:
            s = rig.stream(cap, narrow)
            m = mapping(geometry, ref_name, narrow, delays.get((e.config, narrow), 0.0))
            force, rate = reference_input(ref, s, m)
            for kind, est in (("accel", force), ("gyro", rate)):
                real.setdefault((narrow, e.config), []).extend(events(s, kind, est, full_scale(s, kind)))
            # The sim samples the input on its own clock, so it gets the reference's own samples.
            t_ref = rig.board_time(ref) + m.delay_s
            f_ref, w_ref = reference_input(ref, s, m, t_ref)
            inputs[narrow] = (t_ref - t_start, f_ref, w_ref)
            headers[narrow], odrs[narrow] = s.header, s.odr_hz
        sims = rig.simulate(work_dir, f"{e.config}-s{e.session}-{k}", inputs, headers, odrs, 1000 * k + 1, params)
        for narrow, s in sims.items():
            m = mapping(geometry, ref_name, narrow, delays.get((e.config, narrow), 0.0))
            t_sim = rig.board_time(s) + t_start
            force, rate = (rig.resample(inputs[narrow][0] + t_start, v, t_sim) for v in inputs[narrow][1:])
            for kind, est in (("accel", force), ("gyro", rate)):
                simulated.setdefault((narrow, e.config), []).extend(events(s, kind, est, full_scale(s, kind)))
    out = []
    for key in sorted(real):
        scores = score_events(*key, real[key], simulated.get(key, []))
        scored = [s for s in scores if s.passed is not None]
        log(f"{key[1]} {key[0]}: {sum(bool(s.passed) for s in scored)}/{len(scored)} pass, {len(real[key])} real events")
        out.extend(scores)
    return out


CSV_HEADER = "sensor,config,metric,real_mean,real_sd,sim_mean,window,pass,n_real"


def scores_csv(results: typing.Iterable[Score]) -> str:
    rows = [CSV_HEADER]
    for r in results:
        verdict = "" if r.passed is None else str(int(r.passed))
        rows.append(f"{r.sensor},{r.config},{r.metric},{r.real_mean:.6g},{r.real_sd:.6g},{r.sim_mean:.6g},{r.window:.6g},{verdict},{r.n_real}")
    return "\n".join(rows) + "\n"

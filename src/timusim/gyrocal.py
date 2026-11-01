"""Gyro gain calibration (GyroCal): turns between still poses, each checked against gravity from
the V2-calibrated accelerometer. Per sensor, a reading is m = G N(w) + b, with N the per-axis gain
on negative rates; G and N are fitted on the first capture of a session and checked on the second
with nothing refitted."""

import dataclasses
import pathlib
import sys
import tomllib

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from timusim import capture, fit, manifest, rig, v4

TEST = "GyroCal"
STILL_S = 2.0
# Gaps in a still shorter than this split it without a turn: such blips are left out.
MIN_TURN_DPS = 10.0
# The procedure's pace: faster turns are dropped.
MAX_TURN_DPS = 150.0
TOLERANCE = {"median_deg": 0.5, "max_deg": 2.0}
WORK_DIR = "gyrocal"


@dataclasses.dataclass
class Turn:
    t_s: float  # start, from the capture's first sample
    rate_dps: np.ndarray  # m - b over the turn
    dt_s: float
    up0: np.ndarray  # V2-corrected up at the still before
    up1: np.ndarray  # and after


def turns(s: capture.SensorStream, params: dict) -> list[Turn]:
    """Each turn between two stills, without the blips between stills, turns over the
    procedure's pace, and turns that lose samples (along with the stills those fall in)."""
    stills = rig.still(s.gyro_dps, s.odr_hz, min_s=STILL_S)
    out = []
    for (a0, a1), (b0, b1) in zip(stills, stills[1:]):
        if (np.diff(s.index[a0:b1]) != 1).any():
            continue
        bias0, bias1 = s.gyro_dps[a0:a1].mean(0), s.gyro_dps[b0:b1].mean(0)
        f = np.linspace(0.0, 1.0, b0 - a1)[:, None]
        rate = s.gyro_dps[a1:b0] - (bias0 * (1 - f) + bias1 * f)
        if not MIN_TURN_DPS <= np.abs(rate).max() <= MAX_TURN_DPS:
            continue
        out.append(Turn(float(s.t_s[a1]), rate, 1.0 / s.odr_hz,
                        v4.up_from(s.accel_g[a0:a1], params), v4.up_from(s.accel_g[b0:b1], params)))
    return out


def true_rate(rate: np.ndarray, gain: np.ndarray, negative: np.ndarray) -> np.ndarray:
    """w from m - b = G N(w)."""
    w = np.linalg.solve(gain, rate.T).T
    return np.where(w < 0, w / negative, w)


def chain(r: Rotation) -> Rotation:
    """r[0] r[1] ... r[n-1], pairwise."""
    while len(r) > 1:
        if len(r) % 2:
            r = Rotation.concatenate([r, Rotation.identity()])
        r = r[0::2] * r[1::2]
    return r[0]


def predict_up(turn: Turn, gain: np.ndarray, negative: np.ndarray) -> np.ndarray:
    """The still before's up carried through the turn, in the frame of the still after."""
    w = true_rate(turn.rate_dps, gain, negative)
    return chain(Rotation.from_rotvec(np.radians(w) * turn.dt_s)).inv().apply(turn.up0)


def misfit_deg(turn: Turn, gain: np.ndarray, negative: np.ndarray) -> float:
    c = np.clip(predict_up(turn, gain, negative) @ turn.up1, -1.0, 1.0)
    return float(np.degrees(np.arccos(c)))


def _residuals(x: np.ndarray, ts: list[Turn]) -> np.ndarray:
    gain, negative = x[:9].reshape(3, 3), x[9:]
    # The misfit as a small rotation of the observed up, in degrees.
    return np.concatenate([np.degrees(np.cross(t.up1, predict_up(t, gain, negative))) for t in ts])


def fit_gain(ts: list[Turn]) -> tuple[np.ndarray, np.ndarray]:
    x0 = np.concatenate([np.eye(3).ravel(), np.ones(3)])
    r = least_squares(_residuals, x0, args=(ts,), x_scale=0.01)
    return r.x[:9].reshape(3, 3), r.x[9:]


def still_accel(s: capture.SensorStream) -> np.ndarray:
    """Mean accel of each still."""
    return np.array([s.accel_g[a:b].mean(0) for a, b in rig.still(s.gyro_dps, s.odr_hz, min_s=STILL_S)])


def _symmetric(x: np.ndarray) -> np.ndarray:
    return np.array([[x[0], x[3], x[4]], [x[3], x[1], x[5]], [x[4], x[5], x[2]]])


def refit_accel(params: dict, means: np.ndarray) -> dict:
    """The V2 accel correction refitted on these stills by |a| = 1: a symmetric gain and an
    offset on top of V2, so the frame stays V2's."""
    gain, offset = np.array(params["accel_gain"]), np.array(params["accel_offset"])
    corrected = np.linalg.solve(gain, (means - offset).T).T

    def residuals(x: np.ndarray) -> np.ndarray:
        return np.linalg.norm(corrected @ (np.eye(3) + _symmetric(x)).T - x[6:], axis=1) - 1.0

    x = least_squares(residuals, np.zeros(9)).x
    new_gain = gain @ np.linalg.inv(np.eye(3) + _symmetric(x))
    return dict(params, accel_gain=new_gain.tolist(), accel_offset=(offset + new_gain @ x[6:]).tolist())


def norm_sd_mg(params: dict, means: np.ndarray) -> float:
    corrected = np.linalg.solve(np.array(params["accel_gain"]), (means - np.array(params["accel_offset"])).T)
    return 1000.0 * float(np.linalg.norm(corrected, axis=0).std())


@dataclasses.dataclass
class SensorResult:
    sensor: str
    gain: np.ndarray
    negative: np.ndarray
    accel: dict  # the session's accel correction
    accel_spread_mg: tuple[float, float]  # SD of |a| at the stills, V2 and refitted
    fitted: list[float]  # per-turn misfit on the fitted capture, deg
    checked: list[float]  # and on the check capture
    identity: list[float]  # the check capture at unit gain

    @property
    def passed(self) -> bool:
        return bool(np.median(self.checked) < TOLERANCE["median_deg"] and max(self.checked) <= TOLERANCE["max_deg"])


def params_text(base: str, gain: np.ndarray, negative: np.ndarray) -> str:
    return fit.with_top_level(base, [f"gyro_gain = {fit.matrix(gain)}", f"gyro_negative_gain = {fit.vector(negative)}"])


def sessions(data_dir: pathlib.Path) -> dict[int, list[rig.Entry]]:
    """Per session, [fitted, checked], as the capture notes name them."""
    out: dict[int, list[rig.Entry]] = {}
    for role in ("(fitted", "(checked"):
        for e in rig.entries(data_dir, TEST):
            if role in e.notes:
                out.setdefault(e.session, []).append(e)
    return out


def calibrate(data_dir: pathlib.Path, session: int = 1, *, log: rig.Log = print) -> list[SensorResult]:
    es = sessions(data_dir).get(session, [])
    if len(es) < 2:
        raise RuntimeError(f"GyroCal session {session}: {len(es)} capture(s), need a fitted and a checked one")
    fitted, checked = capture.read_capture(es[0].path), capture.read_capture(es[1].path)
    v2_params = v4.accel_params(data_dir)
    out = []
    for n in rig.SENSORS:
        means = np.vstack([still_accel(fitted.stream(n)), still_accel(checked.stream(n))])
        accel = refit_accel(v2_params[n], means)
        spread = (norm_sd_mg(v2_params[n], means), norm_sd_mg(accel, means))
        ft, ct = turns(fitted.stream(n), accel), turns(checked.stream(n), accel)
        gain, negative = fit_gain(ft)
        r = SensorResult(
            n, gain, negative, accel, spread,
            [misfit_deg(t, gain, negative) for t in ft],
            [misfit_deg(t, gain, negative) for t in ct],
            [misfit_deg(t, np.eye(3), np.ones(3)) for t in ct],
        )
        log(f"{n}: {len(ft)} fitted turns, {len(ct)} checked; check median {np.median(r.checked):.3f} deg, "
            f"max {max(r.checked):.3f} deg (unit gain: median {np.median(r.identity):.3f}, max {max(r.identity):.3f})")
        out.append(r)
    return out


def report(results: list[SensorResult]) -> str:
    lines = [f"GyroCal: acceptance on the check capture, median < {TOLERANCE['median_deg']} deg, max <= {TOLERANCE['max_deg']} deg", ""]
    for r in results:
        lines += [
            f"{r.sensor}: {'PASS' if r.passed else 'FAIL'}",
            f"  gyro_gain = {fit.matrix(r.gain)}",
            f"  gyro_negative_gain = {fit.vector(r.negative)}",
            f"  accel refitted on the stills: |a| SD {r.accel_spread_mg[0]:.1f} mg (V2) -> {r.accel_spread_mg[1]:.1f} mg",
            f"  accel_gain = {fit.matrix(np.array(r.accel['accel_gain']))}",
            f"  accel_offset = {fit.vector(r.accel['accel_offset'])}",
            f"  fitted capture: {len(r.fitted)} turns, median {np.median(r.fitted):.3f}, max {max(r.fitted):.3f} deg",
            f"  check capture:  {len(r.checked)} turns, median {np.median(r.checked):.3f}, max {max(r.checked):.3f} deg",
            f"  check at unit gain: median {np.median(r.identity):.3f}, max {max(r.identity):.3f} deg",
            "  check misfits: " + " ".join(f"{m:.2f}" for m in r.checked),
            "",
        ]
    return "\n".join(lines)


def write(results: list[SensorResult], data_dir: pathlib.Path, work_dir: pathlib.Path) -> None:
    work_dir.mkdir(parents=True, exist_ok=True)
    for r in results:
        base = (data_dir / rig.PARAMS[r.sensor]).read_text()
        (work_dir / f"params-{r.sensor}.toml").write_text(params_text(base, r.gain, r.negative))
    (work_dir / "report.txt").write_text(report(results))


def load(work_dir: pathlib.Path, sensor: str) -> tuple[np.ndarray, np.ndarray]:
    p = tomllib.loads((work_dir / f"params-{sensor}.toml").read_text())
    return np.array(p["gyro_gain"]), np.array(p["gyro_negative_gain"])


def run(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help=f"default data/{WORK_DIR}")
    parser.add_argument("--session", type=int, default=1)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    work_dir = args.work_dir or args.data_dir / WORK_DIR
    results = calibrate(args.data_dir, args.session)
    write(results, args.data_dir, work_dir)
    print(report(results))
    return 0 if all(r.passed for r in results) else 1

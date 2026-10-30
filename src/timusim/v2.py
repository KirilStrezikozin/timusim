"""V2 static geometry: the accelerometer held still at known orientations. The session-1 holds fit
the zero level and the 3x3 gain on top of the V1 chip parameters; every reported hold is simulated
at its declared orientation and scored with the RESEARCH.md acceptance rule."""

import dataclasses
import pathlib
import typing

import numpy as np

from timusim import capture, manifest, sim, v1

# Dropped from the start of each hold, so taping and letting go do not enter the mean.
SETTLE_S = 30.0
TILT_TOLERANCE_DEG = 1.0
NORM_TOLERANCE_G = 0.010
AXES = {
    "+x": (1.0, 0.0, 0.0),
    "-x": (-1.0, 0.0, 0.0),
    "+y": (0.0, 1.0, 0.0),
    "-y": (0.0, -1.0, 0.0),
    "+z": (0.0, 0.0, 1.0),
    "-z": (0.0, 0.0, -1.0),
}


def parse_up(spec: str) -> tuple[float, float, float]:
    """World up in the sensor frame: an axis such as "+z" or "-x", or "x,y,z"."""
    v = np.array(AXES[spec] if spec in AXES else [float(c) for c in spec.split(",")])
    if v.shape != (3,) or not np.linalg.norm(v) > 0:
        raise ValueError(f"up direction {spec!r}: expected +x, -x, +y, -y, +z, -z or x,y,z")
    return tuple(float(c) for c in v / np.linalg.norm(v))  # type: ignore[return-value]


def position(up: typing.Sequence[float]) -> str:
    return next((name for name, axis in AXES.items() if np.allclose(up, axis)), ",".join(f"{c:.3f}" for c in up))


@dataclasses.dataclass(frozen=True)
class Hold:
    path: pathlib.Path
    session: int
    up: tuple[float, float, float]

    @property
    def position(self) -> str:
        return position(self.up)


def entries(data_dir: pathlib.Path = manifest.DATA_DIR, sensor: str = v1.SENSOR) -> list[Hold]:
    """V2 captures whose manifest entry gives this sensor's up direction."""
    out = []
    for e in manifest.load(data_dir / manifest.MANIFEST_PATH.name):
        up = e.get("sensors", {}).get(sensor, {}).get("up")
        if e.get("test") == "V2" and up is not None and v1.usable(e, sensor):
            out.append(Hold(data_dir / e["file"], int(e["session"]), parse_up(",".join(map(str, up)))))
    return out


@dataclasses.dataclass(frozen=True)
class HoldMetrics:
    mean_g: np.ndarray
    tilt_deg: float
    norm_error_g: float


def hold_metrics(cap: capture.Capture, sensor: str, up: typing.Sequence[float], *, trim_s: float = SETTLE_S) -> HoldMetrics:
    s = next(x for x in cap.sensors.values() if x.name == sensor)
    return array_hold_metrics(s.accel_g[s.longest_run(s.t_s >= trim_s)], up)


def array_hold_metrics(accel_g: np.ndarray, up: typing.Sequence[float]) -> HoldMetrics:
    """Tilt error is the angle between the mean accel and up; norm error is |mean accel| - 1 g."""
    mean = accel_g.mean(axis=0)
    norm = float(np.linalg.norm(mean))
    cos = float(np.clip(mean @ np.asarray(up) / norm, -1.0, 1.0))
    return HoldMetrics(mean, float(np.degrees(np.arccos(cos))), norm - 1.0)


def fit_accel(means: np.ndarray, ups: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Zero level b and gain G of mean = G up + b, least squares over the holds. Needs at least
    four up directions that do not lie in one plane, as the six axis positions give."""
    design = np.column_stack([ups, np.ones(len(ups))])
    if np.linalg.matrix_rank(design) < 4:
        raise ValueError("the holds' up directions lie in one plane; the gain is not determined")
    x, *_ = np.linalg.lstsq(design, means, rcond=None)
    return x[3], x[:3].T


def with_accel(params: str, offset: np.ndarray, gain: np.ndarray) -> str:
    """The parameter file with accel_offset and accel_gain set; they go before the first table."""
    lines = [ln for ln in params.splitlines() if not ln.startswith(("accel_offset", "accel_gain"))]
    first_table = next((i for i, ln in enumerate(lines) if ln.startswith("[")), len(lines))
    rows = ", ".join("[" + ", ".join(f"{c:.6g}" for c in row) + "]" for row in gain)
    lines[first_table:first_table] = [
        f"accel_offset = [{', '.join(f'{c:.6g}' for c in offset)}]",
        f"accel_gain = [{rows}]",
    ]
    return "\n".join(lines) + "\n"


@dataclasses.dataclass(frozen=True)
class Score:
    position: str
    metric: str
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool


def score(position_: str, metric: str, real: list[float], simulated: list[float], tolerance: float) -> Score:
    r, s = np.array(real), np.array(simulated)
    sd = float(r.std(ddof=1)) if len(r) > 1 else 0.0
    window = max(tolerance, 2 * sd)
    passed = bool(abs(s.mean() - r.mean()) <= window)
    return Score(position_, metric, float(r.mean()), sd, float(s.mean()), window, passed)


def run(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    base_params: pathlib.Path,
    *,
    sensor: str = v1.SENSOR,
    sims_per_hold: int = 1,
    log: typing.Callable[[str], None] = print,
) -> list[Score]:
    holds = entries(data_dir, sensor)
    fit_set = [h for h in holds if h.session == 1]
    reported = [h for h in holds if h.session > 1]
    if not fit_set:
        raise SystemExit("no V2 session-1 holds in the manifest")

    work_dir.mkdir(parents=True, exist_ok=True)
    means = np.array([hold_metrics(capture.read_capture(h.path), sensor, h.up).mean_g for h in fit_set])
    offset, gain = fit_accel(means, np.array([h.up for h in fit_set]))
    params_path = work_dir / base_params.name
    params_path.write_text(with_accel(base_params.read_text(), offset, gain))
    log(f"accel zero level and gain from {len(fit_set)} session-1 holds -> {params_path}")

    # The sim has palm and finger slots, both running this chip; palm_lsm9 takes the finger's.
    slot = "finger" if sensor == "palm_lsm9" else sensor
    rows: Rows = {}
    for k, h in enumerate(reported):
        cap = capture.read_capture(h.path)
        s = next(x for x in cap.sensors.values() if x.name == sensor)
        simulated = []
        for j in range(sims_per_hold):
            scen = work_dir / f"hold-{k}-{j}.toml"
            scen.write_text(v1.still_scenario(s.header, np.array(h.up), s.duration_s, 1000 * k + j + 1, params_path.name))
            out = work_dir / f"hold-{k}-{j}.bin"
            sim.run(scen, out, truth=False)
            simulated.append(hold_metrics(capture.read_capture(out), slot, h.up))
        add_row(rows, h.position, hold_metrics(cap, sensor, h.up), simulated)

    return score_rows(rows, log)


# Per position and metric: the real values of the reported holds and the simulated ones.
Rows = dict[str, dict[str, tuple[list[float], list[float]]]]


def add_row(rows: Rows, position_: str, real: HoldMetrics, simulated: typing.Iterable[HoldMetrics]) -> None:
    per = rows.setdefault(position_, {"tilt_deg": ([], []), "norm_error_g": ([], [])})
    per["tilt_deg"][0].append(real.tilt_deg)
    per["norm_error_g"][0].append(real.norm_error_g)
    for m in simulated:
        per["tilt_deg"][1].append(m.tilt_deg)
        per["norm_error_g"][1].append(m.norm_error_g)


def score_rows(rows: Rows, log: typing.Callable[[str], None] = print) -> list[Score]:
    results = []
    for pos, metrics in sorted(rows.items()):
        results.append(score(pos, "tilt_deg", *metrics["tilt_deg"], TILT_TOLERANCE_DEG))
        results.append(score(pos, "norm_error_g", *metrics["norm_error_g"], NORM_TOLERANCE_G))
        verdicts = ", ".join(f"{r.metric} {'pass' if r.passed else 'FAIL'}" for r in results[-2:])
        log(f"{pos}: {verdicts} ({len(metrics['tilt_deg'][0])} reported holds)")
    return results


CSV_HEADER = "position,metric,real_mean,real_sd,sim_mean,window,pass"


def scores_csv(results: typing.Iterable[Score]) -> str:
    rows = [CSV_HEADER]
    for r in results:
        rows.append(f"{r.position},{r.metric},{r.real_mean:.6g},{r.real_sd:.6g},{r.sim_mean:.6g},{r.window:.6g},{int(r.passed)}")
    return "\n".join(rows) + "\n"


def read_scores(path: pathlib.Path) -> list[Score]:
    out = []
    for line in path.read_text().splitlines()[1:]:
        pos, metric, real_mean, real_sd, sim_mean, window, passed = line.split(",")
        out.append(Score(pos, metric, float(real_mean), float(real_sd), float(sim_mean), float(window), passed == "1"))
    return out

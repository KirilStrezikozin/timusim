"""The V3/V4 rig: three sensors on one arm that turns on a pivot between two stops. Shared
geometry, stream helpers and the replay scenario that feeds a sensor-frame input through a chip
model. Frames: each sensor's IIMU frame; mounts map a sensor's frame into palm_lsm9's."""

import dataclasses
import pathlib
import typing

import numpy as np

from timusim import capture, manifest, sim, v1

SENSORS = ("palm", "finger", "palm_lsm9")
# Pivot axis to chip centre, measured with a ruler (±1 mm).
RADIUS_M = {"palm": 0.15, "palm_lsm9": 0.15, "finger": 0.185}
THETA_CONTACT_DEG = 171.0
G = 9.80665
# Chip parameters per sensor, in the data directory: V2 adds each unit's accel zero level and gain
# to V1's. palm_lsm9 has no V1 of its own: its V2 sits on the finger's V1 LSM9DS1 parameters.
PARAMS = {"finger": "v2/lsm9ds1.toml", "palm": "v2-palm/lsm6dsl.toml", "palm_lsm9": "v2-palm_lsm9/lsm9ds1.toml"}
STILL_DPS = 3.0
# A code at or past 99% of full scale counts as clipped.
CLIP_CODE = int(0.99 * 32768)


def board_time(s: capture.SensorStream) -> np.ndarray:
    return s.t_s + s.t0_s


def first_sample_s(cap: capture.Capture, names: typing.Iterable[str] = SENSORS) -> float:
    """Board time of the earliest first sample among these sensors."""
    return min(board_time(cap.stream(n))[0] for n in names)


def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Half-open index ranges where mask is true."""
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.flatnonzero(d == 1).tolist(), np.flatnonzero(d == -1).tolist()))


def still(gyro_dps: np.ndarray, odr_hz: float, *, min_s: float = 0.5, threshold_dps: float = STILL_DPS) -> list[tuple[int, int]]:
    """Stretches of at least min_s where every axis stays within threshold of the median rate."""
    ok = np.all(np.abs(gyro_dps - np.median(gyro_dps, axis=0)) < threshold_dps, axis=1)
    return [(a, b) for a, b in runs(ok) if (b - a) / odr_hz >= min_s]


def zero_level(gyro_dps: np.ndarray, odr_hz: float) -> np.ndarray:
    """Mean rate over the still stretches, the gyro's zero level for that capture."""
    rests = still(gyro_dps, odr_hz)
    if not rests:
        return np.median(gyro_dps, axis=0)
    return np.concatenate([gyro_dps[a:b] for a, b in rests]).mean(axis=0)


def resample(t_from: np.ndarray, values: np.ndarray, t_to: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(t_to, t_from, values[:, k]) for k in range(values.shape[1])])


def fit_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """R minimising |a - R b| over rows (Kabsch): R maps b's frame into a's."""
    u, _, vt = np.linalg.svd(a.T @ b)
    d = np.sign(np.linalg.det(u @ vt))
    return u @ np.diag([1.0, 1.0, d]) @ vt


def rotate(axis: np.ndarray, deg: np.ndarray | float, v: np.ndarray) -> np.ndarray:
    """v rotated by deg about the unit axis (Rodrigues); deg may be an array, one per row."""
    th = np.radians(np.asarray(deg, dtype=float))[..., None]
    k = axis / np.linalg.norm(axis)
    return v * np.cos(th) + np.cross(k, v) * np.sin(th) + k * (v @ k)[..., None] * (1 - np.cos(th))


def signed_angle(axis: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Angle in degrees from a to b about axis, both projected onto the plane normal to it."""
    k = axis / np.linalg.norm(axis)
    pa, pb = a - (a @ k) * k, b - (b @ k) * k
    return float(np.degrees(np.arctan2(np.cross(pa, pb) @ k, pa @ pb)))


@dataclasses.dataclass(frozen=True)
class Entry:
    path: pathlib.Path
    test: str
    config: str
    session: int
    notes: str


def entries(data_dir: pathlib.Path, test: str) -> list[Entry]:
    out = []
    for e in manifest.load(manifest.path(data_dir)):
        if e.get("test") == test and all(v1.usable(e, s) for s in SENSORS):
            out.append(Entry(data_dir / e["file"], test, e["config"], int(e["session"]), e.get("notes", "")))
    return out


def write_input(path: pathlib.Path, t_s: np.ndarray, force_g: np.ndarray, rate_dps: np.ndarray) -> None:
    rows = np.column_stack([t_s, force_g, rate_dps])
    np.savetxt(path, rows, delimiter=",", fmt="%.7g", header="t_s,fx_g,fy_g,fz_g,wx_dps,wy_dps,wz_dps", comments="")


@dataclasses.dataclass(frozen=True)
class Slot:
    """One simulated sensor: its capture header (chip config), measured ODR, input CSV and
    parameter file (none: the ideal chip), paths relative to the scenario's directory."""

    header: dict[str, str]
    odr_hz: float
    input: str
    params: str | None = None


def replay_scenario(palm: Slot, finger: Slot, seconds: float, seed: int) -> str:
    def block(name: str, s: Slot) -> str:
        nominal = float(s.header["odr_hz"])
        params = f'params = "{s.params}"\n' if s.params else ""
        return (
            f"[{name}]\nchip = {v1.chip_table(s.header)}\n{params}"
            f"odr_scale = {s.odr_hz / nominal:.9f}\ninput = \"{s.input}\"\n"
        )

    mount = "{ axis = [1.0, 0.0, 0.0], deg = 0.0 }"
    return sim.point_scenario("rig-replay", seed, mount) + f"""{block("palm", palm)}{block("finger", finger)}[[moves]]
hold_s = {seconds:.3f}
"""


def simulate(
    work_dir: pathlib.Path,
    stem: str,
    inputs: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]],
    real: dict[str, capture.SensorStream],
    seed: int,
    params: dict[str, pathlib.Path],
) -> dict[str, capture.SensorStream]:
    """Runs each named sensor's input (t_s from 0, force in g, rate in dps) through its chip model
    with its parameter file, at the chip config and measured ODR of its real stream, two sensors
    per simulator run, and returns the simulated streams by name."""
    names = list(inputs)
    out: dict[str, capture.SensorStream] = {}
    for k in range(0, len(names), 2):
        pair = names[k : k + 2]
        slots = []
        for name in pair:
            csv = work_dir / f"{stem}-{name}.csv"
            write_input(csv, *inputs[name])
            slots.append(Slot(real[name].header, real[name].odr_hz, csv.name, str(params[name].resolve())))
        seconds = max(float(inputs[n][0][-1]) for n in pair)
        scen = work_dir / f"{stem}-{k // 2}.toml"
        # A lone sensor fills both slots; the second copy is ignored.
        scen.write_text(replay_scenario(slots[0], slots[-1], seconds, seed + k))
        bin_path = scen.with_suffix(".bin")
        sim.run(scen, bin_path, truth=False)
        scap = capture.read_capture(bin_path)
        for name, slot in zip(pair, ("palm", "finger")):
            out[name] = scap.stream(slot)
            out[name].header = dict(out[name].header, name=name)
    return out


Log = typing.Callable[[str], None]


@dataclasses.dataclass(frozen=True)
class Score:
    sensor: str
    group: str  # V3: the config; V4: the orientation, empty for the base rotation
    metric: str
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool | None  # None: reported, not scored
    n_real: int


def scores_csv(results: typing.Iterable[Score], group: str) -> str:
    rows = [f"sensor,{group},metric,real_mean,real_sd,sim_mean,window,pass,n_real"]
    for r in results:
        verdict = "" if r.passed is None else str(int(r.passed))
        rows.append(f"{r.sensor},{r.group},{r.metric},{r.real_mean:.6g},{r.real_sd:.6g},{r.sim_mean:.6g},{r.window:.6g},{verdict},{r.n_real}")
    return "\n".join(rows) + "\n"

"""V1 bench-still comparison: one parameter set per chip, fitted on session 1, simulated for
every config and scored against the reported sessions with the RESEARCH.md acceptance rule. The
finger is the LSM9DS1 and the palm the LSM6DSL; both go through the same fitting procedure."""

import concurrent.futures
import dataclasses
import pathlib
import typing

import numpy as np

from timusim import acceptance, bench, capture, fit, manifest, sim

SENSOR = "finger"
CHIP = {"finger": "lsm9ds1", "palm": "lsm6dsl", "palm_lsm9": "lsm9ds1"}
# A capture that lost less than this much of the sensor's stream, or for long holds this share of
# it, is still used, as its longest gap-free stretch.
MAX_MISSING_S = 1.0
MAX_MISSING_SHARE = 1e-3
TOLERANCE = {"bias": 0.20, "std": 0.20, "noise_floor": 0.20, "low_freq": 0.20, "cutoff_hz": 0.10, "allan": 0.25}
# Spectral levels compare in dB.
DB_METRICS = ("noise_floor", "low_freq")
ODR_CHOICE = {119: "119", 238: "238", 476: "476", 952: "952"}
ACCEL_BW = {0x00: "off", 0x10: "odr_div_50", 0x11: "odr_div_100", 0x12: "odr_div_9", 0x13: "odr_div_400"}
GYRO_BW = {0: "strong", 1: "medium", 2: "light", 3: "ultra_light"}
LSM6DSL_GYRO_BW = {0x00: "lp2_only", 0x08: "lp1_strong", 0x09: "lp1_normal", 0x0A: "lp1_light", 0x0B: "lp1_aggressive"}
LSM6DSL_ACCEL_BW = {
    latency | divider: f"{name}_odr_div_{div}"
    for latency, name in ((0x00, "low_latency"), (0x10, "low_noise"))
    for divider, div in ((0x00, 50), (0x01, 100), (0x02, 9), (0x03, 400))
}


@dataclasses.dataclass(frozen=True)
class Entry:
    path: pathlib.Path
    config: str
    session: int
    power_up: int = 0
    # The board turned during the hold (manifest `accel_tilted`), or the chip's accel was not in its
    # usual state (`accel_noisy`): this accel drift is not the chip's.
    accel_excluded: bool = False


def sensor_config(config: str, sensor: str) -> str:
    """Each capture changes one sensor's setting and runs the others at their default, or changes
    one of each, joined by "+"; palm sweep configs are named palm_<setting> and palm_lsm9 sweep
    configs palm_lsm9_<setting>."""
    parts = config.split("+")
    lsm9 = next((p for p in parts if p.startswith("palm_lsm9_")), None)
    palm = next((p for p in parts if p.startswith("palm_") and p != lsm9), None)
    finger = next((p for p in parts if not p.startswith("palm_")), None)
    own = {"palm_lsm9": lsm9 and lsm9.removeprefix("palm_lsm9_"), "palm": palm and palm.removeprefix("palm_")}
    if sensor in own:
        return own[sensor] or (config if config == "long" else "default")
    return finger or "default"


def usable(entry: dict[str, typing.Any], sensor: str) -> bool:
    """A manifest `excluded` reason drops a capture whose data is intact but disturbed."""
    sensors = entry.get("sensors")
    # Captures from before a sensor existed list the others only.
    if entry.get("excluded") or (sensors is not None and sensor not in sensors):
        return False
    stats = (sensors or {}).get(sensor, {})
    return bool(entry.get("clean")) or (
        stats.get("overruns", 1) == 0
        and stats.get("missing_samples", 1e9)
        <= max(MAX_MISSING_S * stats.get("odr_measured_hz", 0.0), MAX_MISSING_SHARE * stats.get("samples", 0))
    )


def entries(data_dir: pathlib.Path = manifest.DATA_DIR, test: str = "V1", sensor: str = SENSOR) -> list[Entry]:
    out = []
    for e in manifest.load(manifest.path(data_dir)):
        if e.get("test") == test and usable(e, sensor):
            out.append(
                Entry(
                    data_dir / e["file"], sensor_config(e["config"], sensor), int(e["session"]), int(e.get("power_up", 0)),
                    sensor in e.get("accel_tilted", {}) or sensor in e.get("accel_noisy", {}),
                )
            )
    return out


def chip_table(header: dict[str, str]) -> str:
    if header.get("chip") == "LSM6DSL":
        return (
            '{ model = "lsm6dsl", '
            f"odr_hz = {int(round(float(header['odr_hz'])))}, "
            f"accel_range_g = {int(float(header['accel_fs_g']))}, "
            f"gyro_range_dps = {int(float(header['gyro_fs_dps']))}, "
            f"gyro_bw = \"{LSM6DSL_GYRO_BW[int(header['gyro_bw'])]}\", "
            f"accel_bw = \"{LSM6DSL_ACCEL_BW[int(header['accel_bw'])]}\""
            " }"
        )
    return (
        "{ "
        f"odr_hz = {ODR_CHOICE[int(round(float(header['odr_hz'])))]}, "
        f"accel_range_g = {int(float(header['accel_fs_g']))}, "
        f"gyro_range_dps = {int(float(header['gyro_fs_dps']))}, "
        f"gyro_out_path = \"{'lpf1_lpf2' if int(header['gyro_out_path']) == fit.LPF1_LPF2_OUT else 'lpf1'}\", "
        f"gyro_bw = \"{GYRO_BW[int(header['gyro_bw']) & 3]}\", "
        f"accel_bw = \"{ACCEL_BW[int(header['accel_bw'])]}\""
        " }"
    )


def still_scenario(
    header: dict[str, str], up_in_sensor: np.ndarray, seconds: float, seed: int, params: str, odr_scale: float | None = None
) -> str:
    """Both boards at the same rest orientation: world up appears along up_in_sensor. odr_scale,
    when given, replaces the parameter file's (another unit's measured rate)."""
    up = up_in_sensor / np.linalg.norm(up_in_sensor)
    # The mount rotation R (sensor -> segment, segment = world here) must map up_in_sensor to +z.
    axis = np.cross(up, [0.0, 0.0, 1.0])
    angle = float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))
    if np.linalg.norm(axis) < 1e-9:
        axis = np.array([1.0, 0.0, 0.0])
    mount = f"{{ axis = [{axis[0]:.9f}, {axis[1]:.9f}, {axis[2]:.9f}], deg = {angle:.9f} }}"
    chip = chip_table(header)
    rate = f"\nodr_scale = {odr_scale:.9f}" if odr_scale is not None else ""
    return sim.point_scenario("v1-still", seed, mount) + f"""[palm]
chip = {chip}
params = "{params}"{rate}
[finger]
chip = {chip}
params = "{params}"{rate}
[[moves]]
hold_s = {seconds:.3f}
"""


def metric_rows(m: bench.StillMetrics) -> dict[str, float]:
    rows: dict[str, float] = {}
    for kind, axes in (("accel", m.accel), ("gyro", m.gyro)):
        for name, a in zip("xyz", axes):
            rows[f"{kind}.{name}.bias"] = a.bias
            rows[f"{kind}.{name}.std"] = a.std
            rows[f"{kind}.{name}.noise_floor"] = 10 * np.log10(a.noise_floor)
            rows[f"{kind}.{name}.low_freq"] = 10 * np.log10(a.low_freq)
            rows[f"{kind}.{name}.cutoff_hz"] = a.cutoff_hz
            for tau, dev in zip(a.allan_tau_s, a.allan_dev):
                rows[f"{kind}.{name}.allan@{tau:.3g}"] = dev
    return rows


def tolerance(metric: str) -> float:
    kind = metric.split(".", 2)[2]
    return TOLERANCE["allan"] if kind.startswith("allan") else TOLERANCE[kind]


@dataclasses.dataclass(frozen=True)
class Score:
    metric: str
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool


def score(real: list[dict[str, float]], simulated: list[dict[str, float]]) -> list[Score]:
    """The acceptance rule; spectral levels are compared in dB, so their window is 10 log10(1 + tol)."""
    out = []
    for metric in real[0]:
        r = np.array([x[metric] for x in real if metric in x])
        s = np.array([x[metric] for x in simulated if metric in x])
        if len(s) == 0:
            continue
        tol = tolerance(metric)
        v = acceptance.judge(r, s, absolute=10 * np.log10(1 + tol)) if metric.endswith(DB_METRICS) else acceptance.judge(r, s, relative=tol)
        out.append(Score(metric, v.real_mean, v.real_sd, v.sim_mean, v.window, v.passed))
    return out


def run(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    *,
    sensor: str = SENSOR,
    sims_per_capture: int = 1,
    jobs: int = 4,
    log: typing.Callable[[str], None] = print,
) -> dict[str, list[Score]]:
    es = entries(data_dir, sensor=sensor)
    # "long" captures are the default config held for hours: they set the drift terms only.
    long_runs = [e for e in es if e.config == "long"]
    fit_set = [e for e in es if e.session == 1 and e.config != "long"]
    reported = [e for e in es if e.session > 1 and e.config != "long"]
    if not fit_set:
        raise SystemExit("no V1 session-1 captures in the manifest")

    work_dir.mkdir(parents=True, exist_ok=True)
    default = [e.path for e in fit_set if e.config == "default"] or [fit_set[0].path]
    fits = fit.fit_files(default, sensor)
    # LPF1 and the accel stages come from every capture whose gyro path is LPF1 only; other full
    # scales also get their own noise density from them.
    others = [e for e in fit_set if e.path not in default]
    voters = fits + fit.fit_files([e.path for e in others if not e.config.startswith("lpf2")], sensor)
    # Behind LPF2, LPF1 keeps that fit, and each BW_G setting's shape comes from the configs that use it.
    lpf2 = [e.path for e in others if e.config.startswith("lpf2")]
    lpf2_fits = fit.fit_files(lpf2, sensor, allan=False, gyro_lpf1=fit.lpf1_by_odr(voters))
    lpf2_by_bw: dict[int, fit.Shape] = {}
    for path, f in zip(lpf2, lpf2_fits):
        cap = capture.read_capture(path)
        bw = int(cap.stream(sensor).header["gyro_bw"]) & 3
        lpf2_by_bw[bw] = fit.vote(a.shape.odr_stage for a in f.gyro)
    ranged = voters[len(fits):] + lpf2_fits
    long_fits = fit.fit_files([e.path for e in long_runs], sensor)
    # Each long hold's zero level stands for its power-up.
    power_ups = sorted({e.power_up for e in long_runs if e.power_up})
    params_text = fit.params_toml(
        fits,
        source=", ".join(e.path.name for e in fit_set),
        gyro_lpf2_by_bw=lpf2_by_bw,
        by_range=ranged,
        shape_fits=voters,
        allan_fits=long_fits or None,
        accel_allan_fits=[f for e, f in zip(long_runs, long_fits) if not e.accel_excluded] or None,
        power_ups=[[f for e, f in zip(long_runs, long_fits) if e.power_up == pu] for pu in power_ups],
    )
    params_path = work_dir / f"{CHIP[sensor]}.toml"
    params_path.write_text(params_text)
    log(f"params from {len(default)} default and {len(lpf2)} LPF2 session-1 captures -> {params_path}")

    return score_configs(reported, params_path, work_dir, sensor=sensor, sims_per_capture=sims_per_capture, jobs=jobs, log=log)


def score_configs(
    reported: list[Entry],
    params_path: pathlib.Path,
    work_dir: pathlib.Path,
    *,
    sensor: str,
    sims_per_capture: int,
    jobs: int,
    log: typing.Callable[[str], None],
    own_odr: bool = False,
) -> dict[str, list[Score]]:
    """Simulates every reported capture from params_path and scores each config. With own_odr
    each simulation runs at its capture's measured ODR instead of the parameter file's."""

    def simulate(scen: pathlib.Path, out: pathlib.Path) -> dict[str, float]:
        sim.run(scen, out, truth=False)
        scap = capture.read_capture(out)
        ssid = scap.stream(sim.slot(sensor)).id
        return metric_rows(bench.still_metrics(scap, ssid, trim_s=60.0))

    results: dict[str, list[Score]] = {}
    with concurrent.futures.ThreadPoolExecutor(jobs) as pool:
        pending: dict[str, tuple[list[dict[str, float]], list[concurrent.futures.Future[dict[str, float]]]]] = {}
        for config in sorted({e.config for e in reported}):
            real_rows, sims = pending.setdefault(config, ([], []))
            for k, e in enumerate(x for x in reported if x.config == config):
                cap = capture.read_capture(e.path)
                sid = cap.stream(sensor).id
                real = bench.still_metrics(cap, sid, trim_s=60.0)
                real_rows.append(metric_rows(real))
                up = np.array([a.bias for a in real.accel])
                for j in range(sims_per_capture):
                    scen = work_dir / f"{config}-{k}-{j}.toml"
                    stream = cap.sensors[sid]
                    scale = stream.odr_hz / float(stream.header["odr_hz"]) if own_odr else None
                    scen.write_text(
                        still_scenario(stream.header, up, real.duration_s + 60.0, 1000 * k + j + 1, str(params_path.resolve()), scale)
                    )
                    sims.append(pool.submit(simulate, scen, work_dir / f"{config}-{k}-{j}.bin"))
        for config, (real_rows, sims) in pending.items():
            results[config] = score(real_rows, [f.result() for f in sims])
            passed = sum(s.passed for s in results[config])
            log(f"{config}: {passed}/{len(results[config])} metrics pass ({len(real_rows)} reported captures)")
    return results


def run_transfer(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    params_path: pathlib.Path,
    *,
    sensor: str,
    test: str = "V1rig",
    sims_per_capture: int = 1,
    jobs: int = 4,
    log: typing.Callable[[str], None] = print,
) -> dict[str, list[Score]]:
    """Another unit's captures scored against parameters fitted elsewhere, nothing refitted: every
    session of the test is reported, each simulated at its own measured ODR."""
    reported = [e for e in entries(data_dir, test=test, sensor=sensor) if e.config != "long"]
    if not reported:
        raise SystemExit(f"no {test} captures for {sensor} in the manifest")
    work_dir.mkdir(parents=True, exist_ok=True)
    log(f"{sensor} {test}: {len(reported)} captures against {params_path}")
    return score_configs(
        reported, params_path, work_dir, sensor=sensor, sims_per_capture=sims_per_capture, jobs=jobs, log=log, own_odr=True
    )

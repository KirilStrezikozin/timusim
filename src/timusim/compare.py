"""Other simulators on the V1 and V2 still holds. Each runs in its own environment under compare/
and gets what its sensor model can express, from the session-1 captures: the measured ODR, the
gyro zero level, the per-axis standard deviation and the LSB; a 3x3 accel gain where the model
has one, an accel zero level alone where it does not; one Gauss-Markov drift term, or a 1/f bias
instability with rate random walk, where the model has those. Every reported capture is then simulated at its orientation and scored with the
acceptance rule of its test. A run whose job is unchanged keeps its earlier output."""

import concurrent.futures
import json
import os
import pathlib
import shutil
import subprocess
import typing

import numpy as np
import scipy.optimize

from timusim import bench, capture, fit, manifest, v1, v2

SIMULATORS = ("imusim", "wimusim", "gnss-ins-sim", "matlab")
# Models with an accel gain matrix; the others take the zero level alone.
GAIN = frozenset({"imusim", "matlab"})
# Models with a first-order Gauss-Markov drift term.
MARKOV = frozenset({"gnss-ins-sim"})
# Models with a 1/f bias instability and rate random walk.
FLICKER = frozenset({"matlab"})
MARKOV_TAUS_S = tuple(float(t) for t in np.logspace(-0.5, 3.5, 41))
MARKOV_REFITS = 3
COMPARE_DIR = pathlib.Path(__file__).resolve().parents[2] / "compare"
TRIM_S = 60.0


def params(fit_set: list[bench.StillMetrics], header: dict[str, str]) -> dict[str, typing.Any]:
    """Accel bias stays at zero: the orientation comes from each reported capture's mean."""
    mean = lambda kind, q: np.mean([[getattr(a, q) for a in getattr(m, kind)] for m in fit_set], axis=0).tolist()
    return {
        "odr_hz": float(np.mean([m.odr_hz for m in fit_set])),
        "accel": {"bias": [0.0, 0.0, 0.0], "sd": mean("accel", "std"), "lsb": float(header["accel_lsb_g"])},
        "gyro": {"bias": mean("gyro", "bias"), "sd": mean("gyro", "std"), "lsb": float(header["gyro_lsb_dps"])},
    }


def markov_term(curves: list[tuple[np.ndarray, np.ndarray]]) -> tuple[float, float]:
    """One Gauss-Markov term over white noise, fitted to the Allan curves of one axis where the
    filters no longer shape them: both variances by non-negative least squares on the error
    relative to the model, refitted from the measured values, and the correlation time on a grid."""
    tau = np.concatenate([t[t >= fit.DRIFT_MIN_TAU_S] for t, _ in curves])
    avar = np.concatenate([a[t >= fit.DRIFT_MIN_TAU_S] ** 2 for t, a in curves])
    best = (np.inf, 0.0, MARKOV_TAUS_S[0])
    for t_c in (t for t in MARKOV_TAUS_S if t <= tau.max()):
        design = np.column_stack([fit.white_avar(1.0, tau), fit.gauss_markov_avar(tau, 1.0, t_c)])
        scale = avar
        for _ in range(MARKOV_REFITS + 1):
            x, _ = scipy.optimize.nnls(design / scale[:, None], avar / scale)
            scale = design @ x
        err = float(np.mean((avar / scale - 1) ** 2))
        if err < best[0]:
            best = (err, float(np.sqrt(x[1])), t_c)
    return best[1], best[2]


def flicker_term(curves: list[tuple[np.ndarray, np.ndarray]]) -> tuple[float, float]:
    """The Allan floor of a 1/f term and the rate random walk over white noise, fitted to the Allan
    curves of one axis as markov_term does: avar = d^2/(2 tau) + floor^2 + k^2 tau/3."""
    tau = np.concatenate([t[t >= fit.DRIFT_MIN_TAU_S] for t, _ in curves])
    avar = np.concatenate([a[t >= fit.DRIFT_MIN_TAU_S] ** 2 for t, a in curves])
    design = np.column_stack([fit.white_avar(1.0, tau), np.ones(len(tau)), tau / 3])
    scale = avar
    for _ in range(MARKOV_REFITS + 1):
        x, _ = scipy.optimize.nnls(design / scale[:, None], avar / scale)
        scale = design @ x
    return float(np.sqrt(x[1])), float(np.sqrt(x[2]))


def drift_params(
    caps: list[capture.Capture], sensor: str, term: typing.Callable, names: tuple[str, str]
) -> dict[str, dict[str, list[float]]]:
    """A two-number drift term per axis, accel and gyro, from the Allan curves of the captures."""
    out = {}
    for kind in ("accel", "gyro"):
        signals = []
        for cap in caps:
            s = cap.stream(sensor)
            keep = s.longest_run(s.t_s >= TRIM_S)
            signals.append(((s.accel_g if kind == "accel" else s.gyro_dps)[keep], s.odr_hz))
        terms = [term([fit.allan_curve(x[:, i], fs) for x, fs in signals]) for i in range(3)]
        out[kind] = {names[0]: [t[0] for t in terms], names[1]: [t[1] for t in terms]}
    return out


def available(simulators: typing.Sequence[str] = SIMULATORS) -> list[str]:
    """The simulators whose environment is set up; MATLAB also needs distrobox and its install."""
    ok = [s for s in simulators if (COMPARE_DIR / s / ".venv").exists()]
    matlab_root = pathlib.Path(os.environ.get("TIMUSIM_MATLAB_ROOT", COMPARE_DIR.parent / "external" / "matlab" / "R2026b"))
    if "matlab" in ok and (shutil.which("distrobox") is None or not (matlab_root / "bin" / "matlab").exists()):
        ok.remove("matlab")
    return ok


def run_driver(simulator: str, job: pathlib.Path, out: pathlib.Path) -> None:
    project = COMPARE_DIR / simulator
    env = {k: v for k, v in os.environ.items() if k != "VIRTUAL_ENV"}
    cmd = ["uv", "run", "--no-sync", "--project", str(project), "python", str(project / "run.py"), str(job), str(out)]
    result = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if result.returncode != 0:
        raise RuntimeError(f"{simulator} failed on {job.name}:\n{result.stderr}")


def submit(
    pool: concurrent.futures.Executor, simulator: str, job: dict[str, typing.Any], out: pathlib.Path
) -> concurrent.futures.Future | None:
    """Runs the job unless out already holds this job's output."""
    out.parent.mkdir(parents=True, exist_ok=True)
    job_path = out.with_suffix(".json")
    text = json.dumps(job, indent=2)
    if out.exists() and job_path.exists() and job_path.read_text() == text:
        return None
    job_path.write_text(text)
    return pool.submit(run_driver, simulator, job_path, out)


def load(out: pathlib.Path, trim_s: float = TRIM_S) -> tuple[np.ndarray, np.ndarray, float]:
    """A driver's accel and gyro after the trim, and the rate."""
    d = np.load(out)
    fs = float(d["odr_hz"])
    keep = slice(int(round(trim_s * fs)), None)
    return d["accel_g"][keep], d["gyro_dps"][keep], fs


def simulated_metrics(out: pathlib.Path) -> bench.StillMetrics:
    return bench.array_metrics(*load(out))


def run(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    simulators: typing.Sequence[str] = SIMULATORS,
    *,
    sensor: str = v1.SENSOR,
    jobs: int = 4,
    log: typing.Callable[[str], None] = print,
) -> dict[str, dict[str, list[v1.Score]]]:
    es = v1.entries(data_dir, sensor=sensor)
    reported = [e for e in es if e.session > 1 and e.config != "long"]
    results: dict[str, dict[str, list[v1.Score]]] = {s: {} for s in simulators}
    with concurrent.futures.ThreadPoolExecutor(jobs) as pool:
        for config in sorted({e.config for e in reported}):
            fit_caps = [capture.read_capture(e.path) for e in es if e.session == 1 and e.config == config]
            if not fit_caps:
                log(f"{config}: no session-1 capture, skipped")
                continue
            sid = lambda cap: cap.stream(sensor).id
            base = params([bench.still_metrics(c, sid(c), trim_s=TRIM_S) for c in fit_caps], fit_caps[0].sensors[sid(fit_caps[0])].header)
            drift = {}
            if MARKOV & set(simulators):
                drift["markov"] = drift_params(fit_caps, sensor, markov_term, ("sigma", "tau_s"))
            if FLICKER & set(simulators):
                drift["flicker"] = drift_params(fit_caps, sensor, flicker_term, ("allan_floor", "rate_random_walk"))
            del fit_caps
            real_rows, pending = [], []
            for k, e in enumerate(x for x in reported if x.config == config):
                cap = capture.read_capture(e.path)
                real = bench.still_metrics(cap, sid(cap), trim_s=TRIM_S)
                real_rows.append(v1.metric_rows(real))
                job = {**base, "seconds": real.duration_s + TRIM_S, "seed": 1000 * k + 1, "up": [a.bias for a in real.accel]}
                for simulator in simulators:
                    own = job
                    for key, members in (("markov", MARKOV), ("flicker", FLICKER)):
                        if simulator in members:
                            own = {**own, **{kind: {**own[kind], key: drift[key][kind]} for kind in ("accel", "gyro")}}
                    out = work_dir / simulator / f"{config}-{k}.npz"
                    pending.append((simulator, out, submit(pool, simulator, own, out)))
            for simulator in simulators:
                sim_rows = []
                for name, out, done in pending:
                    if name == simulator:
                        if done is not None:
                            done.result()
                        sim_rows.append(v1.metric_rows(simulated_metrics(out)))
                results[simulator][config] = v1.score(real_rows, sim_rows)
                passed = sum(s.passed for s in results[simulator][config])
                log(f"{config} {simulator}: {passed}/{len(results[simulator][config])} metrics pass ({len(real_rows)} reported captures)")
    return results


def run_v2(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    simulators: typing.Sequence[str] = SIMULATORS,
    *,
    sensor: str = v1.SENSOR,
    sims_per_hold: int = 3,
    jobs: int = 4,
    log: typing.Callable[[str], None] = print,
) -> dict[str, list[v2.Score]]:
    """V2 holds: noise and rate from the session-1 holds, the accel zero level and gain fitted on
    them as v2.run does, or the zero level alone (mean minus up) for models without a gain. Each
    reported hold is simulated at its declared orientation."""
    holds = v2.entries(data_dir, sensor)
    fit_set = [h for h in holds if h.session == 1]
    reported = [h for h in holds if h.session > 1]
    if not fit_set:
        raise SystemExit("no V2 session-1 holds in the manifest")
    means, metrics = [], []
    for h in fit_set:
        cap = capture.read_capture(h.path)
        means.append(v2.hold_metrics(cap, sensor, h.up).mean_g)
        metrics.append(bench.still_metrics(cap, cap.stream(sensor).id, trim_s=v2.SETTLE_S))
    header = cap.stream(sensor).header
    ups = np.array([h.up for h in fit_set])
    offset, gain = v2.fit_accel(np.array(means), ups)
    zero = np.mean(np.array(means) - ups, axis=0)
    base = params(metrics, header)
    log(f"accel zero level and gain from {len(fit_set)} session-1 holds; zero level alone {np.round(zero, 4).tolist()}")

    pending: list[tuple[str, int, list[tuple[pathlib.Path, concurrent.futures.Future | None]]]] = []
    with concurrent.futures.ThreadPoolExecutor(jobs) as pool:
        for k, h in enumerate(reported):
            cap = capture.read_capture(h.path)
            seconds = cap.stream(sensor).duration_s
            for simulator in simulators:
                accel = {**base["accel"], "bias": (offset if simulator in GAIN else zero).tolist()}
                if simulator in GAIN:
                    accel["gain"] = gain.tolist()
                outs = []
                for j in range(sims_per_hold):
                    job = {**base, "accel": accel, "seconds": seconds, "seed": 1000 * k + j + 1, "up": list(h.up)}
                    out = work_dir / simulator / f"hold-{k}-{j}.npz"
                    outs.append((out, submit(pool, simulator, job, out)))
                pending.append((simulator, k, outs))
        results = {}
        for simulator in simulators:
            rows: v2.Rows = {}
            for name, k, outs in pending:
                if name != simulator:
                    continue
                h = reported[k]
                simulated = []
                for out, done in outs:
                    if done is not None:
                        done.result()
                    simulated.append(v2.array_hold_metrics(load(out, v2.SETTLE_S)[0], h.up))
                real = v2.hold_metrics(capture.read_capture(h.path), sensor, h.up)
                v2.add_row(rows, h.position, real, simulated)
            log(f"{simulator}:")
            results[simulator] = v2.score_rows(rows, lambda line: log(f"  {line}"))
            (work_dir / simulator / "scores.csv").write_text(v2.scores_csv(results[simulator]))
    return results


def table(
    results: dict[str, dict[str, list[v1.Score]]], own: dict[tuple[str, str], tuple[float, bool]] | None = None
) -> list[str]:
    """One row per config and metric: the real mean and window, then each simulator's mean and pass,
    timusim's own from its V1 scores when given."""
    sims = list(results)
    names = (["timusim"] if own else []) + sims
    rows = ["config,metric,real_mean,real_sd,window," + ",".join(f"{n}_mean,{n}_pass" for n in names)]
    first = results[sims[0]]
    for config, scores in first.items():
        for i, s in enumerate(scores):
            cells = [config, s.metric, f"{s.real_mean:.6g}", f"{s.real_sd:.6g}", f"{s.window:.6g}"]
            if own:
                mean, passed = own.get((config, s.metric), (float("nan"), False))
                cells += [f"{mean:.6g}", str(int(passed))]
            for n in sims:
                other = results[n][config][i]
                cells += [f"{other.sim_mean:.6g}", str(int(other.passed))]
            rows.append(",".join(cells))
    return rows


def table_v2(results: dict[str, list[v2.Score]]) -> list[str]:
    """One row per position and metric, as table does for V1; timusim is one of the results."""
    names = list(results)
    rows = ["position,metric,real_mean,real_sd,window," + ",".join(f"{n}_mean,{n}_pass" for n in names)]
    for i, s in enumerate(results[names[0]]):
        cells = [s.position, s.metric, f"{s.real_mean:.6g}", f"{s.real_sd:.6g}", f"{s.window:.6g}"]
        for n in names:
            cells += [f"{results[n][i].sim_mean:.6g}", str(int(results[n][i].passed))]
        rows.append(",".join(cells))
    return rows


def read_v1_scores(path: pathlib.Path) -> dict[tuple[str, str], tuple[float, bool]]:
    out = {}
    for line in path.read_text().splitlines()[1:]:
        config, metric, _, _, sim_mean, _, passed = line.split(",")
        out[(config, metric)] = (float(sim_mean), passed == "1")
    return out


def default_work_dir(data_dir: pathlib.Path = manifest.DATA_DIR, sensor: str = v1.SENSOR, test: str = "V1") -> pathlib.Path:
    name = "compare" if test == "V1" else f"compare-{test.lower()}"
    return data_dir / (name if sensor == v1.SENSOR else f"{name}-{sensor}")

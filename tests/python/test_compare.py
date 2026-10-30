import json
import pathlib

import numpy as np
import pytest
import scipy.signal

from timusim import compare, fit

JOB = {
    "seconds": 90.0, "odr_hz": 476.0, "seed": 1, "up": [0.1, -0.2, 0.97],
    "accel": {"bias": [0.01, -0.02, 0.03], "sd": [0.002, 0.0015, 0.003], "lsb": 0.000122},
    "gyro": {"bias": [1.4, 1.1, -0.4], "sd": [0.2, 0.15, 0.1], "lsb": 0.07},
}


@pytest.mark.parametrize("up", [JOB["up"], [0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
@pytest.mark.parametrize("simulator", compare.SIMULATORS)
def test_driver_holds_still_with_the_job_parameters(simulator: str, up: list[float], tmp_path: pathlib.Path) -> None:
    """Axis-aligned ups as V2 declares them, one of them opposite each simulator's world up."""
    if simulator not in compare.available():
        pytest.skip(f"{simulator} is not set up (README, Compare)")
    job = tmp_path / "job.json"
    job.write_text(json.dumps({**JOB, "up": up}))
    compare.run_driver(simulator, job, tmp_path / "out.npz")
    m = compare.simulated_metrics(tmp_path / "out.npz")
    up = np.array(up) / np.linalg.norm(up)
    assert m.duration_s == pytest.approx(JOB["seconds"] - compare.TRIM_S, abs=0.01)
    assert [a.bias for a in m.accel] == pytest.approx(up + JOB["accel"]["bias"], abs=2e-4)
    assert [a.std for a in m.accel] == pytest.approx(JOB["accel"]["sd"], rel=0.05)
    assert [a.bias for a in m.gyro] == pytest.approx(JOB["gyro"]["bias"], abs=0.01)
    assert [a.std for a in m.gyro] == pytest.approx(JOB["gyro"]["sd"], rel=0.05)


def test_markov_term_recovers_white_plus_gauss_markov() -> None:
    fs, n, sigma, t_c, white = 100.0, 200_000, 0.01, 20.0, 0.05
    rng = np.random.default_rng(3)
    a = np.exp(-1 / (fs * t_c))
    drift = scipy.signal.lfilter([sigma * np.sqrt(1 - a**2)], [1, -a], rng.standard_normal(n))
    x = drift + white * rng.standard_normal(n)
    got_sigma, got_tau = compare.markov_term([fit.allan_curve(x, fs)])
    assert got_sigma == pytest.approx(sigma, rel=0.3)
    assert got_tau == pytest.approx(t_c, rel=0.5)


def test_flicker_term_recovers_floor_and_random_walk() -> None:
    tau = np.logspace(-1, 2.5, 40)
    white, floor, rrw = 0.05, 0.004, 0.0003
    dev = np.sqrt(fit.white_avar(white, tau) + floor**2 + rrw**2 * tau / 3)
    got_floor, got_rrw = compare.flicker_term([(tau, dev)])
    assert got_floor == pytest.approx(floor, rel=1e-3)
    assert got_rrw == pytest.approx(rrw, rel=1e-3)

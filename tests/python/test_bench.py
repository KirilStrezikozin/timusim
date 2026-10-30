import numpy as np
import scipy.signal

from timusim import bench

FS = 476.0


def test_white_noise_floor_and_allan() -> None:
    rng = np.random.default_rng(0)
    sigma = 0.1
    x = 1.5 + sigma * rng.standard_normal(int(600 * FS))
    m = bench.axis_metrics(x, FS)
    assert abs(m.bias - 1.5) < 1e-3
    assert abs(m.std - sigma) / sigma < 0.01
    assert abs(m.noise_floor - 2 * sigma**2 / FS) / (2 * sigma**2 / FS) < 0.05
    assert abs(m.low_freq - 2 * sigma**2 / FS) / (2 * sigma**2 / FS) < 0.25
    assert m.cutoff_hz > 0.45 * FS
    for tau, dev in zip(m.allan_tau_s, m.allan_dev):
        if tau > 10.0:
            continue
        expected = sigma / np.sqrt(tau * FS)
        assert abs(dev - expected) / expected < 0.2


def test_lowpass_cutoff() -> None:
    rng = np.random.default_rng(1)
    sos = scipy.signal.butter(4, 50.0, fs=FS, output="sos")
    x = scipy.signal.sosfilt(sos, rng.standard_normal(int(600 * FS)))
    m = bench.axis_metrics(x, FS)
    assert abs(m.cutoff_hz - 50.0) < 2.0

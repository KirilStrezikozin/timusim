import pathlib

import numpy as np
import pytest

from timusim import capture, fit, sim

pytestmark = pytest.mark.skipif(
    not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)"
)

PARAMS = """
odr_scale = 0.97
accel_aa = { kind = "butterworth", order = 2 }
accel_lp = { kind = "one_pole", order = 1 }
gyro_lpf1 = { kind = "butterworth", order = 2 }
[accel]
white_density = [1.2e-4, 1.0e-4, 1.4e-4]
[gyro]
white_density = [0.017, 0.015, 0.014]
"""

SCENARIO = """
name = "still"
seed = 3
[geometry]
palm_sensor = [0.0, 0.0, 0.0]
mcp = [0.0, 0.0, 0.0]
proximal_m = 0.0
middle_m = 0.0
finger_sensor = [0.0, 0.0, 0.0]
dip_per_pip = 0.0
[palm]
params = "params.toml"
[finger]
params = "params.toml"
[[moves]]
hold_s = 300
"""


def test_fit_recovers_simulated_chip(tmp_path: pathlib.Path) -> None:
    (tmp_path / "params.toml").write_text(PARAMS)
    (tmp_path / "still.toml").write_text(SCENARIO)
    sim.run(tmp_path / "still.toml", tmp_path / "still.bin")
    cap = capture.read_capture(tmp_path / "still.bin")
    f = fit.fit_capture(cap, 1, trim_s=1.0, allan=False)

    assert abs(f.odr_scale - 0.97) < 1e-4
    for axis, density in zip(f.accel, (1.2e-4, 1.0e-4, 1.4e-4)):
        assert axis.shape.odr_stage == fit.Shape("one_pole", 1)
        assert abs(axis.shape.density / density - 1) < 0.05
    for axis, density in zip(f.gyro, (0.017, 0.015, 0.014)):
        assert (axis.shape.internal, axis.shape.odr_stage) == (fit.Shape("butterworth", 2), fit.Shape())
        assert abs(axis.shape.internal_scale - 1) < 0.03
        assert abs(axis.shape.density / density - 1) < 0.05


def test_fit_recovers_lpf1_off_the_table(tmp_path: pathlib.Path) -> None:
    (tmp_path / "params.toml").write_text(
        PARAMS + '[gyro_lpf1_by_odr]\n"476" = { kind = "one_pole", order = 3, cutoff_scale = 0.8 }\n'
    )
    (tmp_path / "still.toml").write_text(SCENARIO)
    sim.run(tmp_path / "still.toml", tmp_path / "still.bin")
    f = fit.fit_capture(capture.read_capture(tmp_path / "still.bin"), 1, trim_s=1.0, allan=False)

    for shape, scale in fit.lpf1_by_odr([f])[476]:
        assert shape == fit.Shape("one_pole", 3)
        assert abs(scale / 0.8 - 1) < 0.03
    for axis, density in zip(f.gyro, (0.017, 0.015, 0.014)):
        assert abs(axis.shape.density / density - 1) < 0.05


def test_fit_drift_recovers_simulated_terms(tmp_path: pathlib.Path) -> None:
    drift = """
markov = [
    { sigma = [0.02, 0.02, 0.02], tau_s = [1.0, 1.0, 1.0] },
    { sigma = [0.02, 0.02, 0.02], tau_s = [30.0, 30.0, 30.0] },
]
"""
    (tmp_path / "params.toml").write_text(PARAMS + drift)
    (tmp_path / "still.toml").write_text(SCENARIO.replace("hold_s = 300", "hold_s = 1200"))
    sim.run(tmp_path / "still.toml", tmp_path / "still.bin")
    f = fit.fit_capture(capture.read_capture(tmp_path / "still.bin"), 1, trim_s=1.0)

    truth = fit.Drift(((0.02, 1.0), (0.02, 30.0)), 0.0)
    tau = np.array([0.5, 2.0, 10.0, 30.0])
    freq = np.array([0.03, 0.1, 0.3, 1.0])
    psd_db = []
    for axis, density in zip(f.gyro, (0.017, 0.015, 0.014)):
        assert axis.allan is not None and axis.spectrum is not None
        got = fit.fit_drift([axis.allan], axis.white_low, [axis.spectrum])
        assert np.all(np.abs(np.sqrt(got.avar(tau) / truth.avar(tau)) - 1) < 0.2)
        psd_db.append(10 * np.log10((got.psd(freq) + density**2) / (truth.psd(freq) + density**2)))
    # 1200 s holds only a few periods at 0.03 Hz; a bias shows in the mean over axes above it.
    assert np.all(np.abs(psd_db) < 2.0)
    assert np.all(np.abs(np.mean(psd_db, axis=0)[1:]) < 0.3)


def test_fit_recovers_simulated_lsm6dsl(tmp_path: pathlib.Path) -> None:
    (tmp_path / "params.toml").write_text(
        """
odr_scale = 1.009
accel_aa = { kind = "butterworth", order = 2 }
accel_lp = { kind = "one_pole", order = 1 }
[gyro_lpf1_by_odr]
"833" = { kind = "one_pole", order = 4, cutoff_scale = 0.85 }
[accel]
white_density = [5.0e-5, 4.5e-5, 5.5e-5]
[gyro]
white_density = [0.0045, 0.004, 0.0035]
"""
    )
    chip = '\nchip = { model = "lsm6dsl", odr_hz = 833, gyro_bw = "lp1_light" }'
    (tmp_path / "still.toml").write_text(SCENARIO.replace("[palm]", "[palm]" + chip))
    sim.run(tmp_path / "still.toml", tmp_path / "still.bin")
    cap = capture.read_capture(tmp_path / "still.bin")
    assert cap.sensors[0].chip == "LSM6DSL"
    f = fit.fit_capture(cap, 0, trim_s=1.0, allan=False)

    assert abs(f.odr_scale - 1.009) < 1e-4
    for axis, density in zip(f.accel, (5.0e-5, 4.5e-5, 5.5e-5)):
        assert axis.shape.odr_stage == fit.Shape("one_pole", 1)
        assert abs(axis.shape.density / density - 1) < 0.05
    for shape, scale in fit.lpf1_by_odr([f])[833]:
        assert shape == fit.Shape("one_pole", 4)
        assert abs(scale / 0.85 - 1) < 0.03
    for axis, density in zip(f.gyro, (0.0045, 0.004, 0.0035)):
        assert abs(axis.shape.density / density - 1) < 0.05

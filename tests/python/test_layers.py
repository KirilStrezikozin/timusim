import pathlib

import numpy as np
import pytest

from timusim import bench, capture, sim

pytestmark = pytest.mark.skipif(
    not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)"
)

PARAMS = """
accel_lp = { kind = "one_pole", order = 1 }
gyro_lpf1 = { kind = "butterworth", order = 2 }
[gyro]
white_density = [0.015, 0.015, 0.015]
"""

# Lakie et al. 2012 hand-about-wrist values; the geometry is a test fixture.
SCENARIO = """
name = "layers"
seed = 5
[realism]
chip_transfer = {chip}
noise = {noise}
tremor = {tremor}
[geometry]
palm_sensor = [0.0, 0.05, 0.02]
mcp = [0.0, 0.09, 0.0]
proximal_m = 0.045
middle_m = 0.025
finger_sensor = [0.0, 0.01, 0.008]
dip_per_pip = 0.7
[palm]
params = "params.toml"
[finger]
params = "params.toml"
[tremor]
wrist_flex = {{ inertia = 0.0025, stiffness = 7.5, damping = 0.03, torque_density = 0.01, muscle_tau_s = 0.12 }}
[[moves]]
hold_s = 60
"""


def palm_gyro(tmp_path: pathlib.Path, *, chip: bool, noise: bool, tremor: bool) -> tuple[np.ndarray, float]:
    name = f"{int(chip)}{int(noise)}{int(tremor)}"
    (tmp_path / "params.toml").write_text(PARAMS)
    scen = tmp_path / f"{name}.toml"
    scen.write_text(SCENARIO.format(chip=str(chip).lower(), noise=str(noise).lower(), tremor=str(tremor).lower()))
    sim.run(scen, tmp_path / f"{name}.bin")
    s = capture.read_capture(tmp_path / f"{name}.bin").sensors[0]
    return s.gyro_dps[s.t_s > 2.0], s.odr_hz


def test_each_layer_toggles(tmp_path: pathlib.Path) -> None:
    quiet, _ = palm_gyro(tmp_path, chip=True, noise=False, tremor=False)
    assert np.all(quiet == 0.0)

    noisy, _ = palm_gyro(tmp_path, chip=True, noise=True, tremor=False)
    assert 0.1 < noisy[:, 0].std() < 0.3

    shaking, fs = palm_gyro(tmp_path, chip=True, noise=False, tremor=True)
    # Wrist flexion turns the palm about the flexion axis, -x, only.
    assert shaking[:, 0].std() > 0.1 and np.all(shaking[:, 1:] == 0.0)
    f, p = bench.psd(shaking[:, 0], fs)
    peak = f[np.argmax(p)]
    assert 7.5 < peak < 9.5
    band = (f > 3) & (f < 13)
    assert p[band].sum() / p[f > 0.5].sum() > 0.8

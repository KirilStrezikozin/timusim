import pathlib

import numpy as np
import pytest

from timusim import capture, rig, sim, v3

LSM9DS1_HEADER = {
    "chip": "LSM9DS1", "odr_hz": "476", "accel_fs_g": "2", "gyro_fs_dps": "245",
    "gyro_out_path": "0", "gyro_bw": "3", "accel_bw": "18",
}


def test_rotate_and_signed_angle_agree() -> None:
    axis = np.array([0.1, -0.2, 1.0])
    v = np.array([1.0, 0.5, 0.2])
    for deg in (-176.0, -30.0, 45.0, 171.0):
        assert abs(rig.signed_angle(axis, v, rig.rotate(axis, deg, v)) - deg) < 1e-9


def test_fit_rotation_recovers_a_mount() -> None:
    rng = np.random.default_rng(0)
    r = np.linalg.qr(rng.standard_normal((3, 3)))[0]
    r *= np.sign(np.linalg.det(r))
    b = rng.standard_normal((200, 3))
    assert np.allclose(rig.fit_rotation(b @ r.T, b), r, atol=1e-9)


def test_plateau_is_the_code_the_output_sits_on() -> None:
    assert v3.plateau(np.array([32700, 32736, 32736, 32746]), 1) == 32736
    assert v3.plateau(np.array([-32763, -32763, -32700]), -1) == -32763
    assert v3.plateau(np.array([32500, 32760, 32700]), 1) == 32760


@pytest.mark.skipif(not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)")
def test_replay_input_clips_at_full_scale_and_runs_at_the_given_rate(tmp_path: pathlib.Path) -> None:
    t = np.arange(0.0, 2.0, 1e-3)
    rate = np.zeros((len(t), 3))
    rate[:, 2] = 400.0 * np.sin(np.pi * t / 2.0)
    force = np.tile([0.0, 0.0, 1.0], (len(t), 1))
    rig.write_input(tmp_path / "in.csv", t, force, rate)
    scen = tmp_path / "replay.toml"
    slot = rig.Slot(LSM9DS1_HEADER, 470.0, "in.csv")
    scen.write_text(rig.replay_scenario(slot, slot, 2.0, 1))
    sim.run(scen, tmp_path / "out.bin", truth=False)
    s = rig.stream(capture.read_capture(tmp_path / "out.bin"), "finger")

    assert abs(s.odr_hz - 470.0) < 0.05
    codes = s.gyro_codes[:, 2].astype(np.int32)
    assert codes.max() >= v3.CLIP_CODE
    fs = 32768 * float(s.header["gyro_lsb_dps"])
    inside = np.abs(np.interp(s.t_s, t, rate[:, 2])) < 0.9 * fs
    assert np.max(np.abs(s.gyro_dps[inside, 2] - np.interp(s.t_s[inside], t, rate[:, 2]))) < 2.0

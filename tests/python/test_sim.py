import pathlib

import numpy as np
import pytest

from timusim import capture, sim

SCENARIO = sim.ROOT / "sim" / "scenarios" / "example.toml"

pytestmark = pytest.mark.skipif(
    not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)"
)


def test_sim_capture_reads_like_a_real_one(tmp_path: pathlib.Path) -> None:
    out = tmp_path / "example.bin"
    truth = sim.read_truth(sim.run(SCENARIO, out))
    cap = capture.read_capture(out)

    assert cap.bad_frames == 0 and cap.restarts == 0
    assert cap.header["fw"] == "timusim-sim"
    for s in cap.sensors.values():
        t = truth[s.name]
        assert s.gaps.clean
        assert np.array_equal(s.index, t.index)
        assert abs(s.odr_hz - 476.0) < 0.01
        # The example uses the ideal chip, so readings are the truth quantized; the CSV keeps 6 decimals.
        lsb_g = float(s.header["accel_lsb_g"])
        lsb_dps = float(s.header["gyro_lsb_dps"])
        assert np.max(np.abs(s.accel_g - t.specific_force_g)) <= lsb_g / 2 + 1e-6
        assert np.max(np.abs(s.gyro_dps - t.rate_dps)) <= lsb_dps / 2 + 1e-4


def test_same_seed_is_reproducible(tmp_path: pathlib.Path) -> None:
    sim.run(SCENARIO, tmp_path / "a.bin", seed=1)
    sim.run(SCENARIO, tmp_path / "b.bin", seed=1)
    ra = (tmp_path / "a.bin").read_bytes()
    rb = (tmp_path / "b.bin").read_bytes()
    assert ra == rb

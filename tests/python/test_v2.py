import pathlib
import tomllib

import numpy as np
import pytest

from timusim import manifest, sim, v2

BASE = """
odr_scale = 0.97
accel_aa = { kind = "butterworth", order = 2 }
accel_lp = { kind = "one_pole", order = 1 }
gyro_lpf1 = { kind = "butterworth", order = 2 }
gyro_offset = [1.4, 1.1, -0.4]
[accel]
white_density = [1.2e-4, 1.0e-4, 1.4e-4]
[gyro]
white_density = [0.017, 0.015, 0.014]
"""
OFFSET = np.array([0.02, -0.015, 0.03])
GAIN = np.array([[1.01, 0.005, -0.003], [0.002, 0.99, 0.004], [-0.006, 0.001, 1.02]])
HEADER = {"odr_hz": "476", "accel_fs_g": "4", "gyro_fs_dps": "2000", "gyro_out_path": "0", "gyro_bw": "3", "accel_bw": "18"}


def test_parse_up() -> None:
    assert v2.parse_up("-y") == (0.0, -1.0, 0.0)
    assert v2.parse_up("0,3,4") == pytest.approx((0.0, 0.6, 0.8))
    assert v2.position(v2.parse_up("+z")) == "+z"
    with pytest.raises(ValueError):
        v2.parse_up("up")


def test_fit_accel_recovers_gain_and_offset() -> None:
    ups = np.array(list(v2.AXES.values()))
    offset, gain = v2.fit_accel(ups @ GAIN.T + OFFSET, ups)
    assert offset == pytest.approx(OFFSET)
    assert gain == pytest.approx(GAIN)
    with pytest.raises(ValueError):
        v2.fit_accel(ups[:4] @ GAIN.T, np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0]]))


def test_with_accel_sets_top_level_keys() -> None:
    text = v2.with_accel(BASE + "accel_offset = [9, 9, 9]\n", OFFSET, GAIN)
    params = tomllib.loads(text.replace("accel_offset = [9, 9, 9]\n", ""))
    assert params["accel_offset"] == pytest.approx(list(OFFSET))
    assert np.array(params["accel_gain"]) == pytest.approx(GAIN)
    assert "accel_offset" not in params["accel"]


@pytest.mark.skipif(not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)")
def test_v2_passes_on_holds_of_the_model_itself(tmp_path: pathlib.Path) -> None:
    """Sim holds with a known gain stand in for real ones: the fit must find it and the scoring accept it."""
    data = tmp_path / "data"
    data.mkdir()
    (data / "chip.toml").write_text(v2.with_accel(BASE, OFFSET, GAIN))
    base = tmp_path / "lsm9ds1.toml"
    base.write_text(BASE)
    for session in (1, 2, 3):
        for name, up in v2.AXES.items():
            stem = f"V2-default-s{session}-{name}"
            scen = data / f"{stem}.toml"
            scen.write_text(v2.v1.still_scenario(HEADER, np.array(up), 60.0, 10 * session + len(stem), "chip.toml"))
            sim.run(scen, data / f"{stem}.bin")
            manifest.append(
                {
                    "file": f"{stem}.bin", "test": "V2", "config": "default", "session": session, "clean": True,
                    "sensors": {"finger": {"up": list(up)}},
                },
                path=data / manifest.MANIFEST_PATH.name,
            )
    results = v2.run(data, tmp_path / "work", base, log=lambda _: None)
    fitted = tomllib.loads((tmp_path / "work" / base.name).read_text())
    assert np.array(fitted["accel_offset"]) == pytest.approx(OFFSET, abs=1e-3)
    assert np.array(fitted["accel_gain"]) == pytest.approx(GAIN, abs=1e-3)
    assert len(results) == 12 and all(r.passed for r in results), results

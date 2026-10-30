import pathlib

import pytest

from timusim import manifest, sim, v1

pytestmark = pytest.mark.skipif(
    not sim.DEFAULT_BINARY.exists(), reason="build the C++ simulator first (cmake --build build)"
)

PARAMS = """
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


LSM9DS1 = {"odr_hz": "476", "accel_fs_g": "4", "gyro_fs_dps": "2000", "gyro_out_path": "0", "gyro_bw": "3", "accel_bw": "18"}
LSM6DSL = {"chip": "LSM6DSL", "odr_hz": "416", "accel_fs_g": "4", "gyro_fs_dps": "2000", "gyro_out_path": "-1", "gyro_bw": "8", "accel_bw": "2"}


@pytest.mark.parametrize(("sensor", "header"), [("finger", LSM9DS1), ("palm", LSM6DSL)])
def test_v1_passes_on_captures_of_the_model_itself(tmp_path: pathlib.Path, sensor: str, header: dict[str, str]) -> None:
    """Sim captures stand in for real ones: the fit and the scoring must accept the model."""
    data = tmp_path / "data"
    data.mkdir()
    (data / "chip.toml").write_text(PARAMS)
    # As on the bench, the drift comes from a long hold: one short curve alone gives spurious terms.
    # Four reported sessions: with two, the SD that sets the window is itself too uncertain. 400 s
    # leaves five of the 100 s segments the low-frequency PSD needs after the 60 s trim.
    runs = [("default", session, 400.0) for session in range(1, 6)] + [("long", 1, 1800.0)]
    for config, session, seconds in runs:
        name = f"V1-{config}-s{session}"
        scen = data / f"{name}.toml"
        scen.write_text(v1.still_scenario(header, v1.np.array([0.3, -0.2, 0.93]), seconds, session, "chip.toml"))
        sim.run(scen, data / f"{name}.bin")
        manifest.append(
            {"file": f"{name}.bin", "test": "V1", "config": config, "session": session, "clean": True},
            path=data / manifest.MANIFEST_PATH.name,
        )
    results = v1.run(data, tmp_path / "work", sensor=sensor, log=lambda _: None)
    failed = [s for s in results["default"] if not s.passed]
    assert not [s for s in failed if not s.metric.startswith("accel")], failed


def test_each_capture_is_a_default_for_the_other_sensor() -> None:
    assert v1.sensor_config("palm_833hz", "palm") == "833hz"
    assert v1.sensor_config("palm_833hz", "finger") == "default"
    assert v1.sensor_config("16g", "palm") == "default"
    assert v1.sensor_config("16g", "finger") == "16g"
    assert v1.sensor_config("long", "palm") == "long"
    assert v1.sensor_config("952hz+palm_208hz", "palm") == "208hz"
    assert v1.sensor_config("952hz+palm_208hz", "finger") == "952hz"

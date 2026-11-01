import pathlib

import numpy as np

from timusim import acceptance, fit, protocol, reproduce, v1

INI = """[env]
build_flags = -Wall

[env:rig]
build_flags =
	${env.build_flags}
	-DPALM_LSM9=1

[env:trio_952hz_208hz]
build_flags =
	${env.build_flags}
	-DCAPTURE_CONFIG=\\"952hz+palm_208hz+palm_lsm9_952hz\\"
	-DPALM_LSM9=1
"""


def test_env_config_reads_capture_config(tmp_path: pathlib.Path) -> None:
    ini = tmp_path / "platformio.ini"
    ini.write_text(INI)
    assert protocol.env_config("rig", ini) == "default"
    assert protocol.env_config("trio_952hz_208hz", ini) == "952hz+palm_208hz+palm_lsm9_952hz"


def test_rig_sweep_covers_every_setting_of_each_sensor() -> None:
    configs = [protocol.env_config(e) for e in protocol.RIG_SWEEP]
    for sensor, settings in (
        ("finger", {"default", "low_range", "238hz", "119hz", "lpf2_strong", "lpf2_light", "952hz", "mid_range", "16g", "xl_lp_off", "xl_div50"}),
        ("palm", {"default", "208hz", "833hz", "lp1_light", "lp1_aggressive", "xl_div50", "low_range"}),
    ):
        assert {v1.sensor_config(c, sensor) for c in configs} == settings
    assert [v1.sensor_config(c, "palm_lsm9") for c in configs] == [v1.sensor_config(c, "finger") for c in configs]


def test_dropped_flags_a_second_or_a_thousandth() -> None:
    entry = {"sensors": {
        "palm": {"missing_samples": 400, "odr_measured_hz": 420.0, "samples": 250000},
        "finger": {"missing_samples": 500, "odr_measured_hz": 461.0, "samples": 270000},
    }}
    assert protocol.dropped(entry) == ["finger"]


def test_reproduce_runs_v2_before_the_rig_tests() -> None:
    names = [s[0] for s in reproduce.steps(pathlib.Path("data"))]
    assert names.index("v1-finger") < names.index("v2-finger") < names.index("v2-palm_lsm9") < names.index("v4")


def test_judge_takes_the_wider_of_tolerance_and_two_sd() -> None:
    v = acceptance.judge([1.0, 1.2, 1.4], [1.5], absolute=0.1)
    assert abs(v.window - 0.4) < 1e-12 and v.passed
    v = acceptance.judge([10.0], [12.6], relative=0.25)
    assert v.window == 2.5 and not v.passed


def test_with_top_level_replaces_and_inserts_before_the_first_table() -> None:
    text = "odr_scale = 1\naccel_gain = [[2]]\n\n[accel]\nwhite_density = [1, 1, 1]\n"
    out = fit.with_top_level(text, [f"accel_gain = {fit.matrix(np.eye(1))}"])
    assert out == "odr_scale = 1\n\naccel_gain = [[1]]\n[accel]\nwhite_density = [1, 1, 1]\n"


def test_place_reports_the_turn_and_the_tilt() -> None:
    from timusim import place, rig

    hinge = np.array([0.0, 0.0, 1.0])
    ref = np.array([0.6, 0.0, 0.8])
    line = place.report(rig.rotate(hinge, -10.0, ref), {"A": ref}, hinge)
    assert "turn from A  +10.0" in line and "to hinge  36.9 (ref  36.9)" in line

import tomllib

import numpy as np
import pytest

from timusim import rig, v5

HAND = {"geometry": {"palm_sensor": [0.0, 0.05, 0.02], "mcp": [0.0, 0.09, 0.0], "proximal_m": 0.045, "middle_m": 0.025,
                     "finger_sensor": [0.0, 0.01, 0.008], "dip_per_pip": 0.7, "twist_at_90_deg": 5.0},
        "board": {"kg": 0.124, "offset": [0.0, -0.02, 0.01]}}
HEADER = {"chip": "LSM9DS1", "odr_hz": "476", "accel_fs_g": "4", "gyro_fs_dps": "2000", "gyro_out_path": "0", "gyro_bw": "0", "accel_bw": "0"}
CHIPS = v5.Chips({n: HEADER for n in v5.SENSORS}, {n: 470.0 for n in v5.SENSORS})


def moves_seconds(text: str) -> float:
    return sum(m.get("hold_s", m.get("duration_s", 0.0)) for m in tomllib.loads(text)["moves"])


def test_routine_fills_its_phase() -> None:
    last = v5.move_windows(v5.MOVES[-1][0])[-1]
    assert last[1] + v5.REST_S == v5.PHASES["routine"][1]
    assert [d for _, _, d in v5.move_windows("index_curl")] == [1.0, -1.0] * v5.REPS


def test_scenario_timeline_matches_the_capture() -> None:
    params = {n: v5.TREMOR_FILE for n in v5.SENSORS}
    text = v5.scenario(HAND, CHIPS, params, v5.timeline_moves(), seed=1, start_support=0.3,
                       tremor={"mcp_flex": {"torque_density": 1e-4}}, mounts=(np.eye(3), np.eye(3)))
    t = tomllib.loads(text)
    assert moves_seconds(text) == v5.CAPTURE_S
    assert t["start"]["support"] == 0.3
    assert t["tremor"]["mcp_flex"]["torque_density"] == 1e-4
    assert [(m["kg"], m["at"]) for m in t["masses"]] == [(0.124, "palm"), (v5.FINGER_KG, "finger")]
    assert t["masses"][0]["offset"] == [0.0, -0.02, 0.01]


def test_rungs_add_layers_in_order() -> None:
    realism, twist, still = v5.rung_settings("ideal")
    assert not any(realism.values()) and not twist and still == set(v5.WRIST_JOINTS) | set(v5.DRIFT_JOINTS)
    realism, twist, still = v5.rung_settings("finger_tremor")
    assert all(realism.values()) and not twist and still == set(v5.WRIST_JOINTS) | set(v5.DRIFT_JOINTS)
    realism, twist, still = v5.rung_settings("wrist_and_twist")
    assert all(realism.values()) and twist and still == set(v5.DRIFT_JOINTS)
    realism, twist, still = v5.rung_settings("drift")
    assert all(realism.values()) and twist and not still
    drives = [{"name": "central"}, {"name": "heartbeat"}, {"name": "breathing"}]
    assert v5.rung_drives("finger_tremor", drives) == []
    assert [d["name"] for d in v5.rung_drives("drift", drives)] == ["central"]
    assert [d["name"] for d in v5.rung_drives("body", drives)] == ["central", "heartbeat", "breathing"]
    rough = drives + [{"name": "roughness"}]
    assert [d["name"] for d in v5.rung_drives("body", rough)] == ["central", "heartbeat", "breathing"]
    assert [d["name"] for d in v5.rung_drives("roughness", rough)] == ["central", "heartbeat", "breathing", "roughness"]


def test_handedness_sign() -> None:
    odr, w = 476.0, 60.0
    t = np.arange(int(10 * odr)) / odr
    # A body turning at w about its z sees gravity, here in its x-z plane, turn by -w about z.
    up = rig.rotate(np.array([0.0, 0.0, 1.0]), -w * t, np.tile([0.8, 0.0, 0.6], (len(t), 1)))
    gyro = np.tile([0.0, 0.0, w], (len(t), 1))
    assert v5.handedness(up, gyro, odr) > 0.99
    assert v5.handedness(up, -gyro, odr) < -0.99


def test_band_power_and_dominant_frequency_of_a_sine() -> None:
    odr = 476.0
    t = np.arange(int(60 * odr)) / odr
    gyro = np.column_stack([2.0 * np.sin(2 * np.pi * 9.0 * t), np.zeros_like(t), np.zeros_like(t)])
    f, p = v5.summed_psd(gyro, odr)
    bands = v5.band_powers(f, p)
    assert abs(v5.dominant_hz(f, p) - 9.0) <= f[1] - f[0]
    assert np.argmax(bands) == 2
    assert abs(bands[2] - 2.0) < 0.1  # a sine of amplitude 2 has power 2


def test_axis_angle_round_trip() -> None:
    m = rig.fit_rotation(np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]]), np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]))
    aa = v5.axis_angle(m)
    # Rows of the identity rotated: the columns of m.
    back = rig.rotate(np.array(aa["axis"]), aa["deg"], np.eye(3))
    assert np.allclose(back, m.T, atol=1e-9)


def test_fit_tremor_recovers_scales_weights_central_and_support(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(v5, "STIFFNESS_SCALES", v5.STIFFNESS_SCALES[2:7])
    f = np.linspace(0.0, 238.0, 1905)

    def peak(f0: float) -> np.ndarray:
        return 1.0 / (1.0 + ((f - f0) / 1.5) ** 2)

    # Joint -> (resonance at scale 1, segment-frame axis it turns about); the palm sees the
    # finger joints weakly.
    base = {"wrist_flex": (8.0, 0), "wrist_dev": (13.0, 2), "sway_ry": (11.0, 1),
            "mcp_flex": (14.0, 0), "pip_flex": (20.0, 0), "mcp_abd": (14.0, 2), "sway_rx": (1.0, 0), "sway_rz": (1.0, 2)}
    joint = {}
    for j, (f0, axis) in base.items():
        for s in v5.scales(j):
            joint[(j, s)] = {}
            for n in v5.SENSORS:
                psd = np.zeros((3, len(f)))
                psd[axis] = peak(f0 * np.sqrt(s)) * (2.0 if n == "finger" or v5.family(j) != "finger" else 0.2)
                joint[(j, s)][n] = psd
    none = {n: np.full((3, len(f)), 1e-4) for n in v5.SENSORS}
    units = v5.UnitRuns({n: f for n in v5.SENSORS}, none, joint)
    wrist, finger, x = v5.STIFFNESS_SCALES[1], v5.STIFFNESS_SCALES[2], np.array([1.5, 0.5, 0.6, 0.4, 0.3, 0.2, 0.1])
    at = {"wrist": wrist, "finger": finger, "drift": v5.DRIFT_SCALES[1]}
    central = v5.band_shape(f, 11.5, 0.8)

    def targets(scale: float) -> v5.Targets:
        out = {}
        for n in v5.SENSORS:
            psd = none[n] + scale * sum(xi * joint[(j, at[v5.family(j)])][n] for xi, group in zip(x, v5.FIT_GROUPS) for j in group)
            psd = psd + scale * 4.0 * joint[("wrist_flex", wrist)][n] * central
            out[n] = (v5.axis_bands_db(f, psd), v5.dominant_hz(f, psd.sum(axis=0)))
        return out

    tremor, drives, support, _ = v5.fit_tremor(units, targets(1.0), targets(0.25), log=lambda _: None, central_frequency_hz=11.5)
    lit = v5.literature()
    assert np.isclose(tremor["wrist_flex"]["stiffness"], lit["wrist_flex"]["stiffness"] * wrist)
    assert np.isclose(tremor["mcp_flex"]["stiffness"], lit["mcp_flex"]["stiffness"] * finger)
    abd = tremor["mcp_abd"]["torque_density"] / (v5.REFERENCE_DENSITY_PER_STIFFNESS * tremor["mcp_abd"]["stiffness"])
    assert abs(abd**2 - 0.3) < 0.03
    dev = tremor["wrist_dev"]["torque_density"] / (v5.REFERENCE_DENSITY_PER_STIFFNESS * tremor["wrist_dev"]["stiffness"])
    assert abs(dev**2 - 0.5) < 0.05
    assert abs(support - 0.5) < 0.05
    assert abs(drives[0]["bandwidth_hz"] - 0.8) < 0.2
    assert drives[0]["gain"]["wrist_flex"] > 10 * max(drives[0]["gain"]["wrist_dev"], drives[0]["gain"]["sway_ry"])


def test_mass_loading_notes() -> None:
    assert v5.added_grams("mass-loading +100 g; strap tight") == 100.0
    assert v5.added_grams("mass-loading +52.5 g") == 52.5
    assert v5.added_grams("lift at 65.3 s") is None


def test_routine_gain_and_rates() -> None:
    text = "\n".join(v5.timeline_moves(None, 1.6))
    assert text.count("support = 1.6") == 1 and text.index("support = 1.6") > text.index("joints_deg")
    reps = v5.scripted_reps()[:1]
    t = np.linspace(reps[0].out_start_s - 1, reps[0].back_start_s + 2, 20001)
    rates = v5.scripted_rates(reps, t)
    out = (t >= reps[0].out_start_s) & (t < reps[0].out_start_s + reps[0].out_s)
    for j, deg in reps[0].joints.items():
        assert abs(np.trapezoid(rates[j][out], t[out]) - np.radians(deg)) < 1e-3
    assert abs(np.trapezoid(rates["mcp_flex"], t)) < 1e-3
    masks = v5.routine_masks(reps, t)
    assert not (masks["moves"] & masks["holds"]).any()

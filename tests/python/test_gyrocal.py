import numpy as np
from scipy.spatial.transform import Rotation

from timusim import gyrocal

ODR = 470.0
GAIN = np.array([[0.96, 0.012, -0.004], [0.008, 1.01, -0.015], [0.003, 0.010, 0.98]])
NEGATIVE = np.array([0.997, 0.972, 1.03])


def sensed(w: np.ndarray) -> np.ndarray:
    return np.where(w < 0, w * NEGATIVE, w) @ GAIN.T


def synthetic_turns(n: int, seed: int) -> list[gyrocal.Turn]:
    rng = np.random.default_rng(seed)
    out = []
    up = np.array([0.0, 0.0, 1.0])
    for _ in range(n):
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        angle = rng.uniform(60.0, 150.0) * rng.choice([-1.0, 1.0])
        k = int(rng.uniform(1.0, 2.5) * ODR)
        # A smooth bell of rate, so a turn has both small and large rates.
        profile = np.sin(np.linspace(0.0, np.pi, k)) ** 2
        w = np.outer(profile / profile.sum() * angle * ODR, axis)
        r = Rotation.from_rotvec(np.radians(angle) * axis)
        up1 = r.inv().apply(up)
        out.append(gyrocal.Turn(0.0, sensed(w), 1.0 / ODR, up, up1))
        up = up1
    return out


def test_true_rate_inverts_the_chip_model():
    w = np.array([[30.0, -40.0, 5.0], [-10.0, 20.0, -60.0]])
    assert np.allclose(gyrocal.true_rate(sensed(w), GAIN, NEGATIVE), w)


def test_unit_gain_misfit_is_zero_for_an_ideal_gyro():
    t = synthetic_turns(1, 0)[0]
    t.rate_dps = gyrocal.true_rate(t.rate_dps, GAIN, NEGATIVE)
    assert gyrocal.misfit_deg(t, np.eye(3), np.ones(3)) < 0.02


def test_fit_recovers_gain_and_negative_gain():
    gain, negative = gyrocal.fit_gain(synthetic_turns(40, 1))
    assert np.abs(gain - GAIN).max() < 2e-3
    assert np.abs(negative - NEGATIVE).max() < 2e-3
    check = synthetic_turns(20, 2)
    assert max(gyrocal.misfit_deg(t, gain, negative) for t in check) < 0.1


def test_params_text_replaces_gains():
    base = "gyro_gain = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]\nodr = 476\n[accel]\nx = 1\n"
    text = gyrocal.params_text(base, GAIN, NEGATIVE)
    assert text.count("gyro_gain =") == 1 and "gyro_negative_gain =" in text
    assert text.index("gyro_negative_gain") < text.index("[accel]")


def test_refit_accel_recovers_offset_and_keeps_the_frame():
    rng = np.random.default_rng(3)
    ups = rng.normal(size=(40, 3))
    ups /= np.linalg.norm(ups, axis=1)[:, None]
    true_gain = np.array([[1.01, 0.003, 0.0], [0.003, 0.995, -0.002], [0.0, -0.002, 1.004]])
    offset = np.array([0.004, -0.01, 0.02])
    readings = ups @ true_gain.T + offset
    v2 = {"accel_gain": np.eye(3).tolist(), "accel_offset": [0.0, 0.0, 0.0]}
    params = gyrocal.refit_accel(v2, readings)
    assert gyrocal.norm_sd_mg(params, readings) < 0.01
    assert np.allclose(params["accel_offset"], offset, atol=1e-4)
    corrected = np.linalg.solve(np.array(params["accel_gain"]), (readings - np.array(params["accel_offset"])).T).T
    assert np.abs(corrected - ups).max() < 1e-4

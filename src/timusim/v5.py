"""V5, hand statistics: band powers, dominant frequency and motion statistics of the palm LSM6DSL
(on the STM32 board) and the finger LSM9DS1 on the hand, real against simulated, over the table,
free-hold and routine phases of RESEARCH.md's V5 procedure. Session 1 fits the mounts, the tremor and the table support;
sessions 2 and later are scored, on every rung of the ladder."""

import dataclasses
import itertools
import pathlib
import re
import subprocess
import sys
import threading
import time
import tomllib
import typing

import numpy as np
import scipy.ndimage
import scipy.optimize
import scipy.spatial.transform
import scipy.special
import scipy.stats

from timusim import acceptance, bench, capture, manifest, rig, sim, v1

SENSORS = ("palm", "finger")
SLOT = {"palm": "palm", "finger": "finger"}
HAND_FILE = sim.ROOT / "sim" / "scenarios" / "v5" / "hand.toml"
TREMOR_FILE = sim.ROOT / "sim" / "params" / "tremor_literature.toml"
G = rig.G

# The capture's timeline, seconds from its first sample.
LIFT_S = 60.0
LIFT_DURATION_S = 2.0
HOLD_END_S = 122.0
CAPTURE_S = 280.0
# Mass loading: table, lift and hold only, with weight added on the board. Mechanical tremor
# slows with the load as sqrt(K / I); a central oscillation keeps its frequency.
MASS_CAPTURE_S = 125.0
MASS_TAG = "mass-loading"
MECHANICAL_SEARCH_HZ = (3.0, 9.5)
# Peaks for the mass check are taken on the spectrum smoothed over this width: single bins
# jump between trials when the hold's spectrum is flat.
PEAK_SMOOTH_HZ = 1.0
PHASES = {"table": (5.0, 58.0), "hold": (67.0, 120.0), "routine": (122.0, 272.0)}
# The lift may start 3 s early (the table window ends at 58 s) or 6 s late (the hand settles
# within about 3 s, by the hold window at 67 s).
LIFT_EARLY_S, LIFT_LATE_S = 3.0, 6.0
LIFT_DPS = 30.0

# Routine moves from the free-hold pose, joint targets in degrees. Right hand: positive MCP
# abduction and wrist deviation turn about the palm's z, towards the thumb.
MOVES = (
    ("index_curl", {"mcp_flex": 45.0, "pip_flex": 60.0}),
    ("wrist_extension", {"wrist_flex": -30.0}),
    ("wrist_flexion", {"wrist_flex": 30.0}),
    ("index_abduction", {"mcp_abd": 15.0}),
    ("radial_deviation", {"wrist_dev": 15.0}),
)
REPS = 5
OUT_S, HOLD_S, BACK_S, REST_S = 1.0, 2.0, 1.0, 2.0
REP_S = OUT_S + HOLD_S + BACK_S + REST_S
ZERO_JOINTS = {"wrist_flex": 0.0, "wrist_dev": 0.0, "mcp_abd": 0.0, "mcp_flex": 0.0, "pip_flex": 0.0}
# Each repetition is measured from the real capture and the sim replays it: the angle integrated
# from the segment-frame gyro (the palm's, or the finger's less the palm's) about the move's axis,
# signed as the joint (flexion about the palm's -x, deviation and abduction about +z), and its
# timing from the 50% crossings of that angle, the duration from the 10-90% rise (0.51 of a
# min-jerk move). The search window spans the repetition's slot widened for early or late moves.
MOVE_AXIS = {"index_curl": ("finger", 0, -1.0), "wrist_extension": ("palm", 0, -1.0), "wrist_flexion": ("palm", 0, -1.0),
             "index_abduction": ("finger", 2, 1.0), "radial_deviation": ("palm", 2, 1.0)}
MOVE_WINDOW_S = (-2.4, 4.2)
MIN_JERK_10_90 = 0.51
MIN_MOVE_S = 0.15
# Each move's curve is then fitted with the beta velocity profile, s^(a-1) (1-s)^(b-1) (min-jerk
# is a = b = 3): its start, duration and shape, the shape bounded so that the acceleration stays
# finite (a, b >= 2) and the peak plausible.
SHAPE_BOUNDS = (2.0, 8.0)
MIN_JERK_SHAPE = (3.0, 3.0)

BANDS = ((0.1, 3.0), (3.0, 8.0), (8.0, 13.0), (13.0, 25.0), (25.0, 100.0))
DOMINANT_HZ = (3.0, 25.0)
PEAK_WINDOW_S = 1.0

# Tolerances from RESEARCH.md: absolute for dB and Hz, relative otherwise. Handedness has none
# listed and is judged on 2 real SD alone until the author sets one.
ABSOLUTE = {"band_power_db": 3.0, "dominant_hz": 1.0}
RELATIVE = {"gyro_std_dps": 0.25, "median_peak_dps": 0.25, "linear_accel_g": 0.25, "accel_noise_g": 0.20}
# The central oscillation's level varies from day to day (session 1's free hold had an 11-13 Hz
# peak, session 2's none), and session 1 alone sets it: its band is scored with this tolerance
# in the hold and routine. Set after session 2 was seen (RESEARCH.md Changes); the ladder also
# reports the strict count.
CENTRAL_BAND_METRIC, CENTRAL_BAND_PHASES, CENTRAL_BAND_DB = "band_power_db.8-13", ("hold", "routine"), 6.0

# Tremor fit: the fitted joints, the stiffness scale grid, and per joint a reference drive
# density (a fraction of the stiffness, for sub-degree motion) the unit runs use.
FIT_JOINTS = ("wrist_flex", "wrist_dev", "sway_ry", "mcp_flex", "pip_flex", "mcp_abd", "sway_rx", "sway_rz")
WRIST_JOINTS = ("wrist_flex", "wrist_dev", "sway_ry")
# Slow wander of the held hand about the wrist's flexion and deviation axes: critically damped,
# so their damping scales with the square root of the stiffness scale.
DRIFT_JOINTS = ("sway_rx", "sway_rz")
# One fitted power weight per group: the middle-phalanx finger sensor turns with MCP and PIP
# flexion about the same axis and cannot tell them apart.
FIT_GROUPS = (("wrist_flex",), ("wrist_dev",), ("sway_ry",), ("mcp_flex", "pip_flex"), ("mcp_abd",), ("sway_rx",), ("sway_rz",))
STIFFNESS_SCALES = tuple(2.0 ** np.arange(-2.0, 3.01, 0.5))
# Drift resonance 0.5, 1 and 2 Hz.
DRIFT_SCALES = (0.25, 1.0, 4.0)
# The central oscillation: one narrow-band drive shared by the wrist and forearm DOF, a gain each.
# Its frequency is the median over trials of the hold's spectral peak in CENTRAL_SEARCH_HZ (above
# the mechanical peak): the 5 bands alone place a narrow peak at a band edge. Its half-power width
# is fitted within the bounds.
CENTRAL_JOINTS = WRIST_JOINTS
CENTRAL_SEARCH_HZ, CENTRAL_WIDTH_BOUNDS = (9.5, 16.0), (0.2, 5.0)
# Heartbeat (fundamental and 2nd harmonic) and breathing: unsupported drives, present on the table
# too. Both move the whole hand; the heartbeat also reaches the finger joints on its own gain.
# Fitted to the table spectrum over BODY_HZ, where they stand out.
HEART_JOINTS = (("sway_rx",), ("sway_ry",), ("sway_rz",), ("mcp_flex", "pip_flex"))
BREATH_JOINTS = (("sway_rx",), ("sway_ry",), ("sway_rz",))
HEART_HZ, HEART_WIDTH, HEART_H2 = (0.8, 2.0), (0.02, 0.2), (0.05, 3.0)
BREATH_HZ, BREATH_WIDTH = (0.15, 0.6), (0.02, 0.15)
BODY_HZ = (0.15, 3.5)
# The lines are fitted against the table spectrum's own background (a running median over
# BODY_BACKGROUND_HZ), at BODY_SEGMENT_S resolution: the table phase's broadband level is not the
# body's doing.
BODY_SEGMENT_S, BODY_BACKGROUND_HZ, BODY_GRID_HZ = 20.0, 0.8, 0.05


def band_shape(f: np.ndarray, frequency_hz: float, bandwidth_hz: float) -> np.ndarray:
    """|H|^2 of SharedDrive's band-pass for one harmonic: 1 at the centre, half at +-width/2."""
    w0, w, zw = 2 * np.pi * frequency_hz, 2 * np.pi * f, 2 * np.pi * bandwidth_hz
    return (zw * w) ** 2 / ((w0**2 - w**2) ** 2 + (zw * w) ** 2)


def family(joint: str) -> str:
    return "wrist" if joint in WRIST_JOINTS else "drift" if joint in DRIFT_JOINTS else "finger"


def scales(joint: str) -> tuple[float, ...]:
    return DRIFT_SCALES if joint in DRIFT_JOINTS else STIFFNESS_SCALES


def oscillator(joint: str, scale: float, lit: dict[str, dict[str, float]]) -> dict[str, float]:
    """The joint's stiffness (and for drift, damping) at a stiffness scale."""
    out = {"stiffness": lit[joint]["stiffness"] * scale}
    if joint in DRIFT_JOINTS:
        out["damping"] = lit[joint]["damping"] * float(np.sqrt(scale))
    return out
REFERENCE_DENSITY_PER_STIFFNESS = 2e-3
FIT_SECONDS = 300.0
# Table support, second pass: full-model table sims at these multiples of the band-power value,
# scored on the table's per-axis band powers. Linear acceleration is left out: on the table it
# reads the chips' static offset from 1 g, not motion.
SUPPORT_MULTIPLES = (0.5, 0.65, 0.8, 1.0, 1.2, 1.45)
# Table contact: the hand rests on the table and rocks about a held point rather than the wrist,
# and the finger is held at its own point. Rates do not change, so the support comes from the gyro
# and each pivot from its sensor's accel noise: palm pivot along the palm's y from the wrist,
# finger pivot along the segment's y from the PIP, each sweep in the order its accel noise falls.
PALM_PIVOT_Y = (0.0, 0.02, 0.04, 0.06, 0.08)
FINGER_PIVOT_Y = (-0.12, -0.09, -0.06, -0.03, 0.0)
# Routine roughness, fitted on session 1's routine split into move windows (each move from 0.1 s
# before to 0.3 s after) and hold windows (from 0.4 s after the out move to 0.1 s before the
# back move), band powers per axis over ROUGH_BANDS:
# - routine gain: the support during the routine, scaling every supported drive (held postures
#   after a move tremble more than the free hold);
# - roughness: a drive gated by each joint's scripted speed, per ROUGH_GROUPS group a weight, one
#   band shared, through the muscle filter and the joint (3-13 Hz);
# - mount jolts: rate added at each sensor while its segment moves, a level per sensor, one band
#   shared (the part above 13 Hz that is not coherent between the sensors).
ROUGH_BANDS = ((3.0, 8.0), (8.0, 13.0), (13.0, 25.0), (25.0, 100.0))
ROUGH_GROUPS = (("wrist_flex",), ("wrist_dev",), ("sway_ry",), ("mcp_flex", "pip_flex"), ("mcp_abd",))
# The roughness band is taken from a grid, each point's weights by least squares on the windows'
# band powers, the point chosen on those plus each sensor's routine dominant frequency (as the
# tremor fit's grid): band powers alone cannot tell the band's width.
ROUGH_HZ_GRID = (2.0, 2.5, 3.0, 4.5, 6.0, 7.5, 9.0)
ROUGH_WIDTH_GRID = (0.7, 1.5, 3.0, 6.0, 12.0)
# Then handedness: forearm roll's roughness swings the palm chip, above the roll axis, sideways,
# which the accelerometer reads as tilt the gyro does not predict, so it sets how much of the
# roughness is roll, which the band powers cannot. Roll's amplitude is scaled by each share, the
# other weights refitted, and session 1's routines simulated; the share that best matches the
# real handedness (at HANDEDNESS_TOL, with the band powers and dominant frequencies) wins.
ROLL_SHARES = (1.0, 0.5, 0.25, 0.0)
HANDEDNESS_TOL = 0.03
# Jolts are held above 13 Hz, where palm and finger are incoherent during moves.
JOLT_HZ, JOLT_WIDTH = (13.0, 120.0), (2.0, 200.0)
ROUTINE_GAIN = (0.5, 5.0)
MOVE_PAD_S, HOLD_PAD_S = (0.1, 0.3), (0.4, 0.1)
SUPPORT_SIM_S, SUPPORT_SEEDS = 60.0, 2
# Each axis's band power is compared with this fraction of the band's total over the axes added:
# a mount error of a degree or two leaks about -30 dB of the strong axis into the weak ones.
AXIS_FLOOR = 0.01
SIM_SEEDS = 3
RUNGS = ("ideal", "chip_transfer", "noise", "finger_tremor", "wrist_and_twist", "drift", "body", "roughness", "contact")
FINGER_KG = 0.008


def move_windows(name: str) -> list[tuple[float, float, float]]:
    """(start, end, sign) of each out (+1) and back (-1) stretch of a routine move."""
    m = next(k for k, (n, _) in enumerate(MOVES) if n == name)
    out = []
    for r in range(REPS):
        t0 = HOLD_END_S + (m * REPS + r) * REP_S
        out.append((t0, t0 + OUT_S, 1.0))
        out.append((t0 + OUT_S + HOLD_S, t0 + OUT_S + HOLD_S + BACK_S, -1.0))
    return out


def cues() -> list[tuple[float, str]]:
    out = [(0.0, "hand flat on the table, relaxed"), (LIFT_S, "lift to horizontal and hold")]
    for m, (name, _) in enumerate(MOVES):
        for r in range(REPS):
            t0 = HOLD_END_S + (m * REPS + r) * REP_S
            out += [(t0, f"{name.replace('_', ' ')} {r + 1}/{REPS}: out"), (t0 + OUT_S + HOLD_S, "back")]
    out.append((PHASES["routine"][1], "hand back on the table"))
    return out


# --- metrics


@dataclasses.dataclass
class Phase:
    t_s: np.ndarray
    accel_g: np.ndarray
    gyro_dps: np.ndarray
    odr_hz: float


def phases(cap: capture.Capture, name: str, names: typing.Iterable[str]) -> dict[str, Phase]:
    """The sensor's phase windows on the capture's timeline, gyro less its mean on the table."""
    s = cap.stream(name)
    t = rig.board_time(s) - rig.first_sample_s(cap, names)
    table = (t >= PHASES["table"][0]) & (t < PHASES["table"][1])
    zero = s.gyro_dps[table].mean(axis=0) if table.any() else np.zeros(3)
    out = {}
    for phase, (a, b) in PHASES.items():
        k = (t >= a) & (t < b)
        if k.sum() > s.odr_hz * 10:
            out[phase] = Phase(t[k], s.accel_g[k], s.gyro_dps[k] - zero, s.odr_hz)
    return out


def summed_psd(gyro_dps: np.ndarray, odr_hz: float) -> tuple[np.ndarray, np.ndarray]:
    f, p = bench.psd(gyro_dps[:, 0], odr_hz)
    for k in (1, 2):
        p = p + bench.psd(gyro_dps[:, k], odr_hz)[1]
    return f, p


def axis_psd(gyro_dps: np.ndarray, odr_hz: float, mount: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The PSD of each axis of the gyro in the segment frame, mount the sensor-to-segment
    rotation: rows x, y, z."""
    g = gyro_dps @ mount.T
    f, _ = bench.psd(g[:, 0], odr_hz)
    return f, np.array([bench.psd(g[:, k], odr_hz)[1] for k in range(3)])


def band_powers(f: np.ndarray, p: np.ndarray) -> np.ndarray:
    """dps^2 per band; bands past Nyquist are nan."""
    df = f[1] - f[0]
    return np.array([p[(f >= a) & (f < b)].sum() * df if b <= f[-1] else np.nan for a, b in BANDS])


def dominant_hz(f: np.ndarray, p: np.ndarray) -> float:
    k = (f >= DOMINANT_HZ[0]) & (f <= DOMINANT_HZ[1])
    return float(f[k][np.argmax(p[k])])


def handedness(accel_g: np.ndarray, gyro_dps: np.ndarray, odr_hz: float) -> float:
    """imu/conformance.hpp HandednessCheck's correlation with its default settings: +1
    right-handed, -1 left-handed, nan with nothing scored."""
    n = 24
    blocks = len(accel_g) // n
    a = accel_g[: blocks * n].reshape(blocks, n, 3).mean(axis=1)
    w = np.radians(gyro_dps[: blocks * n].reshape(blocks, n, 3).mean(axis=1))
    dt = n / odr_hz
    ok = np.abs(np.linalg.norm(a, axis=1) - 1.0) <= 0.1
    ok = ok[:-1] & ok[1:]
    ua = a[:-1] / np.linalg.norm(a[:-1], axis=1, keepdims=True)
    ub = a[1:] / np.linalg.norm(a[1:], axis=1, keepdims=True)
    mid = (ua + ub) / np.linalg.norm(ua + ub, axis=1, keepdims=True)
    measured = (ub - ua) / dt
    predicted = -np.cross(0.5 * (w[:-1] + w[1:]), mid)
    ok &= np.linalg.norm(predicted, axis=1) >= np.radians(20.0)
    mp, mm, pp = (np.einsum("ij,ij->i", x, y)[ok].sum() for x, y in ((measured, predicted), (measured, measured), (predicted, predicted)))
    return float(mp / np.sqrt(mm * pp)) if mm > 0 and pp > 0 else float("nan")


def phase_metrics(phase: str, p: Phase) -> dict[str, float]:
    f, psd = summed_psd(p.gyro_dps, p.odr_hz)
    rate = np.linalg.norm(p.gyro_dps, axis=1)
    norm = np.linalg.norm(p.accel_g, axis=1)
    n = int(PEAK_WINDOW_S * p.odr_hz)
    peaks = rate[: len(rate) // n * n].reshape(-1, n).max(axis=1)
    out = {f"band_power_db.{a:g}-{b:g}": 10 * np.log10(v) for (a, b), v in zip(BANDS, band_powers(f, psd)) if np.isfinite(v)}
    out |= {
        "dominant_hz": dominant_hz(f, psd),
        "gyro_std_dps": float(rate.std(ddof=1)),
        "median_peak_dps": float(np.median(peaks)),
        "linear_accel_g": float(np.percentile(np.abs(norm - 1.0), 95)),
    }
    if phase == "table":
        out["accel_noise_g"] = float(norm.std(ddof=1))
    if phase == "routine":
        out["handedness"] = handedness(p.accel_g, p.gyro_dps, p.odr_hz)
    return out


def lift_s(cap: capture.Capture, names: typing.Iterable[str]) -> float | None:
    """When the palm's |ω| first passes LIFT_DPS after the table phase."""
    s = cap.stream("palm")
    t = rig.board_time(s) - rig.first_sample_s(cap, names)
    zero = s.gyro_dps[(t >= PHASES["table"][0]) & (t < PHASES["table"][1])].mean(axis=0)
    fast = np.flatnonzero((t > PHASES["table"][1] - 10.0) & (np.linalg.norm(s.gyro_dps - zero, axis=1) > LIFT_DPS))
    return float(t[fast[0]]) if len(fast) else None


# --- scenarios


def _toml(v: typing.Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, (list, tuple, np.ndarray)):
        return "[" + ", ".join(_toml(x) for x in v) + "]"
    if isinstance(v, dict):
        return "{ " + ", ".join(f"{k} = {_toml(x)}" for k, x in v.items()) + " }"
    if isinstance(v, str):
        return f'"{v}"'
    return repr(float(v))


@dataclasses.dataclass
class Fit:
    """Session 1's fit: sensor-to-segment mounts and the tremor overrides."""

    palm_mount: np.ndarray
    finger_mount: np.ndarray
    tremor: dict[str, dict[str, float]]
    table_support: float
    drives: list[dict[str, typing.Any]] = dataclasses.field(default_factory=list)
    routine_gain: float = 1.0
    jolts: dict[str, dict[str, float]] = dataclasses.field(default_factory=dict)
    contact_pivot: list[float] | None = None
    finger_contact_pivot: list[float] | None = None

    def write(self, path: pathlib.Path) -> None:
        lines = [f"table_support = {self.table_support:.6g}", f"routine_gain = {self.routine_gain:.6g}"]
        for name in ("contact_pivot", "finger_contact_pivot"):
            if getattr(self, name) is not None:
                lines.append(f"{name} = {_toml(getattr(self, name))}")
        for name in ("palm_mount", "finger_mount"):
            lines.append(f"{name} = {_toml([list(r) for r in getattr(self, name)])}")
        for joint, keys in self.tremor.items():
            lines += [f"\n[tremor.{joint}]"] + [f"{k} = {v:.6g}" for k, v in keys.items()]
        lines += drive_lines(self.drives)
        lines += jolt_lines(self.jolts)
        path.write_text("\n".join(lines) + "\n")

    @classmethod
    def read(cls, path: pathlib.Path) -> "Fit":
        t = tomllib.loads(path.read_text())
        return cls(np.array(t["palm_mount"]), np.array(t["finger_mount"]), t["tremor"], t["table_support"], t.get("drives", []),
                   t.get("routine_gain", 1.0), t.get("jolt", {}), t.get("contact_pivot"), t.get("finger_contact_pivot"))


def jolt_lines(jolts: dict[str, dict[str, float]]) -> list[str]:
    out = []
    for n, j in jolts.items():
        out += [f"\n[jolt.{n}]"] + [f"{k} = {v:.9g}" for k, v in j.items()]
    return out


def drive_lines(drives: list[dict[str, typing.Any]]) -> list[str]:
    out = []
    for d in drives:
        out.append("\n[[drives]]")
        out += [f"{k} = {_toml(v) if not isinstance(v, float) else f'{v:.9g}'}" for k, v in d.items() if k != "gain"]
        out.append("gain = { " + ", ".join(f"{j} = {g:.9g}" for j, g in d["gain"].items()) + " }")
    return out


def axis_angle(m: np.ndarray) -> dict[str, typing.Any]:
    rv = scipy.spatial.transform.Rotation.from_matrix(m).as_rotvec()
    deg = float(np.degrees(np.linalg.norm(rv)))
    axis = rv / np.linalg.norm(rv) if deg > 0 else np.array([1.0, 0.0, 0.0])
    return {"axis": [float(x) for x in axis], "deg": deg}


@dataclasses.dataclass(frozen=True)
class Chips:
    """Each sensor's chip config header and measured ODR, from a real capture."""

    header: dict[str, dict[str, str]]
    odr_hz: dict[str, float]

    @classmethod
    def of(cls, cap: capture.Capture) -> "Chips":
        return cls({n: cap.stream(n).header for n in SENSORS}, {n: cap.stream(n).odr_hz for n in SENSORS})


def scenario(
    hand: dict[str, typing.Any],
    chips: Chips,
    params: dict[str, pathlib.Path],
    moves: list[str],
    *,
    seed: int,
    start_support: float,
    tremor: dict[str, dict[str, float]],
    mounts: tuple[np.ndarray, np.ndarray] | None = None,
    realism: dict[str, bool] | None = None,
    twist: bool = True,
    drives: list[dict[str, typing.Any]] | None = None,
    added_kg: float = 0.0,
    jolts: dict[str, dict[str, float]] | None = None,
    contact_pivot: list[float] | None = None,
    finger_contact_pivot: list[float] | None = None,
) -> str:
    geometry = dict(hand["geometry"])
    if not twist:
        geometry["twist_at_90_deg"] = 0.0
    if mounts is not None:
        geometry["palm_mount"], geometry["finger_mount"] = (axis_angle(m) for m in mounts)
    if contact_pivot is not None:
        geometry["contact_pivot"] = list(contact_pivot)
    if finger_contact_pivot is not None:
        geometry["finger_contact_pivot"] = list(finger_contact_pivot)
    lines = [f'name = "v5"', f"seed = {seed}", f'tremor_file = "{TREMOR_FILE}"']
    if realism:
        lines += ["[realism]"] + [f"{k} = {_toml(v)}" for k, v in realism.items()]
    lines += ["[geometry]"] + [f"{k} = {_toml(v)}" for k, v in geometry.items()]
    for n in SENSORS:
        nominal = float(chips.header[n]["odr_hz"])
        lines += [
            f"[{SLOT[n]}]",
            f"chip = {v1.chip_table(chips.header[n])}",
            f'params = "{params[n].resolve()}"',
            f"odr_scale = {chips.odr_hz[n] / nominal:.9f}",
        ]
    board = hand["board"]
    lines += ["[[masses]]", f"kg = {board['kg'] + added_kg:.6g}", 'at = "palm"', f"offset = {_toml(board['offset'])}"]
    lines += ["[[masses]]", f"kg = {FINGER_KG}", 'at = "finger"']
    for joint, keys in tremor.items():
        lines += [f"[tremor.{joint}]"] + [f"{k} = {v:.9g}" for k, v in keys.items()]
    lines += drive_lines(drives or [])
    lines += jolt_lines(jolts or {})
    lines += ["[start]", f"support = {start_support:.6g}", f"joints_deg = {_toml(ZERO_JOINTS)}"]
    return "\n".join(lines + moves) + "\n"


def hold_moves(seconds: float) -> list[str]:
    return ["[[moves]]", f"hold_s = {seconds:.3f}"]


@dataclasses.dataclass
class Rep:
    move: str
    joints: dict[str, float]
    out_start_s: float
    out_s: float
    back_start_s: float
    back_s: float
    out_shape: tuple[float, float] = MIN_JERK_SHAPE
    back_shape: tuple[float, float] = MIN_JERK_SHAPE


def fit_shape(tt: np.ndarray, angle: np.ndarray, start: float, duration: float, frm: float, to: float) -> tuple[float, float, tuple[float, float]]:
    """Start, duration and (a, b) of one move from frm to to on the angle trace, by least squares
    from the min-jerk estimate, over the move and a third of its length either side."""
    k = (tt >= start - duration / 3) & (tt < start + duration * 4 / 3)

    def curve(p: np.ndarray) -> np.ndarray:
        return frm + (to - frm) * scipy.special.betainc(p[2], p[3], np.clip((tt[k] - p[0]) / p[1], 0.0, 1.0))

    lo = [start - duration / 2, MIN_MOVE_S, SHAPE_BOUNDS[0], SHAPE_BOUNDS[0]]
    hi = [start + duration / 2, 3 * duration, SHAPE_BOUNDS[1], SHAPE_BOUNDS[1]]
    p0 = np.clip([start, duration, *MIN_JERK_SHAPE], np.array(lo) + 1e-6, np.array(hi) - 1e-6)
    p = scipy.optimize.least_squares(lambda p: curve(p) - angle[k], p0, bounds=(lo, hi)).x
    return float(p[0]), float(p[1]), (float(p[2]), float(p[3]))


def scripted_reps() -> list[Rep]:
    out = []
    for m, (name, target) in enumerate(MOVES):
        for r in range(REPS):
            t0 = HOLD_END_S + (m * REPS + r) * REP_S
            out.append(Rep(name, dict(target), t0, OUT_S, t0 + OUT_S + HOLD_S, BACK_S))
    return out


def measure_reps(cap: capture.Capture, mounts: tuple[np.ndarray, np.ndarray]) -> list[Rep]:
    """Each routine repetition's angle and timing (MOVE_AXIS). A repetition that cannot be found
    takes the median of its move's others in the capture (the script's if none). Index curl's
    angle is split between MCP and PIP as in the script."""
    streams = {}
    for n, m in zip(SENSORS, mounts):
        s = cap.stream(n)
        t = rig.board_time(s) - rig.first_sample_s(cap, SENSORS)
        table = (t >= PHASES["table"][0]) & (t < PHASES["table"][1])
        streams[n] = (t, (s.gyro_dps - s.gyro_dps[table].mean(axis=0)) @ m.T)
    t, palm = streams["palm"]
    finger = np.column_stack([np.interp(t, streams["finger"][0], streams["finger"][1][:, k]) for k in range(3)])
    rate = {"palm": palm, "finger": finger - palm}
    dt = np.gradient(t)

    def one(rep: Rep) -> Rep | None:
        src, axis, sign = MOVE_AXIS[rep.move]
        k = (t >= rep.out_start_s + MOVE_WINDOW_S[0]) & (t < rep.out_start_s + MOVE_WINDOW_S[1])
        tt = t[k]
        angle = np.cumsum(sign * rate[src][k, axis] * dt[k])
        angle -= np.median(angle[tt < tt[0] + 0.6])
        i_peak = int(np.argmax(np.abs(angle)))
        if abs(angle[i_peak]) < 3.0:
            return None
        plateau = np.abs(angle) >= 0.8 * abs(angle[i_peak])
        a = float(np.median(angle[plateau]))
        rest = float(np.median(angle[tt > tt[-1] - 0.6]))
        rise = angle / a
        fall = (angle - rest) / (a - rest)
        up, down = [], []
        for f in (0.1, 0.5, 0.9):
            i = np.flatnonzero(rise[: i_peak + 1] >= f)
            j = np.flatnonzero(fall[i_peak:] >= f)
            if not len(i) or not len(j) or i_peak + j[-1] + 1 >= len(tt):
                return None
            up.append(float(tt[i[0]]))
            down.append(float(tt[i_peak + j[-1] + 1]))
        out_s = max((up[2] - up[0]) / MIN_JERK_10_90, MIN_MOVE_S)
        back_s = max((down[0] - down[2]) / MIN_JERK_10_90, MIN_MOVE_S)
        out_start, out_s, out_shape = fit_shape(tt, angle, up[1] - out_s / 2, out_s, 0.0, a)
        back_start, back_s, back_shape = fit_shape(tt, angle, down[1] - back_s / 2, back_s, a, rest)
        target = dict(MOVES)[rep.move]
        total = sum(target.values())
        return Rep(rep.move, {j: a * v / total for j, v in target.items()}, out_start, out_s, back_start, back_s, out_shape, back_shape)

    scripted = scripted_reps()
    found = [one(r) for r in scripted]
    out = []
    for rep, got in zip(scripted, found):
        if got is not None:
            out.append(got)
            continue
        same = [(g, s) for g, s in zip(found, scripted) if g is not None and g.move == rep.move]
        if not same:
            out.append(rep)
            continue
        def med(f: typing.Callable[[Rep, Rep], float]) -> float:
            return float(np.median([f(g, s) for g, s in same]))
        start = rep.out_start_s + med(lambda g, s: g.out_start_s - s.out_start_s)
        out_s = med(lambda g, s: g.out_s)
        held = med(lambda g, s: g.back_start_s - g.out_start_s)
        joints = {j: med(lambda g, s, j=j: g.joints[j]) for j in rep.joints}
        shapes = [tuple(med(lambda g, s, i=i, d=d: getattr(g, d)[i]) for i in (0, 1)) for d in ("out_shape", "back_shape")]
        out.append(Rep(rep.move, joints, start, out_s, start + held, med(lambda g, s: g.back_s), shapes[0], shapes[1]))
    return out


def timeline_moves(reps: list[Rep] | None = None, routine_gain: float = 1.0) -> list[str]:
    """The capture's phases: table, lift to support 1, free hold, then the routine, scripted or
    as measured (measure_reps), its support raised to routine_gain over the first move."""
    out = hold_moves(LIFT_S) + ["[[moves]]", f"duration_s = {LIFT_DURATION_S}", "support = 1.0"]
    now = LIFT_S + LIFT_DURATION_S
    first = True
    for rep in scripted_reps() if reps is None else reps:
        for start, duration, joints, shape in ((rep.out_start_s, rep.out_s, ZERO_JOINTS | rep.joints, rep.out_shape),
                                               (rep.back_start_s, rep.back_s, ZERO_JOINTS, rep.back_shape)):
            if start > now:
                out += hold_moves(start - now)
                now = start
            out += ["[[moves]]", f"duration_s = {duration:.4f}", f"joints_deg = {_toml(joints)}"]
            out += ['profile = "min_jerk"'] if shape == MIN_JERK_SHAPE else ['profile = "beta"', f"shape = {_toml(shape)}"]
            if first and routine_gain != 1.0:
                out.append(f"support = {routine_gain:.6g}")
            first = False
            now += duration
    return out + hold_moves(max(CAPTURE_S - now, 0.0))


def simulate(scen_text: str, path: pathlib.Path) -> capture.Capture:
    path.write_text(scen_text)
    sim.run(path, path.with_suffix(".bin"), truth=False)
    cap = capture.read_capture(path.with_suffix(".bin"))
    for n in SENSORS:
        cap.stream(SLOT[n]).header["name"] = n
    return cap


def literature() -> dict[str, dict[str, float]]:
    t = tomllib.loads(TREMOR_FILE.read_text())
    return {j: t[j] for j in FIT_JOINTS}


# --- session-1 fit


def fit_mounts(caps: list[capture.Capture]) -> tuple[np.ndarray, np.ndarray]:
    """Sensor-to-segment rotations from the table's gravity (segment z) and the mean gyro axis of
    the wrist flexion and extension (palm) and index curl (finger) moves (flex axis, palm -x)."""
    up, flex = np.array([0.0, 0.0, 1.0]), np.array([-1.0, 0.0, 0.0])
    mounts = []
    for n, moves in (("palm", (("wrist_flexion", 1.0), ("wrist_extension", -1.0))), ("finger", (("index_curl", 1.0),))):
        gravity, axis = np.zeros(3), np.zeros(3)
        for cap in caps:
            ph = phases(cap, n, SENSORS)
            gravity += ph["table"].accel_g.mean(axis=0)
            s = cap.stream(n)
            t = rig.board_time(s) - rig.first_sample_s(cap, SENSORS)
            zero = s.gyro_dps[(t >= PHASES["table"][0]) & (t < PHASES["table"][1])].mean(axis=0)
            for move, sign in moves:
                for a, b, direction in move_windows(move):
                    k = (t >= a) & (t < b)
                    axis += sign * direction * (s.gyro_dps[k] - zero).sum(axis=0)
        sensor = np.array([gravity / np.linalg.norm(gravity), axis / np.linalg.norm(axis)])
        mounts.append(rig.fit_rotation(np.array([up, flex]), sensor))
    return mounts[0], mounts[1]


@dataclasses.dataclass
class UnitRuns:
    """Per sensor, the segment-frame gyro PSD per axis of the hold with no tremor, and of each
    joint driven alone at its reference density with chip noise off, by stiffness scale."""

    f: dict[str, np.ndarray]
    none: dict[str, np.ndarray]
    joint: dict[tuple[str, float], dict[str, np.ndarray]]


def unit_runs(hand: dict[str, typing.Any], chips: Chips, params: dict[str, pathlib.Path], mounts: tuple[np.ndarray, np.ndarray], work_dir: pathlib.Path, log: rig.Log) -> UnitRuns:
    lit = literature()

    def hold_psd(tremor: dict[str, dict[str, float]], realism: dict[str, bool], stem: str, seed: int) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        text = scenario(hand, chips, params, hold_moves(FIT_SECONDS), seed=seed, start_support=1.0, tremor=tremor, mounts=mounts, realism=realism)
        cap = simulate(text, work_dir / f"{stem}.toml")
        f, p = {}, {}
        for n, mount in zip(SENSORS, mounts):
            s = cap.stream(n)
            k = s.t_s >= PHASES["table"][0]
            f[n], p[n] = axis_psd(s.gyro_dps[k], s.odr_hz, mount)
        return f, p

    off = {j: {"torque_density": 0.0} for j in FIT_JOINTS}
    f, none = hold_psd(off, {"tremor": False}, "unit-none", 1)
    joint = {}
    for j in FIT_JOINTS:
        for k, scale in enumerate(scales(j)):
            osc = oscillator(j, scale, lit)
            tremor = off | {j: osc | {"torque_density": REFERENCE_DENSITY_PER_STIFFNESS * osc["stiffness"]}}
            joint[(j, scale)] = hold_psd(tremor, {"noise": False}, f"unit-{j}-{k}", 100 + k)[1]
        log(f"unit runs: {j} done")
    return UnitRuns(f, none, joint)


# Per sensor: band powers in dB per segment-frame axis (3 x bands) and the dominant frequency.
Targets = dict[str, tuple[np.ndarray, float]]


def axis_bands_db(f: np.ndarray, psd: np.ndarray) -> np.ndarray:
    return 10 * np.log10(np.array([band_powers(f, a) for a in psd]))


def floored_db(bands_db: np.ndarray) -> np.ndarray:
    p = 10 ** (bands_db / 10)
    return 10 * np.log10(p + AXIS_FLOOR * p.sum(axis=0))


def table_spectra(caps: list[capture.Capture], mounts: tuple[np.ndarray, np.ndarray]) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """The body grid over BODY_HZ and per sensor the mean segment-frame axis PSD of the table
    phase on it, BODY_SEGMENT_S segments, rows x, y, z."""
    f = np.arange(BODY_HZ[0], BODY_HZ[1] + BODY_GRID_HZ / 2, BODY_GRID_HZ)
    out = {}
    for n, m in zip(SENSORS, mounts):
        psds = []
        for c in caps:
            p = phases(c, n, SENSORS)["table"]
            g = p.gyro_dps @ m.T
            fc, _ = bench.psd(g[:, 0], p.odr_hz, BODY_SEGMENT_S)
            psds.append([np.interp(f, fc, bench.psd(g[:, k], p.odr_hz, BODY_SEGMENT_S)[1]) for k in range(3)])
        out[n] = np.mean(psds, axis=0)
    return f, out


def hold_peak_hz(
    cap: capture.Capture, mounts: tuple[np.ndarray, np.ndarray], search: tuple[float, float], smooth_hz: float = 0.0
) -> float:
    """The hold's spectral peak in search, all axes of both sensors, each sensor's PSD over its
    median, after a running mean over smooth_hz."""
    total = None
    for n, m in zip(SENSORS, mounts):
        p = phases(cap, n, SENSORS)["hold"]
        f, psd = axis_psd(p.gyro_dps, p.odr_hz, m)
        if total is None:
            grid, total = f, psd.sum(axis=0) / np.median(psd.sum(axis=0))
        else:
            total = total + np.interp(grid, f, psd.sum(axis=0)) / np.median(psd.sum(axis=0))
    width = max(1, int(round(smooth_hz / (grid[1] - grid[0]))))
    total = np.convolve(total, np.ones(width) / width, mode="same")
    k = (grid >= search[0]) & (grid <= search[1])
    return float(grid[k][np.argmax(total[k])])


def central_hz(caps: list[capture.Capture], mounts: tuple[np.ndarray, np.ndarray]) -> float:
    """Median over the captures of the hold's peak in CENTRAL_SEARCH_HZ."""
    return float(np.median([hold_peak_hz(c, mounts, CENTRAL_SEARCH_HZ) for c in caps]))


def fit_targets(caps: list[capture.Capture], mounts: tuple[np.ndarray, np.ndarray], phase: str) -> Targets:
    """The fit's targets, means over the captures. Per axis, so that joints turning about
    different axes (flexion, abduction) are told apart."""
    out = {}
    for n, mount in zip(SENSORS, mounts):
        bands, dominant = [], []
        for cap in caps:
            p = phases(cap, n, SENSORS)[phase]
            f, psd = axis_psd(p.gyro_dps, p.odr_hz, mount)
            bands.append(axis_bands_db(f, psd))
            dominant.append(dominant_hz(f, psd.sum(axis=0)))
        out[n] = (np.mean(bands, axis=0), float(np.mean(dominant)))
    return out


def fit_tremor(
    units: UnitRuns, real_hold: Targets, real_table: Targets, log: rig.Log, central_frequency_hz: float,
    table_spectrum: tuple[np.ndarray, dict[str, np.ndarray]] | None = None,
) -> tuple[dict[str, dict[str, float]], list[dict[str, typing.Any]], float, list[str]]:
    """Stiffness scales on the grid; per FIT_GROUPS group a power weight x >= 0 (density = sqrt(x)
    times the reference), per CENTRAL_JOINTS joint a central-drive weight at central_frequency_hz,
    and its width, by least squares; then the table's support;
    then, given the table spectra, the heartbeat and breathing (fit_body).
    Returns the overrides, the drives, the support and a fit report."""
    lit = literature()
    terms = [(group, False) for group in FIT_GROUPS] + [((j,), True) for j in CENTRAL_JOINTS]
    nw = len(terms)

    def predicted(p: np.ndarray, at: dict[str, float]) -> dict[str, np.ndarray]:
        x, fc, bw = np.exp(p[:nw]), central_frequency_hz, float(np.exp(p[nw]))
        out = {}
        for n in SENSORS:
            shape = band_shape(units.f[n], fc, bw)
            out[n] = units.none[n] + sum(
                xi * sum(units.joint[(j, at[family(j)])][n] for j in group) * (shape if central else 1.0)
                for xi, (group, central) in zip(x, terms)
            )
        return out

    def residuals(psds: dict[str, np.ndarray], real: Targets, with_frequency: bool) -> np.ndarray:
        out: list[float] = []
        for n in SENSORS:
            f = units.f[n]
            bands, dominant = real[n]
            d = floored_db(axis_bands_db(f, psds[n])) - floored_db(bands)
            out += list(d[np.isfinite(d)] / ABSOLUTE["band_power_db"])
            if with_frequency:
                out.append((dominant_hz(f, psds[n].sum(axis=0)) - dominant) / ABSOLUTE["dominant_hz"])
        return np.array(out)

    lo = [-40.0] * nw + [np.log(CENTRAL_WIDTH_BOUNDS[0])]
    hi = [20.0] * nw + [np.log(CENTRAL_WIDTH_BOUNDS[1])]
    best: tuple[float, np.ndarray, dict[str, float]] | None = None
    for wrist, finger, drift in itertools.product(STIFFNESS_SCALES, STIFFNESS_SCALES, DRIFT_SCALES):
        at = {"wrist": wrist, "finger": finger, "drift": drift}
        p0 = np.concatenate([np.zeros(nw), [0.0]])
        fitted = scipy.optimize.least_squares(lambda p: residuals(predicted(p, at), real_hold, False), p0, bounds=(lo, hi))
        cost = float(np.sum(residuals(predicted(fitted.x, at), real_hold, True) ** 2))
        if best is None or cost < best[0]:
            best = (cost, fitted.x, at)
    assert best is not None
    cost, p, at = best
    full = predicted(p, at)

    def table_psd(s: float) -> dict[str, np.ndarray]:
        return {n: units.none[n] + s * s * (full[n] - units.none[n]) for n in SENSORS}

    support = float(np.exp(scipy.optimize.least_squares(lambda v: residuals(table_psd(np.exp(v[0])), real_table, False), [0.0]).x[0]))
    x = np.exp(p[:nw])
    tremor: dict[str, dict[str, float]] = {}
    gain: dict[str, float] = {}
    for xi, (group, central) in zip(x, terms):
        for j in group:
            osc = oscillator(j, at[family(j)], lit)
            density = float(np.sqrt(xi)) * REFERENCE_DENSITY_PER_STIFFNESS * osc["stiffness"]
            if central:
                gain[j] = density
            else:
                tremor[j] = osc | {"torque_density": density}
    drives = [{"name": "central", "frequency_hz": central_frequency_hz, "bandwidth_hz": float(np.exp(p[nw])),
               "harmonics": [1.0], "gate": "support", "gain": gain}]
    report = [f"stiffness scale wrist {at['wrist']:g} finger {at['finger']:g} drift {at['drift']:g}, cost {cost:.3g}, table support {support:.3g}"]
    report += [f"{j}: stiffness {v['stiffness']:.4g} density {v['torque_density']:.4g}" for j, v in tremor.items()]
    report.append(f"central {drives[0]['frequency_hz']:.2f} Hz, width {drives[0]['bandwidth_hz']:.2f} Hz, gain "
                  + ", ".join(f"{j} {g:.4g}" for j, g in gain.items()))
    if table_spectrum is not None:
        body, body_report = fit_body(units, at, *table_spectrum, lit)
        drives += body
        report += body_report
    for line in report:
        log(line)
    return tremor, drives, support, report


def fit_body(
    units: UnitRuns, at: dict[str, float], f: np.ndarray, real: dict[str, np.ndarray],
    lit: dict[str, dict[str, float]],
) -> tuple[list[dict[str, typing.Any]], list[str]]:
    """Heartbeat and breathing over the real table spectrum's background (table_spectra's grid f):
    frequencies, widths, the heartbeat's 2nd harmonic and a power weight per HEART_JOINTS /
    BREATH_JOINTS group, by least squares on the log spectrum, every sensor and axis."""
    nh, nb = len(HEART_JOINTS), len(BREATH_JOINTS)
    half = int(round(BODY_BACKGROUND_HZ / BODY_GRID_HZ / 2))
    base = {n: scipy.ndimage.median_filter(real[n], size=(1, 2 * half + 1), mode="nearest") for n in SENSORS}
    unit = {n: [np.array([np.interp(f, units.f[n], row) for row in sum(units.joint[(j, at[family(j)])][n] for j in g)])
                for g in HEART_JOINTS + BREATH_JOINTS] for n in SENSORS}

    def model(p: np.ndarray) -> dict[str, np.ndarray]:
        x = np.exp(p[: nh + nb])
        fh, wh, h2, fb, wb = np.exp(p[nh + nb:])
        out = {}
        for n in SENSORS:
            heart = band_shape(f, fh, wh) + h2 * h2 * band_shape(f, 2 * fh, 2 * wh)
            breath = band_shape(f, fb, wb)
            out[n] = base[n] + sum(xi * u * (heart if i < nh else breath) for i, (xi, u) in enumerate(zip(x, unit[n])))
        return out

    def residuals(p: np.ndarray) -> np.ndarray:
        m = model(p)
        return np.concatenate([(10 * np.log10(m[n] / real[n])).ravel() / ABSOLUTE["band_power_db"] for n in SENSORS])

    def peak(lo: float, hi: float) -> float:
        k = (f >= lo) & (f <= hi)
        return float(f[k][np.argmax(sum((real[n] / base[n]).sum(axis=0) for n in SENSORS)[k])])

    lo = [-40.0] * (nh + nb) + list(np.log([HEART_HZ[0], HEART_WIDTH[0], HEART_H2[0], BREATH_HZ[0], BREATH_WIDTH[0]]))
    hi = [20.0] * (nh + nb) + list(np.log([HEART_HZ[1], HEART_WIDTH[1], HEART_H2[1], BREATH_HZ[1], BREATH_WIDTH[1]]))
    best = None
    for fh0 in (peak(*HEART_HZ), peak(2 * HEART_HZ[0], min(2 * HEART_HZ[1], BODY_HZ[1])) / 2):
        p0 = np.concatenate([np.full(nh + nb, -6.0), np.log([fh0, 0.1, 1.0, peak(*BREATH_HZ), 0.08])])
        r = scipy.optimize.least_squares(residuals, np.clip(p0, np.array(lo) + 1e-9, np.array(hi) - 1e-9), bounds=(lo, hi))
        if best is None or r.cost < best.cost:
            best = r
    p = best.x
    x = np.exp(p[: nh + nb])
    fh, wh, h2, fb, wb = (float(v) for v in np.exp(p[nh + nb:]))

    def gains(groups: tuple[tuple[str, ...], ...], xs: np.ndarray) -> dict[str, float]:
        return {j: float(np.sqrt(xi)) * REFERENCE_DENSITY_PER_STIFFNESS * oscillator(j, at[family(j)], lit)["stiffness"]
                for xi, g in zip(xs, groups) for j in g}

    drives = [
        {"name": "heartbeat", "frequency_hz": fh, "bandwidth_hz": wh, "harmonics": [1.0, h2], "gate": "none",
         "gain": gains(HEART_JOINTS, x[:nh])},
        {"name": "breathing", "frequency_hz": fb, "bandwidth_hz": wb, "harmonics": [1.0], "gate": "none",
         "gain": gains(BREATH_JOINTS, x[nh:])},
    ]
    rms = float(np.sqrt(np.mean((residuals(p) * ABSOLUTE["band_power_db"]) ** 2)))
    before = float(np.sqrt(np.mean((residuals(np.concatenate([np.full(nh + nb, -40.0), p[nh + nb:]])) * ABSOLUTE["band_power_db"]) ** 2)))
    report = [f"heartbeat {fh:.2f} Hz width {wh:.2f} 2nd harmonic {h2:.2f}, breathing {fb:.2f} Hz width {wb:.2f}; "
              f"table spectrum {BODY_HZ[0]:g}-{BODY_HZ[1]:g} Hz rms {before:.1f} dB without, {rms:.1f} dB with"]
    for d in drives:
        report.append(f"{d['name']} gain " + ", ".join(f"{j} {g:.4g}" for j, g in d["gain"].items()))
    return drives, report


# --- data


def all_trials(data_dir: pathlib.Path) -> list[rig.Entry]:
    """V5 captures with both hand sensors usable."""
    out = []
    for e in manifest.load(manifest.path(data_dir)):
        if e.get("test") == "V5" and all(v1.usable(e, n) for n in SENSORS):
            out.append(rig.Entry(data_dir / e["file"], "V5", e["config"], int(e["session"]), e.get("notes", "")))
    return out


def trials(data_dir: pathlib.Path) -> list[rig.Entry]:
    """The full-timeline V5 trials."""
    return [t for t in all_trials(data_dir) if not t.notes.startswith(MASS_TAG)]


def added_grams(notes: str) -> float | None:
    """The added mass of a mass-loading capture, from its notes "mass-loading +100 g ..."."""
    m = re.match(re.escape(MASS_TAG) + r" \+([0-9.]+) g", notes)
    return float(m.group(1)) if m else None


def mass_trials(data_dir: pathlib.Path) -> list[tuple[rig.Entry, float]]:
    return [(t, g) for t in all_trials(data_dir) if (g := added_grams(t.notes)) is not None]


def check_lift(cap: capture.Capture, path: pathlib.Path, log: rig.Log) -> bool:
    t = lift_s(cap, SENSORS)
    ok = t is not None and LIFT_S - LIFT_EARLY_S <= t <= LIFT_S + LIFT_LATE_S
    if not ok:
        log(f"{path.name}: lift at {t} s, not within {LIFT_S - LIFT_EARLY_S:g}-{LIFT_S + LIFT_LATE_S:g} s; redo the trial")
    return ok


def real_metrics(cap: capture.Capture) -> dict[str, dict[str, dict[str, float]]]:
    """sensor -> phase -> metric -> value."""
    out: dict[str, dict[str, dict[str, float]]] = {}
    for n in SENSORS:
        out[n] = {phase: phase_metrics(phase, p) for phase, p in phases(cap, n, SENSORS).items()}
    return out


def contact_args(fitted: "Fit") -> dict[str, typing.Any]:
    return {"contact_pivot": fitted.contact_pivot, "finger_contact_pivot": fitted.finger_contact_pivot}


def rung_drives(rung: str, drives: list[dict[str, typing.Any]]) -> list[dict[str, typing.Any]]:
    """The central drive from wrist_and_twist on, heartbeat and breathing from body on, the
    movement roughness from roughness on."""
    k = RUNGS.index(rung)
    first = {"central": "wrist_and_twist", "roughness": "roughness"}
    return [d for d in drives if k >= RUNGS.index(first.get(d.get("name", ""), "body"))]


def rung_settings(rung: str) -> tuple[dict[str, bool], bool, set[str]]:
    """Realism flags, finger twist, and the joints left without tremor on this rung."""
    k = RUNGS.index(rung)
    realism = {"chip_transfer": k >= 1, "noise": k >= 2, "tremor": k >= 3}
    return realism, k >= 4, (set(WRIST_JOINTS) if k < 4 else set()) | (set(DRIFT_JOINTS) if k < 5 else set())


def fit_support(
    hand: dict[str, typing.Any], chips: Chips, params: dict[str, pathlib.Path], mounts: tuple[np.ndarray, np.ndarray],
    caps: list[capture.Capture], tremor: dict[str, dict[str, float]], drives: list[dict[str, typing.Any]],
    first: float, work_dir: pathlib.Path, log: rig.Log,
) -> float:
    """The table support from table sims around first (SUPPORT_MULTIPLES): per-axis band powers
    as in the tremor fit; the minimum of a parabola in log support through the best grid point
    and its neighbours."""
    real_bands = fit_targets(caps, mounts, "table")
    grid = [first * m for m in SUPPORT_MULTIPLES]
    costs = []
    for s in grid:
        sims = [simulate(scenario(hand, chips, params, hold_moves(SUPPORT_SIM_S), seed=500 + i, start_support=s, tremor=tremor,
                                  mounts=mounts, drives=drives), work_dir / f"support-{s:.4f}-{i}.toml") for i in range(SUPPORT_SEEDS)]
        bands = fit_targets(sims, mounts, "table")
        cost = 0.0
        for n in SENSORS:
            d = floored_db(bands[n][0]) - floored_db(real_bands[n][0])
            cost += float(np.sum((d[np.isfinite(d)] / ABSOLUTE["band_power_db"]) ** 2))
        costs.append(cost)
        log(f"table support {s:.4g}: cost {cost:.3g}")
    i = int(np.clip(np.argmin(costs), 1, len(grid) - 2))
    x, y = np.log(grid[i - 1: i + 2]), np.array(costs[i - 1: i + 2])
    a, b, _ = np.polyfit(x, y, 2)
    best = float(np.exp(np.clip(-b / (2 * a), x[0], x[-1]))) if a > 0 else grid[int(np.argmin(costs))]
    log(f"table support {best:.4g} from table sims (from the unit runs {first:.4g})")
    return best


def fit_contact(
    hand: dict[str, typing.Any], chips: Chips, params: dict[str, pathlib.Path], mounts: tuple[np.ndarray, np.ndarray],
    caps: list[capture.Capture], fitted: "Fit", work_dir: pathlib.Path, log: rig.Log,
) -> tuple[list[float], list[float]]:
    """Palm and finger contact pivots: table sims with the pivots stepped together along
    PALM_PIVOT_Y and FINGER_PIVOT_Y (independent: the finger's held point fixes its own
    acceleration), each set where its sensor's accel noise, interpolated in log, meets the real."""
    real = {n: float(np.mean([real_metrics(c)[n]["table"]["accel_noise_g"] for c in caps])) for n in SENSORS}
    noise: dict[str, list[float]] = {n: [] for n in SENSORS}
    for py, fy in zip(PALM_PIVOT_Y, FINGER_PIVOT_Y):
        sims = [simulate(scenario(hand, chips, params, hold_moves(SUPPORT_SIM_S), seed=600 + i, start_support=fitted.table_support,
                                  tremor=fitted.tremor, mounts=mounts, drives=rung_drives("body", fitted.drives),
                                  contact_pivot=[0.0, py, 0.0], finger_contact_pivot=[0.0, fy, 0.0]),
                         work_dir / f"contact-{py:.3f}-{fy:.3f}-{i}.toml") for i in range(SUPPORT_SEEDS)]
        for n in SENSORS:
            noise[n].append(float(np.mean([real_metrics(c)[n]["table"]["accel_noise_g"] for c in sims])))
        log(f"contact pivots palm y {py:.3f} finger y {fy:.3f}: accel noise palm {noise['palm'][-1] * 1e3:.3g} mg, "
            f"finger {noise['finger'][-1] * 1e3:.3g} mg (real {real['palm'] * 1e3:.3g}, {real['finger'] * 1e3:.3g})")

    def place(ys: tuple[float, ...], values: list[float], target: float) -> float:
        v = np.log(values)
        order = np.argsort(v)
        return float(np.interp(np.log(target), v[order], np.array(ys)[order]))

    palm_y, finger_y = place(PALM_PIVOT_Y, noise["palm"], real["palm"]), place(FINGER_PIVOT_Y, noise["finger"], real["finger"])
    log(f"contact pivots: palm y {palm_y:.3f} m from the wrist, finger y {finger_y:.3f} m from the PIP")
    return [0.0, palm_y, 0.0], [0.0, finger_y, 0.0]


def fitted_scales(tremor: dict[str, dict[str, float]]) -> dict[str, float]:
    """The grid scale of each stiffness family in a fitted tremor."""
    lit = literature()
    out = {}
    for j, v in tremor.items():
        ratio = v["stiffness"] / lit[j]["stiffness"]
        out[family(j)] = min(scales(j), key=lambda x: abs(np.log(x / ratio)))
    return out


def progress_rate(t: np.ndarray, start: float, duration: float, shape: tuple[float, float]) -> np.ndarray:
    """d/dt of the beta progress (min-jerk at (3, 3)) from start over duration, 1/s."""
    x = (t - start) / duration
    inside = (x > 0.0) & (x < 1.0)
    return np.where(inside, scipy.stats.beta.pdf(np.clip(x, 1e-12, 1 - 1e-12), *shape), 0.0) / duration


def scripted_rates(reps: list[Rep], t: np.ndarray) -> dict[str, np.ndarray]:
    """Each joint's scripted rate at times t, rad/s."""
    out = {j: np.zeros_like(t) for j in ZERO_JOINTS}  # sway_ry, unscripted, takes the wrist's speed
    for rep in reps:
        out_rate = progress_rate(t, rep.out_start_s, rep.out_s, rep.out_shape)
        back_rate = progress_rate(t, rep.back_start_s, rep.back_s, rep.back_shape)
        for j, deg in rep.joints.items():
            out[j] += np.radians(deg) * (out_rate - back_rate)
    out["sway_ry"] = np.hypot(out["wrist_flex"], out["wrist_dev"])
    return out


def segment_speeds(rates: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    """The gates of the mount jolts, as sim/main.cpp computes them."""
    return {"palm": np.hypot(rates["wrist_flex"], rates["wrist_dev"]),
            "finger": np.sqrt((rates["wrist_flex"] + rates["mcp_flex"] + rates["pip_flex"]) ** 2
                              + rates["wrist_dev"] ** 2 + rates["mcp_abd"] ** 2)}


def routine_masks(reps: list[Rep], t: np.ndarray) -> dict[str, np.ndarray]:
    moves = np.zeros(len(t), bool)
    holds = np.zeros(len(t), bool)
    for rep in reps:
        for start, duration in ((rep.out_start_s, rep.out_s), (rep.back_start_s, rep.back_s)):
            moves |= (t >= start - MOVE_PAD_S[0]) & (t < start + duration + MOVE_PAD_S[1])
        holds |= (t >= rep.out_start_s + rep.out_s + HOLD_PAD_S[0]) & (t < rep.back_start_s - HOLD_PAD_S[1])
    routine = (t >= PHASES["routine"][0]) & (t < PHASES["routine"][1])
    return {"moves": moves, "holds": holds, "routine": routine}


def band_power_lin(f: np.ndarray, psd: np.ndarray, bands: tuple[tuple[float, float], ...]) -> np.ndarray:
    df = f[1] - f[0]
    return np.array([[row[(f >= a) & (f < b)].sum() * df for a, b in bands] for row in np.atleast_2d(psd)])


def routine_psds(
    cap: capture.Capture, mounts: tuple[np.ndarray, np.ndarray], reps: list[Rep], grid: dict[str, np.ndarray]
) -> dict[tuple[str, str], np.ndarray]:
    """(window, sensor) -> PSD per segment axis on grid[sensor], of the gyro over the window's
    samples (the rest zeroed, the PSD scaled up by the window's share)."""
    out = {}
    for n, m in zip(SENSORS, mounts):
        s = cap.stream(n)
        t = rig.board_time(s) - rig.first_sample_s(cap, SENSORS)
        table = (t >= PHASES["table"][0]) & (t < PHASES["table"][1])
        g = (s.gyro_dps - s.gyro_dps[table].mean(axis=0)) @ m.T
        for w, mask in routine_masks(reps, t).items():
            x = np.where(mask[:, None], g, 0.0)
            f, _ = bench.psd(x[:, 0], s.odr_hz)
            psd = np.array([bench.psd(x[:, k], s.odr_hz)[1] for k in range(3)]) / max(mask.mean(), 1e-9)
            out[(w, n)] = np.array([np.interp(grid[n], f, row) for row in psd])
    return out


def fit_routine(
    hand: dict[str, typing.Any], params: dict[str, pathlib.Path], mounts: tuple[np.ndarray, np.ndarray],
    caps: list[capture.Capture], units: UnitRuns, at: dict[str, float], fitted: "Fit", work_dir: pathlib.Path, log: rig.Log,
) -> tuple[float, dict[str, typing.Any], dict[str, dict[str, float]], list[str]]:
    """The routine gain, the roughness drive and the mount jolts (see ROUGH_BANDS). Each capture's
    routine is replayed in two sims, the full model and the same without tremor; tremor scales
    as gain^2 between them, and the roughness and jolts add as each source's unit response times
    its band and the window's mean squared gate. For each ROUGH_HZ_GRID x ROUGH_WIDTH_GRID point,
    least squares on the move and hold windows' band powers; the point that also best matches
    each sensor's routine dominant frequency wins."""
    lit = literature()
    real_rows, base, tremorless, gates, real_dominant = [], [], [], [], []
    for i, cap in enumerate(caps):
        reps = measure_reps(cap, mounts)
        chips = Chips.of(cap)
        real_rows.append(routine_psds(cap, mounts, reps, units.f))
        real_dominant.append({n: real_metrics(cap)[n]["routine"]["dominant_hz"] for n in SENSORS})
        sims = []
        for label, realism in (("full", None), ("still", {"tremor": False})):
            text = scenario(hand, chips, params, timeline_moves(reps), seed=900 + i, start_support=fitted.table_support,
                            tremor=fitted.tremor, mounts=mounts, drives=fitted.drives, realism=realism)
            sims.append(routine_psds(simulate(text, work_dir / f"routine-{label}-{i}.toml"), mounts, reps, units.f))
        base.append(sims[0])
        tremorless.append(sims[1])
        t = np.arange(PHASES["routine"][0] - 5.0, CAPTURE_S, 0.002)
        rates = scripted_rates(reps, t)
        speeds = segment_speeds(rates)
        gates.append({w: ({j: float(np.mean(rates[j][mk] ** 2)) for j in rates} | {f"segment.{n}": float(np.mean(speeds[n][mk] ** 2)) for n in SENSORS})
                      for w, mk in routine_masks(reps, t).items()})
    keys = list(real_rows[0])
    fitted_keys = [k for k in keys if k[0] != "routine"]
    real = {k: np.mean([10 * np.log10(band_power_lin(units.f[k[1]], r[k], ROUGH_BANDS)) for r in real_rows], axis=0) for k in keys}
    dominant = {n: float(np.mean([d[n] for d in real_dominant])) for n in SENSORS}
    b = {k: np.mean([r[k] for r in base], axis=0) for k in keys}
    still = {k: np.mean([r[k] for r in tremorless], axis=0) for k in keys}
    gate = {w: {g: float(np.mean([x[w][g] for x in gates])) for g in gates[0][w]} for w in gates[0]}
    ng = len(ROUGH_GROUPS)

    def model_psd(p: np.ndarray, fr: float, wr: float) -> dict[tuple[str, str], np.ndarray]:
        g = np.exp(p[0])
        x = np.exp(p[1: 1 + ng])
        fj, wj = np.exp(p[1 + ng: 3 + ng])
        level = dict(zip(SENSORS, np.exp(p[3 + ng: 5 + ng])))
        out = {}
        for (w, n) in keys:
            f = units.f[n]
            m = still[(w, n)] + g * g * np.clip(b[(w, n)] - still[(w, n)], 0.0, None)
            shape_r = band_shape(f, fr, wr)
            for xi, group in zip(x, ROUGH_GROUPS):
                for j in group:
                    m = m + xi * gate[w][j] * units.joint[(j, at[family(j)])][n] * shape_r
            out[(w, n)] = m + level[n] ** 2 * gate[w][f"segment.{n}"] * band_shape(f, fj, wj)
        return out

    def band_residuals(m: dict[tuple[str, str], np.ndarray]) -> np.ndarray:
        return np.concatenate([((floored_db(10 * np.log10(band_power_lin(units.f[k[1]], m[k], ROUGH_BANDS))) - floored_db(real[k]))
                                / ABSOLUTE["band_power_db"]).ravel() for k in fitted_keys])

    def dominant_residuals(m: dict[tuple[str, str], np.ndarray]) -> np.ndarray:
        return np.array([(dominant_hz(units.f[n], m[("routine", n)].sum(axis=0)) - dominant[n]) / ABSOLUTE["dominant_hz"] for n in SENSORS])

    lo = [np.log(ROUTINE_GAIN[0])] + [-40.0] * ng + list(np.log([JOLT_HZ[0], JOLT_WIDTH[0]])) + [-20.0, -20.0]
    hi = [np.log(ROUTINE_GAIN[1])] + [20.0] * ng + list(np.log([JOLT_HZ[1], JOLT_WIDTH[1]])) + [10.0, 10.0]
    best = None
    for fr, wr in itertools.product(ROUGH_HZ_GRID, ROUGH_WIDTH_GRID):
        fit_best = None
        for fj0 in (20.0, 50.0):
            p0 = np.concatenate([[0.0], np.full(ng, -4.0), np.log([fj0, 40.0]), [0.0, 0.0]])
            r = scipy.optimize.least_squares(lambda p: band_residuals(model_psd(p, fr, wr)),
                                             np.clip(p0, np.array(lo) + 1e-9, np.array(hi) - 1e-9), bounds=(lo, hi))
            if fit_best is None or r.cost < fit_best.cost:
                fit_best = r
        m = model_psd(fit_best.x, fr, wr)
        cost = float(np.sum(band_residuals(m) ** 2) + np.sum(dominant_residuals(m) ** 2))
        if best is None or cost < best[0]:
            best = (cost, fit_best.x, fr, wr)
    assert best is not None
    cost, p, fr, wr = best

    def assemble(p: np.ndarray) -> tuple[float, dict[str, typing.Any], dict[str, dict[str, float]]]:
        gain = {}
        for xi, group in zip(np.exp(p[1: 1 + ng]), ROUGH_GROUPS):
            for j in group:
                gain[j] = float(np.sqrt(xi)) * REFERENCE_DENSITY_PER_STIFFNESS * oscillator(j, at[family(j)], lit)["stiffness"]
        fj, wj = (float(v) for v in np.exp(p[1 + ng: 3 + ng]))
        drive = {"name": "roughness", "frequency_hz": fr, "bandwidth_hz": wr, "harmonics": [1.0], "gate": "speed", "gain": gain}
        jolts = {n: {"frequency_hz": fj, "bandwidth_hz": wj, "level_dps": float(v)} for n, v in zip(SENSORS, np.exp(p[3 + ng: 5 + ng]))}
        return float(np.exp(p[0])), drive, jolts

    roll = 1 + ROUGH_GROUPS.index(("sway_ry",))
    real_hand = {n: float(np.mean([real_metrics(c)[n]["routine"]["handedness"] for c in caps])) for n in SENSORS}
    choice = None
    for share in ROLL_SHARES:
        lo_s, hi_s = list(lo), list(hi)
        fixed = p[roll] + 2 * np.log(share) if share > 0 else -40.0
        lo_s[roll], hi_s[roll] = fixed - 1e-6, fixed + 1e-6
        p0 = np.clip(p.copy(), np.array(lo_s) + 1e-9, np.array(hi_s) - 1e-9)
        ps = scipy.optimize.least_squares(lambda q: band_residuals(model_psd(q, fr, wr)), p0, bounds=(lo_s, hi_s)).x
        m = model_psd(ps, fr, wr)
        band_cost = float(np.sum(band_residuals(m) ** 2) + np.sum(dominant_residuals(m) ** 2))
        g, drive, jolts = assemble(ps)
        hands = {n: [] for n in SENSORS}
        for i, cap in enumerate(caps):
            reps = measure_reps(cap, mounts)
            text = scenario(hand, Chips.of(cap), params, timeline_moves(reps, g), seed=950 + i, start_support=fitted.table_support,
                            tremor=fitted.tremor, mounts=mounts, drives=fitted.drives + [drive], jolts=jolts)
            sm = real_metrics(simulate(text, work_dir / f"routine-roll-{share:g}-{i}.toml"))
            for n in SENSORS:
                hands[n].append(sm[n]["routine"]["handedness"])
        hand_cost = sum(((np.mean(hands[n]) - real_hand[n]) / HANDEDNESS_TOL) ** 2 for n in SENSORS)
        log(f"roll share {share:g}: band and dominant cost {band_cost:.3g}, handedness "
            + ", ".join(f"{n} {np.mean(hands[n]):.3f} (real {real_hand[n]:.3f})" for n in SENSORS) + f", total {band_cost + hand_cost:.3g}")
        if choice is None or band_cost + hand_cost < choice[0]:
            choice = (band_cost + hand_cost, ps, share)
    assert choice is not None
    cost, p, share = choice
    none_p = np.concatenate([[0.0], np.full(ng, -40.0), p[1 + ng: 3 + ng], [-20.0, -20.0]])
    m0 = model_psd(none_p, fr, wr)
    before = float(np.sum(band_residuals(m0) ** 2) + np.sum(dominant_residuals(m0) ** 2))
    g, drive, jolts = assemble(p)
    gain = drive["gain"]
    fj, wj = jolts[SENSORS[0]]["frequency_hz"], jolts[SENSORS[0]]["bandwidth_hz"]
    m = model_psd(p, fr, wr)
    n_terms = len(band_residuals(m)) + 3 * len(SENSORS)
    report = [f"routine: cost {before:.3g} without roughness, {cost:.3g} with ({n_terms} terms, handedness included); "
              f"routine gain {g:.3g}, roll share {share:g}",
              f"roughness {fr:.2f} Hz width {wr:.2f}, gain per rad/s " + ", ".join(f"{j} {v:.4g}" for j, v in gain.items()),
              f"jolts {fj:.1f} Hz width {wj:.1f}, level dps/sqrt(Hz) per rad/s " + ", ".join(f"{n} {j['level_dps']:.4g}" for n, j in jolts.items()),
              "routine dominant Hz, model (real): " + ", ".join(f"{n} {dominant_hz(units.f[n], m[('routine', n)].sum(axis=0)):.2f} ({dominant[n]:.2f})" for n in SENSORS)]
    for k in fitted_keys:
        d = floored_db(10 * np.log10(band_power_lin(units.f[k[1]], m[k], ROUGH_BANDS))) - floored_db(real[k])
        report.append(f"  {k[0]:5s} {k[1]:6s} model - real dB per axis (x, y, z) over {[f'{a:g}-{c:g}' for a, c in ROUGH_BANDS]}: "
                      + " | ".join(" ".join(f"{v:+.1f}" for v in row) for row in d))
    for line in report:
        log(line)
    return g, drive, jolts, report


def fit(data_dir: pathlib.Path, work_dir: pathlib.Path, *, log: rig.Log = print) -> Fit:
    hand = tomllib.loads(HAND_FILE.read_text())
    params = {n: data_dir / rig.PARAMS[n] for n in SENSORS}
    s1 = [t for t in trials(data_dir) if t.session == 1]
    if not s1:
        raise RuntimeError("no V5 session-1 trials")
    caps = [capture.read_capture(t.path) for t in s1]
    caps = [c for c, t in zip(caps, s1) if check_lift(c, t.path, log)]
    mounts = fit_mounts(caps)
    units = unit_runs(hand, Chips.of(caps[0]), params, mounts, work_dir, log)
    tremor, drives, support, report = fit_tremor(units, fit_targets(caps, mounts, "hold"), fit_targets(caps, mounts, "table"), log,
                                                 central_hz(caps, mounts), table_spectra(caps, mounts))
    support = fit_support(hand, Chips.of(caps[0]), params, mounts, caps, tremor, drives, support, work_dir, log)
    report.append(f"table support on gyro and accel {support:.4g}")
    result = Fit(mounts[0], mounts[1], tremor, support, drives)
    result.contact_pivot, result.finger_contact_pivot = fit_contact(hand, Chips.of(caps[0]), params, mounts, caps, result, work_dir, log)
    report.append(f"contact pivots palm {result.contact_pivot}, finger {result.finger_contact_pivot}")
    at = fitted_scales(tremor)
    gain, rough, jolts, rough_report = fit_routine(hand, params, mounts, caps, units, at, result, work_dir, log)
    result.drives.append(rough)
    result.routine_gain, result.jolts = gain, jolts
    report += rough_report
    (work_dir / "fit.txt").write_text("\n".join(report) + "\n")
    return result


def score(
    data_dir: pathlib.Path,
    work_dir: pathlib.Path,
    fitted: Fit,
    *,
    report_sessions: typing.Callable[[int], bool] = lambda s: s > 1,
    log: rig.Log = print,
) -> list[rig.Score]:
    hand = tomllib.loads(HAND_FILE.read_text())
    params = {n: data_dir / rig.PARAMS[n] for n in SENSORS}
    mounts = (fitted.palm_mount, fitted.finger_mount)
    # (rung, sensor, phase, metric) -> real, simulated
    rows: dict[tuple[str, str, str, str], tuple[list[float], list[float]]] = {}
    for k, t in enumerate(x for x in trials(data_dir) if report_sessions(x.session)):
        cap = capture.read_capture(t.path)
        if not check_lift(cap, t.path, log):
            continue
        real = real_metrics(cap)
        chips = Chips.of(cap)
        reps = measure_reps(cap, mounts)
        for rung in RUNGS:
            realism, twist, still_joints = rung_settings(rung)
            tremor = {j: v | ({"torque_density": 0.0} if j in still_joints else {}) for j, v in fitted.tremor.items()}
            sims = []
            for seed in range(SIM_SEEDS):
                rough = RUNGS.index(rung) >= RUNGS.index("roughness")
                text = scenario(
                    hand, chips, params, timeline_moves(reps, fitted.routine_gain if rough else 1.0), seed=1000 * k + seed + 1,
                    start_support=fitted.table_support, tremor=tremor, mounts=mounts, realism=realism, twist=twist,
                    drives=rung_drives(rung, fitted.drives), jolts=fitted.jolts if rough else None,
                    **(contact_args(fitted) if RUNGS.index(rung) >= RUNGS.index("contact") else {}),
                )
                sims.append(real_metrics(simulate(text, work_dir / f"s{t.session}-{k}-{rung}-{seed}.toml")))
            for n in SENSORS:
                for phase, metrics in real[n].items():
                    for metric, value in metrics.items():
                        rr, ss = rows.setdefault((rung, n, phase, metric), ([], []))
                        rr.append(value)
                        ss.extend(m[n][phase][metric] for m in sims)
        log(f"{t.path.name} scored")
    out = []
    strict: dict[str, int] = {}
    for (rung, n, phase, metric), (real_values, sim_values) in sorted(rows.items()):
        base = metric.split(".")[0]
        absolute = ABSOLUTE.get(base, 0.0)
        v = acceptance.judge(real_values, sim_values, absolute=absolute, relative=RELATIVE.get(base, 0.0))
        if metric == CENTRAL_BAND_METRIC and phase in CENTRAL_BAND_PHASES:
            strict[rung] = strict.get(rung, 0) + bool(v.passed)
            v = acceptance.judge(real_values, sim_values, absolute=CENTRAL_BAND_DB, relative=0.0)
        else:
            strict[rung] = strict.get(rung, 0) + bool(v.passed)
        out.append(rig.Score(n, f"{rung}/{phase}", metric, v.real_mean, v.real_sd, v.sim_mean, v.window, v.passed, len(real_values)))
    for rung in RUNGS:
        part = [s for s in out if s.group.startswith(rung + "/") and s.passed is not None]
        if part:
            log(f"{rung}: {sum(bool(s.passed) for s in part)}/{len(part)} pass ({strict.get(rung, 0)} at 3 dB on the central band)")
    return out


def mass_check(data_dir: pathlib.Path, work_dir: pathlib.Path, fitted: Fit, *, log: rig.Log = print) -> list[str]:
    """Per session with mass-loading captures: the hold's mechanical and central peaks loaded
    against unloaded, real and simulated (the full model, SIM_SEEDS seeds each)."""
    hand = tomllib.loads(HAND_FILE.read_text())
    params = {n: data_dir / rig.PARAMS[n] for n in SENSORS}
    mounts = (fitted.palm_mount, fitted.finger_mount)
    searches = {"mechanical": MECHANICAL_SEARCH_HZ, "central": CENTRAL_SEARCH_HZ}
    lines = []
    loaded = mass_trials(data_dir)
    for session in sorted({t.session for t, _ in loaded}):
        heavy = [(capture.read_capture(t.path), g) for t, g in loaded if t.session == session]
        light = [capture.read_capture(t.path) for t in trials(data_dir) if t.session == session]
        heavy = [(c, g) for c, g in heavy if lift_s(c, SENSORS) is not None]
        if not heavy or not light:
            continue
        grams = float(np.mean([g for _, g in heavy]))
        chips = Chips.of(light[0])
        sim = {}
        for label, kg in (("light", 0.0), ("heavy", grams / 1000.0)):
            sim[label] = [simulate(scenario(hand, chips, params, timeline_moves(), seed=7000 + 10 * session + seed,
                                            start_support=fitted.table_support, tremor=fitted.tremor, mounts=mounts,
                                            drives=fitted.drives, added_kg=kg, **contact_args(fitted)),
                                   work_dir / f"mass-s{session}-{label}-{seed}.toml") for seed in range(SIM_SEEDS)]
        for name, search in searches.items():
            def peaks(cs: list[capture.Capture]) -> np.ndarray:
                return np.array([hold_peak_hz(c, mounts, search, PEAK_SMOOTH_HZ) for c in cs])
            real_l, real_h = peaks(light), peaks([c for c, _ in heavy])
            sim_l, sim_h = peaks(sim["light"]), peaks(sim["heavy"])
            lines.append(f"session {session} +{grams:g} g {name} peak: real {real_l.mean():.2f} -> {real_h.mean():.2f} Hz "
                         f"({real_h.mean() - real_l.mean():+.2f}; unloaded trials "
                         + ", ".join(f"{v:.2f}" for v in real_l)
                         + f"), model {sim_l.mean():.2f} -> {sim_h.mean():.2f} Hz ({sim_h.mean() - sim_l.mean():+.2f})")
    for line in lines:
        log(line)
    return lines


def ladder_csv(results: list[rig.Score]) -> str:
    rows = ["rung,pass,scored"]
    for rung in RUNGS:
        part = [s for s in results if s.group.startswith(rung + "/") and s.passed is not None]
        rows.append(f"{rung},{sum(bool(s.passed) for s in part)},{len(part)}")
    return "\n".join(rows) + "\n"


# --- capture with cues


def cue(argv: list[str]) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Record one V5 trial, printing each phase with a bell.")
    parser.add_argument("--session", type=int, required=True)
    parser.add_argument("--power-up", type=int, required=True)
    parser.add_argument("--notes", default="")
    parser.add_argument("--added-g", type=float, help="mass-loading capture: grams added on the board (table, lift, hold only)")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    args = parser.parse_args(argv)
    seconds, notes = CAPTURE_S, args.notes
    if args.added_g is not None:
        seconds = MASS_CAPTURE_S
        notes = f"{MASS_TAG} +{args.added_g:g} g" + (f"; {notes}" if notes else "")

    recorder = subprocess.Popen(
        [str(pathlib.Path(sys.executable).parent / "timusim-capture"), "--test", "V5", "--config", "default",
         "--session", str(args.session), "--power-up", str(args.power_up), "--seconds", str(seconds),
         "--port", args.port, "--data-dir", str(args.data_dir), "--notes", notes],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    output = recorder.stdout
    assert output is not None
    for line in output:
        if line.startswith("recording"):
            print(line.rstrip(), flush=True)
            break
    started = time.monotonic()
    tail: list[str] = []
    drain = threading.Thread(target=lambda: tail.extend(output), daemon=True)
    drain.start()
    try:
        steps = cues() if args.added_g is None else cues()[:2] + [(PHASES["hold"][1], "hand back on the table")]
        for at, text in steps:
            if at >= seconds:
                break
            while time.monotonic() - started < at and recorder.poll() is None:
                time.sleep(0.05)
            if recorder.poll() is not None:
                break
            print(f"\a{at:6.0f} s  {text}", flush=True)
        recorder.wait()
    except KeyboardInterrupt:
        recorder.wait()
    drain.join(timeout=2)
    print("".join(line for line in tail if not line.lstrip()[:1].isdigit()), end="")
    return recorder.returncode


def run(argv: list[str] | None = None) -> int:
    import argparse

    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["cue"]:
        return cue(argv[1:])
    parser = argparse.ArgumentParser(description=__doc__, epilog="timusim-v5 cue --help: record a trial")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--refit", action="store_true", help="fit session 1 again even if data/v5/fit.toml exists")
    parser.add_argument("--allow-placeholder", action="store_true", help="run with hand.toml's placeholder geometry")
    args = parser.parse_args(argv)

    if not tomllib.loads(HAND_FILE.read_text()).get("measured") and not args.allow_placeholder:
        print(f"{HAND_FILE} still holds placeholders; measure the hand first (RESEARCH.md, V5)")
        return 1
    out_dir = args.data_dir / "v5"
    work_dir = out_dir / "work"
    work_dir.mkdir(parents=True, exist_ok=True)
    fit_path = out_dir / "fit.toml"
    if args.refit or not fit_path.exists():
        fit(args.data_dir, work_dir).write(fit_path)
    results = score(args.data_dir, work_dir, Fit.read(fit_path))
    (out_dir / "scores.csv").write_text(rig.scores_csv(results, "rung_phase"))
    (out_dir / "ladder.csv").write_text(ladder_csv(results))
    print(f"scores in {out_dir / 'scores.csv'}, ladder in {out_dir / 'ladder.csv'}")
    if mass := mass_check(args.data_dir, work_dir, Fit.read(fit_path)):
        (out_dir / "mass.txt").write_text("\n".join(mass) + "\n")
    return 0

"""V1 bench-still metrics, as defined in RESEARCH.md."""

import dataclasses

import numpy as np
import scipy.ndimage
import scipy.signal

from timusim import capture

PSD_SEGMENT_S = 8.0
FLOOR_LOW_HZ = 5.0
SMOOTH_HZ = 2.0
CUTOFF_HOLD_HZ = 5.0
ALLAN_TAUS_S = (0.01, 0.1, 1.0, 10.0, 60.0)
# Where the bias instability rises above the white floor; long segments resolve it.
LOW_FREQ_HZ = (0.05, 0.5)
LOW_FREQ_SEGMENT_S = 100.0


@dataclasses.dataclass(frozen=True)
class AxisMetrics:
    bias: float
    std: float
    noise_floor: float
    low_freq: float
    cutoff_hz: float
    allan_tau_s: tuple[float, ...]
    allan_dev: tuple[float, ...]


@dataclasses.dataclass(frozen=True)
class StillMetrics:
    sensor: str
    chip: str
    odr_hz: float
    duration_s: float
    accel: tuple[AxisMetrics, ...]
    gyro: tuple[AxisMetrics, ...]
    temp_mean_c: float
    temp_drift_c: float


def psd(x: np.ndarray, fs: float, segment_s: float = PSD_SEGMENT_S) -> tuple[np.ndarray, np.ndarray]:
    nperseg = min(len(x), int(round(segment_s * fs)))
    return scipy.signal.welch(x, fs=fs, window="hann", nperseg=nperseg, noverlap=nperseg // 2)


def _first_sustained(mask: np.ndarray, hold: int) -> int | None:
    run = 0
    for i, m in enumerate(mask):
        run = run + 1 if m else 0
        if run >= hold:
            return i - hold + 1
    return None


def noise_floor_and_cutoff(f: np.ndarray, p: np.ndarray, fs: float) -> tuple[float, float]:
    """The floor band ends at half the cutoff and the cutoff is found from the floor, so the
    two are iterated from a cutoff at Nyquist until they agree. The cutoff is where a running
    median of the PSD drops 3 dB below the floor and stays there for CUTOFF_HOLD_HZ, so single
    bins and narrow vibration lines don't count."""
    df = f[1] - f[0]
    smooth = scipy.ndimage.median_filter(p, size=max(1, int(round(SMOOTH_HZ / df))) | 1, mode="nearest")
    hold = max(1, int(round(CUTOFF_HOLD_HZ / df)))
    cutoff = fs / 2
    floor = float("nan")
    for _ in range(10):
        band = (f >= FLOOR_LOW_HZ) & (f <= 0.5 * cutoff)
        if not band.any():
            break
        floor = float(np.median(p[band]))
        start = int(np.searchsorted(f, FLOOR_LOW_HZ))
        i = _first_sustained(smooth[start:] < floor / 2, hold)
        new_cutoff = float(f[start + i]) if i is not None else fs / 2
        if new_cutoff == cutoff:
            break
        cutoff = new_cutoff
    return floor, cutoff


def allan_deviation(
    x: np.ndarray, fs: float, taus_s: tuple[float, ...] = ALLAN_TAUS_S
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Overlapping Allan deviation of a rate signal. Taus longer than a third of the record
    are skipped; returned taus are rounded to whole samples."""
    theta = np.concatenate([[0.0], np.cumsum(x)]) / fs
    n = len(x)
    out_tau, out_dev = [], []
    for tau in taus_s:
        m = max(1, int(round(tau * fs)))
        if 3 * m > n:
            continue
        d = theta[2 * m :] - 2 * theta[m:-m] + theta[: -2 * m]
        t = m / fs
        out_tau.append(t)
        out_dev.append(float(np.sqrt(np.mean(d**2) / (2 * t**2))))
    return tuple(out_tau), tuple(out_dev)


def axis_metrics(x: np.ndarray, fs: float) -> AxisMetrics:
    f, p = psd(x - x.mean(), fs)
    floor, cutoff = noise_floor_and_cutoff(f, p, fs)
    fl, pl = psd(x - x.mean(), fs, LOW_FREQ_SEGMENT_S)
    low = (fl >= LOW_FREQ_HZ[0]) & (fl <= LOW_FREQ_HZ[1])
    taus, devs = allan_deviation(x, fs)
    return AxisMetrics(
        bias=float(x.mean()),
        std=float(x.std(ddof=1)),
        noise_floor=floor,
        low_freq=float(np.median(pl[low])),
        cutoff_hz=cutoff,
        allan_tau_s=taus,
        allan_dev=devs,
    )


def array_metrics(accel: np.ndarray, gyro: np.ndarray, fs: float) -> StillMetrics:
    """The same metrics on samples from elsewhere, without a temperature."""
    return StillMetrics(
        sensor="",
        chip="",
        odr_hz=fs,
        duration_s=(len(accel) - 1) / fs,
        accel=tuple(axis_metrics(accel[:, i], fs) for i in range(3)),
        gyro=tuple(axis_metrics(gyro[:, i], fs) for i in range(3)),
        temp_mean_c=float("nan"),
        temp_drift_c=float("nan"),
    )


def still_metrics(cap: capture.Capture, sensor_id: int, *, trim_s: float = 0.0) -> StillMetrics:
    s = cap.sensors[sensor_id]
    keep = s.longest_run(s.t_s >= trim_s)
    accel, gyro = s.accel_g[keep], s.gyro_dps[keep]

    temp_mean = temp_drift = float("nan")
    if sensor_id in cap.temps:
        t, c = cap.temps[sensor_id]
        if len(c) >= 2:
            temp_mean = float(c.mean())
            temp_drift = float(np.polyfit(t, c, 1)[0] * (t[-1] - t[0]))

    return StillMetrics(
        sensor=s.name,
        chip=s.chip,
        odr_hz=s.odr_hz,
        duration_s=float(s.t_s[keep][-1] - s.t_s[keep][0]),
        accel=tuple(axis_metrics(accel[:, i], s.odr_hz) for i in range(3)),
        gyro=tuple(axis_metrics(gyro[:, i], s.odr_hz) for i in range(3)),
        temp_mean_c=temp_mean,
        temp_drift_c=temp_drift,
    )

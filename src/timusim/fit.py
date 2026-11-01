"""Fits chip parameters (lib/chip/include/chip/model.hpp Params) to V1 bench captures of an
LSM9DS1 or LSM6DSL.

The PSD model mirrors the C++ signal path: white noise at oversample x ODR through the
internal-rate filter, folded by the decimation, then the ODR-rate filter. The Allan model is
white noise + first-order Gauss-Markov drift + rate random walk.
"""

import dataclasses
import itertools
import pathlib
import typing

import numpy as np
import scipy.optimize
import scipy.signal

from timusim import bench, capture

OVERSAMPLE = 8
MAX_ORDER = 4
MAX_TAPS = 16
FIT_LOW_HZ = 2.0
# Drift Markov terms half a decade apart; Allan points below DRIFT_MIN_TAU_S still carry the filters.
DRIFT_TAUS_S = tuple(float(t) for t in np.logspace(-0.5, 3.5, 9))
DRIFT_MIN_TAU_S = 0.1
# Spectrum points the drift is fitted to as well: below the filters, from 100 s segments.
DRIFT_PSD_HZ = (0.02, 2.0)
DRIFT_REFITS = 3
# Log10 PSD residual where the fit loss turns from squared to linear: 0.1 is 1 dB.
ROBUST_SCALE = 0.1
# Factors on the datasheet internal gyro cutoff searched where that stage alone shapes the gyro band.
LPF1_SCALES = tuple(float(k) for k in np.exp(np.linspace(np.log(0.5), np.log(2.0), 57)))

# Datasheet cutoffs as in chip/lsm9ds1.hpp.
ACCEL_AA_HZ = {952: 408.0, 476: 211.0, 238: 105.0, 119: 50.0}
GYRO_LPF1_HZ = {952: 100.0, 476: 100.0, 238: 76.0, 119: 38.0}
GYRO_LPF2_HZ = {
    119: (14.0, 31.0, 31.0, 31.0),
    238: (14.0, 29.0, 63.0, 78.0),
    476: (21.0, 28.0, 57.0, 100.0),
    952: (33.0, 40.0, 58.0, 100.0),
}
ACCEL_LP_DIVIDER = {0x10: 50.0, 0x11: 100.0, 0x12: 9.0, 0x13: 400.0}
LPF1_LPF2_OUT = 0x02

# As in chip/lsm6dsl.hpp: AN5040 Tables 8, 9 and 11, internal rate at the 6.66 kHz top ODR.
LSM6DSL_INTERNAL_HZ = 6664.0
LSM6DSL_ACCEL_ANALOG_HZ = 400.0
# Overall gyro cutoff for LPF1 off, then FTYPE 00, 01, 10, 11.
LSM6DSL_GYRO_HZ = {
    104: (33.0, 33.0, 33.0, 33.0, 33.0),
    208: (67.0, 67.0, 67.0, 67.0, 67.0),
    416: (137.0, 138.0, 131.0, 121.0, 138.0),
    833: (312.0, 245.0, 195.0, 155.0, 293.0),
}
LSM6DSL_LP2_DIVIDER = (50.0, 100.0, 9.0, 400.0)


@dataclasses.dataclass(frozen=True)
class Path:
    """One sensor's filters for a config: (cutoff Hz or 0) at the internal rate and at ODR."""

    internal_hz: float
    odr_stage_hz: float
    oversample: int = OVERSAMPLE


def paths(header: dict[str, str], odr_scale: float, lpf1_scale: float = 1.0) -> dict[str, Path]:
    if header.get("chip") == "LSM6DSL":
        return _lsm6dsl_paths(header, odr_scale, lpf1_scale)
    nominal = int(round(float(header["odr_hz"])))
    divider = ACCEL_LP_DIVIDER.get(int(header["accel_bw"]), 0.0)
    odr = nominal * odr_scale
    lpf2 = int(header["gyro_out_path"]) == LPF1_LPF2_OUT
    return {
        "accel": Path(ACCEL_AA_HZ[nominal], odr / divider if divider else 0.0),
        "gyro": Path(
            GYRO_LPF1_HZ[nominal] * odr_scale * lpf1_scale,
            GYRO_LPF2_HZ[nominal][int(header["gyro_bw"]) & 3] * odr_scale if lpf2 else 0.0,
        ),
    }


def _lsm6dsl_paths(header: dict[str, str], odr_scale: float, lpf1_scale: float) -> dict[str, Path]:
    nominal = int(round(float(header["odr_hz"])))
    accel_bw, gyro_bw = int(header["accel_bw"]), int(header["gyro_bw"])
    odr = nominal * odr_scale
    oversample = int(round(LSM6DSL_INTERNAL_HZ / nominal))
    lpf1 = nominal / (4.0 if accel_bw & 0x10 else 2.0)
    gyro = LSM6DSL_GYRO_HZ[nominal][1 + (gyro_bw & 3) if gyro_bw & 0x08 else 0]
    return {
        "accel": Path(min(LSM6DSL_ACCEL_ANALOG_HZ, lpf1), odr / LSM6DSL_LP2_DIVIDER[accel_bw & 3], oversample),
        "gyro": Path(gyro * odr_scale * lpf1_scale, 0.0, oversample),
    }


@dataclasses.dataclass(frozen=True)
class Shape:
    """chip::FilterShape."""

    kind: str = "butterworth"  # or "one_pole", "moving_average" (order = taps)
    order: int = 0

    def toml(self) -> str:
        return f'{{ kind = "{self.kind}", order = {self.order} }}'


def shapes(available: bool, moving_average: bool = False) -> list[Shape]:
    if not available:
        return [Shape()]
    out = [Shape()] + [Shape(k, n) for k in ("butterworth", "one_pole") for n in range(1, MAX_ORDER + 1)]
    if moving_average:
        out += [Shape("moving_average", n) for n in range(2, MAX_TAPS + 1)]
    return out


def one_pole_coefficient(order: int, cutoff_hz: float, fs: float) -> float:
    """As chip/filter.hpp: the cascade is -3 dB at the cutoff."""
    g = 0.5 ** (1 / order)
    m = 1 - np.cos(2 * np.pi * cutoff_hz / fs)
    return float((-2 * g * m + np.sqrt(4 * g * g * m * m + 8 * (1 - g) * g * m)) / (2 * (1 - g)))


def _power_response(shape: Shape, cutoff_hz: float, f: np.ndarray, fs: float) -> np.ndarray:
    if shape.order == 0:
        return np.ones_like(f)
    if shape.kind == "moving_average":
        n = shape.order
        x = np.pi * f / fs
        with np.errstate(invalid="ignore", divide="ignore"):
            h = np.where(np.abs(np.sin(x)) < 1e-12, 1.0, np.sin(n * x) / (n * np.sin(x)))
        return h**2
    if cutoff_hz <= 0.0:
        return np.ones_like(f)
    if shape.kind == "one_pole":
        a = one_pole_coefficient(shape.order, cutoff_hz, fs)
        z1 = np.exp(-2j * np.pi * f / fs)
        return np.abs(a / (1 - (1 - a) * z1)) ** (2 * shape.order)
    sos = scipy.signal.butter(shape.order, cutoff_hz, fs=fs, output="sos")
    _, h = scipy.signal.sosfreqz(sos, worN=f, fs=fs)
    return np.abs(h) ** 2


def quantization_psd(lsb: float, odr: float) -> float:
    return lsb**2 / 12 / (odr / 2)


def model_psd(
    f: np.ndarray, odr: float, path: Path, internal: Shape, odr_stage: Shape, density: float = 1.0
) -> np.ndarray:
    """One-sided output PSD at f (0..ODR/2) for white noise of `density` per sqrt(Hz), before
    quantization."""
    fs = odr * path.oversample
    folded = np.zeros_like(f)
    for m in range(path.oversample):
        for g in (m * odr + f, (m + 1) * odr - f):
            keep = g <= fs / 2
            folded[keep] += _power_response(internal, path.internal_hz, g[keep], fs)
    return density**2 * folded * _power_response(odr_stage, path.odr_stage_hz, f, odr)


@dataclasses.dataclass(frozen=True)
class ShapeFit:
    density: float
    internal: Shape
    odr_stage: Shape
    rms_db: float
    internal_scale: float = 1.0


def fit_shape(
    f: np.ndarray,
    p: np.ndarray,
    odr: float,
    path: Path,
    lsb: float,
    *,
    internal_scales: typing.Sequence[float] = (1.0,),
    internal: Shape | None = None,
) -> ShapeFit:
    """Orders by grid search, density by a 1-D fit on log PSD for each pair. The quantization
    floor is known from the LSB and added to the model, not fitted. A soft-L1 loss on the log
    residual keeps narrow vibration lines from pulling the fit. internal_scales multiplies the
    internal-rate cutoff; internal fixes that stage's shape."""
    band = (f >= FIT_LOW_HZ) & (f < 0.98 * odr / 2)
    f, p = f[band], p[band]
    q = quantization_psd(lsb, odr)
    best: ShapeFit | None = None
    internals = [internal] if internal is not None else shapes(path.internal_hz > 0)
    # A moving average can only be the ODR-rate stage: its zeros sit at multiples of ODR / n.
    for k, ni, no in itertools.product(internal_scales, internals, shapes(path.odr_stage_hz > 0, moving_average=True)):
        shape = model_psd(f, odr, dataclasses.replace(path, internal_hz=path.internal_hz * k), ni, no)

        def err(log_d: float, shape: np.ndarray = shape) -> float:
            r = (np.log10(p) - np.log10(10 ** (2 * log_d) * shape + q)) / ROBUST_SCALE
            return float(np.mean(2 * (np.sqrt(1 + r**2) - 1)) * ROBUST_SCALE**2)

        guess = 0.5 * np.mean(np.log10(p[: max(1, len(p) // 4)]) - np.log10(shape[: max(1, len(p) // 4)]))
        res = scipy.optimize.minimize_scalar(err, bracket=(guess - 1, guess + 1))
        rms_db = float(10 * np.sqrt(res.fun))
        if best is None or rms_db < best.rms_db:
            best = ShapeFit(float(10**res.x), ni, no, rms_db, k)
    assert best is not None
    return best


def gauss_markov_avar(tau: np.ndarray, sigma: float, t_c: float) -> np.ndarray:
    """Allan variance of a first-order Gauss-Markov process with steady-state std sigma."""
    r = tau / t_c
    return (sigma**2 / r**2) * (2 * r - 3 + 4 * np.exp(-r) - np.exp(-2 * r))


def gauss_markov_psd(f: np.ndarray, sigma: float, t_c: float) -> np.ndarray:
    """One-sided PSD of a first-order Gauss-Markov process with steady-state std sigma."""
    return 4 * sigma**2 * t_c / (1 + (2 * np.pi * f * t_c) ** 2)


def random_walk_psd(f: np.ndarray, k: float) -> np.ndarray:
    """One-sided PSD of a random walk whose increments have variance k^2 dt."""
    return k**2 / (2 * np.pi**2 * f**2)


def drift_spectrum(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    f, p = bench.psd(x - x.mean(), fs, bench.LOW_FREQ_SEGMENT_S)
    keep = (f >= DRIFT_PSD_HZ[0]) & (f <= DRIFT_PSD_HZ[1])
    return f[keep], p[keep]


def allan_curve(x: np.ndarray, fs: float, points: int = 40) -> tuple[np.ndarray, np.ndarray]:
    taus = tuple(np.unique(np.round(np.logspace(np.log10(2 / fs), np.log10(len(x) / fs / 5), points) * fs)) / fs)
    return tuple(np.array(v) for v in bench.allan_deviation(x, fs, taus))  # type: ignore[return-value]


def white_avar(density: float, tau: np.ndarray) -> np.ndarray:
    """Allan variance of white noise with a one-sided density."""
    return density**2 / (2 * tau)


@dataclasses.dataclass(frozen=True)
class Drift:
    markov: tuple[tuple[float, float], ...]  # (steady-state std, correlation time s)
    rate_random_walk: float  # unit/sqrt(s)

    def avar(self, tau: np.ndarray) -> np.ndarray:
        out = self.rate_random_walk**2 * tau / 3
        for sigma, t_c in self.markov:
            out = out + gauss_markov_avar(tau, sigma, t_c)
        return out

    def psd(self, f: np.ndarray) -> np.ndarray:
        out = random_walk_psd(f, self.rate_random_walk)
        for sigma, t_c in self.markov:
            out = out + gauss_markov_psd(f, sigma, t_c)
        return out


def _drift_points(curves: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
    """Taus and Allan variances from every curve where filtering no longer shapes it."""
    tau = np.concatenate([t[t >= DRIFT_MIN_TAU_S] for t, _ in curves])
    avar = np.concatenate([a[t >= DRIFT_MIN_TAU_S] ** 2 for t, a in curves])
    return tau, avar


def fit_drift(
    curves: list[tuple[np.ndarray, np.ndarray]],
    density: float,
    spectra: list[tuple[np.ndarray, np.ndarray]] | None = None,
) -> Drift:
    """Bias instability and random walk from the Allan curves of one axis, all captures at once,
    and their low-frequency spectra when given: the Allan variance alone cannot tell a short
    Markov term from white noise. White noise is held at density, the output level below the
    filters; the Markov terms sit on DRIFT_TAUS_S up to the longest tau measured. Both are linear
    in each term's variance, so non-negative least squares on the relative error finds the global
    fit, with the Allan and spectrum points weighted equally as two blocks: a flat floor comes
    out as several terms.
    The error is relative to the model, refitted a few times from the measured values: relative
    to a noisy measurement it pulls the fit toward the points that came out low."""
    tau, avar = _drift_points(curves)
    taus_c = [t for t in DRIFT_TAUS_S if t <= tau.max()]
    blocks = [(np.column_stack([gauss_markov_avar(tau, 1.0, t_c) for t_c in taus_c] + [tau / 3]), avar, white_avar(density, tau))]
    if spectra:
        f = np.concatenate([fr for fr, _ in spectra])
        psd = np.concatenate([p for _, p in spectra])
        spec = np.column_stack([gauss_markov_psd(f, 1.0, t_c) for t_c in taus_c] + [random_walk_psd(f, 1.0)])
        blocks.append((spec, psd, np.full(len(f), density**2)))
    scales = [measured for _, measured, _ in blocks]
    for _ in range(DRIFT_REFITS + 1):
        rows = [design / scale[:, None] / np.sqrt(len(scale)) for (design, _, _), scale in zip(blocks, scales)]
        target = [(measured - white) / scale / np.sqrt(len(scale)) for (_, measured, white), scale in zip(blocks, scales)]
        x, _ = scipy.optimize.nnls(np.vstack(rows), np.concatenate(target))
        scales = [design @ x + white for design, _, white in blocks]
    markov = tuple((float(np.sqrt(v)), float(t_c)) for v, t_c in zip(x[:-1], taus_c) if v > 0)
    return Drift(markov, float(np.sqrt(x[-1])))


def fit_drift_scale(curves: list[tuple[np.ndarray, np.ndarray]], density: float, drift: Drift) -> float:
    """The factor on a drift model's level that fits other captures, as at another full scale."""
    tau, avar = _drift_points(curves)
    base = drift.avar(tau)

    def err(log_k: float) -> float:
        model = white_avar(density, tau) + np.exp(2 * log_k) * base
        return float(np.mean(np.log(model / avar) ** 2))

    return float(np.exp(scipy.optimize.minimize_scalar(err, bounds=(-5.0, 2.0), method="bounded").x))


@dataclasses.dataclass(frozen=True)
class AxisFit:
    shape: ShapeFit
    allan: tuple[np.ndarray, np.ndarray] | None  # Allan curve: tau s, deviation
    mean: float
    spectrum: tuple[np.ndarray, np.ndarray] | None = None  # low-frequency PSD: Hz, unit^2/Hz
    # Output white density below the filters: the aliased internal band and quantization raise it
    # above shape.density, and the drift is what is left above it.
    white_low: float = 0.0


@dataclasses.dataclass(frozen=True)
class SensorFit:
    sensor: str
    chip: str
    config: str
    odr_nominal: int
    accel_fs_g: float
    gyro_fs_dps: float
    odr_scale: float
    temperature_c: float
    accel: tuple[AxisFit, ...]
    gyro: tuple[AxisFit, ...]


def fit_capture(
    cap: capture.Capture,
    sensor_id: int,
    *,
    trim_s: float = 60.0,
    allan: bool = True,
    gyro_lpf1: dict[int, tuple[tuple[Shape, float], ...]] | None = None,
) -> SensorFit:
    """Where LPF1 alone shapes the gyro band its cutoff factor is fitted; behind LPF2, LPF1 takes
    its shape and factor at this ODR and axis from gyro_lpf1 when given."""
    s = cap.sensors[sensor_id]
    nominal = float(s.header["odr_hz"])
    odr_scale = s.odr_hz / nominal
    p = paths(s.header, odr_scale)
    known = (gyro_lpf1 or {}).get(int(round(nominal)))
    gyro_kwargs: list[dict[str, typing.Any]] = [{}, {}, {}]
    if p["gyro"].odr_stage_hz == 0:
        gyro_kwargs = [{"internal_scales": LPF1_SCALES}] * 3
    elif known is not None:
        gyro_kwargs = [{"internal_scales": (scale,), "internal": shape} for shape, scale in known]
    keep = s.longest_run(s.t_s >= trim_s)
    out: dict[str, list[AxisFit]] = {"accel": [], "gyro": []}
    lsbs = {"accel": float(s.header["accel_lsb_g"]), "gyro": float(s.header["gyro_lsb_dps"])}
    for kind, data in (("accel", s.accel_g[keep]), ("gyro", s.gyro_dps[keep])):
        for i in range(3):
            x = data[:, i]
            f, psd = bench.psd(x - x.mean(), s.odr_hz)
            shape = fit_shape(f, psd, s.odr_hz, p[kind], lsbs[kind], **(gyro_kwargs[i] if kind == "gyro" else {}))
            curve = allan_curve(x, s.odr_hz) if allan else None
            spectrum = drift_spectrum(x, s.odr_hz) if allan else None
            path = dataclasses.replace(p[kind], internal_hz=p[kind].internal_hz * shape.internal_scale)
            dc = model_psd(np.zeros(1), s.odr_hz, path, shape.internal, shape.odr_stage, shape.density)[0]
            white_low = float(np.sqrt(dc + quantization_psd(lsbs[kind], s.odr_hz)))
            out[kind].append(AxisFit(shape, curve, float(x.mean()), spectrum, white_low))
    temps = cap.temps.get(sensor_id)
    return SensorFit(
        sensor=s.name,
        chip=s.chip,
        config=cap.header.get("config", "?"),
        odr_nominal=int(round(nominal)),
        accel_fs_g=float(s.header["accel_fs_g"]),
        gyro_fs_dps=float(s.header["gyro_fs_dps"]),
        odr_scale=odr_scale,
        temperature_c=float(temps[1].mean()) if temps is not None and len(temps[1]) else float("nan"),
        accel=tuple(out["accel"]),
        gyro=tuple(out["gyro"]),
    )


def vector(values: typing.Iterable[float]) -> str:
    return "[" + ", ".join(f"{v:.6g}" for v in values) + "]"


def vote(shapes_: typing.Iterable[Shape]) -> Shape:
    """The shape most axes and captures agree on."""
    counts: dict[Shape, int] = {}
    for sh in shapes_:
        counts[sh] = counts.get(sh, 0) + 1
    return max(counts, key=lambda k: counts[k])


def lpf1_by_odr(fits: typing.Iterable[SensorFit]) -> dict[int, tuple[tuple[Shape, float], ...]]:
    """LPF1 shape per nominal ODR and gyro axis, voted over the LPF1-only fits, with the median
    cutoff factor of the captures that took that shape."""
    groups: dict[int, tuple[list[ShapeFit], ...]] = {}
    for f in fits:
        if all(a.shape.odr_stage.order == 0 for a in f.gyro):
            axes = groups.setdefault(f.odr_nominal, ([], [], []))
            for axis, a in zip(axes, f.gyro):
                axis.append(a.shape)
    out = {}
    for odr, axes in groups.items():
        per_axis = []
        for group in axes:
            shape = vote(g.internal for g in group)
            per_axis.append((shape, float(np.median([g.internal_scale for g in group if g.internal == shape]))))
        out[odr] = tuple(per_axis)
    return out


def params_toml(
    fits: list[SensorFit],
    *,
    source: str,
    gyro_lpf2_by_bw: dict[int, Shape] | None = None,
    by_range: list[SensorFit] | None = None,
    shape_fits: list[SensorFit] | None = None,
    allan_fits: list[SensorFit] | None = None,
    accel_allan_fits: list[SensorFit] | None = None,
    power_ups: list[list[SensorFit]] | None = None,
) -> str:
    """Chip parameter file for timusim-sim from one or more captures of the same chip. Densities
    are medians over the captures. power_ups groups captures by board power-up: with two or more,
    the gyro zero level is the mean over power-ups and the turn-on sigma their spread; otherwise
    the zero level is the median gyro mean and the turn-on sigma zero. The drift is fitted to the
    Allan curves and low-frequency spectra of allan_fits (default: fits) together; the accel's
    of accel_allan_fits when given (holds where the board tilted). Captures in by_range at other full
    scales give that range's own white noise density, gyro zero level and drift level. Filter
    shapes are voted over shape_fits (default: fits) across axes and captures."""
    voters = shape_fits or fits
    drift_fits = allan_fits or fits

    def med(get: typing.Callable[[SensorFit, int], float]) -> list[float]:
        return [float(np.median([get(f, i) for f in fits])) for i in range(3)]

    def by_power_up() -> np.ndarray | None:
        groups = [g for g in power_ups or [] if g]
        if len(groups) < 2:
            return None
        return np.array([[np.mean([f.gyro[i].mean for f in g]) for i in range(3)] for g in groups])

    def zero_level() -> list[float]:
        levels = by_power_up()
        return med(lambda f, i: f.gyro[i].mean) if levels is None else [float(v) for v in levels.mean(axis=0)]

    def turn_on() -> list[float]:
        levels = by_power_up()
        return [0.0, 0.0, 0.0] if levels is None else [float(v) for v in levels.std(axis=0, ddof=1)]

    lines = [
        f"# Fitted by timusim.fit from {source}.",
        f"odr_scale = {np.median([f.odr_scale for f in fits]):.6f}",
        *([f"oversample = {OVERSAMPLE}"] if fits[0].chip != "LSM6DSL" else []),
        f"accel_aa = {vote(a.shape.internal for f in voters for a in f.accel).toml()}",
        f"accel_lp = {vote(a.shape.odr_stage for f in voters for a in f.accel).toml()}",
        f"gyro_lpf1 = {vote(a.shape.internal for f in voters for a in f.gyro).toml()}",
        f"gyro_lpf2 = {vote(a.shape.odr_stage for f in voters for a in f.gyro).toml()}",
        f"temperature_c = {np.median([f.temperature_c for f in fits]):.2f}",
        f"gyro_offset = {vector(zero_level())}",
    ]
    lpf1 = lpf1_by_odr(voters)
    if lpf1:
        def scaled(sh: Shape, k: float) -> str:
            return f'{{ kind = "{sh.kind}", order = {sh.order}, cutoff_scale = {k:.4f} }}'

        lines += ["", "[gyro_lpf1_by_odr]"] + [
            f'"{odr}" = ' + (scaled(*axes[0]) if len(set(axes)) == 1 else "[" + ", ".join(scaled(*a) for a in axes) + "]")
            for odr, axes in sorted(lpf1.items())
        ]
    if gyro_lpf2_by_bw:
        lines += ["", "[gyro_lpf2_by_bw]"] + [f'"{bw}" = {sh.toml()}' for bw, sh in sorted(gyro_lpf2_by_bw.items())]
    for kind in ("accel", "gyro"):
        def axes(f: SensorFit, kind: str = kind) -> tuple[AxisFit, ...]:
            return f.accel if kind == "accel" else f.gyro

        lines += ["", f"[{kind}]"]
        lines.append(f"white_density = {vector(med(lambda f, i: axes(f)[i].shape.density))}")
        if kind == "gyro":
            lines.append(f"turn_on_sigma = {vector(turn_on())}")
        white = med(lambda f, i: axes(f)[i].white_low)
        drifts: list[Drift] = []
        held = accel_allan_fits if kind == "accel" and accel_allan_fits else drift_fits
        if all(a.allan is not None for f in held for a in axes(f)):
            drifts = [
                fit_drift(
                    [axes(f)[i].allan for f in held],  # type: ignore[misc]
                    white[i],
                    [axes(f)[i].spectrum for f in held if axes(f)[i].spectrum is not None],  # type: ignore[misc]
                )
                for i in range(3)
            ]
            lines.append("markov = [")
            for t_c in DRIFT_TAUS_S:
                sigma = [next((sg for sg, t in d.markov if t == t_c), 0.0) for d in drifts]
                if any(sigma):
                    lines.append(f"    {{ sigma = {vector(sigma)}, tau_s = {vector([t_c] * 3)} }},")
            lines.append("]")
            lines.append(f"rate_random_walk = {vector(d.rate_random_walk for d in drifts)}")
        others: dict[float, list[SensorFit]] = {}
        for f in by_range or []:
            fs = f.accel_fs_g if kind == "accel" else f.gyro_fs_dps
            if fs != (fits[0].accel_fs_g if kind == "accel" else fits[0].gyro_fs_dps):
                others.setdefault(fs, []).append(f)
        if others:
            lines += ["", f"[{kind}.by_fs]"]
            for fs, group in sorted(others.items()):
                d = [float(np.median([axes(f)[i].shape.density for f in group])) for i in range(3)]
                lines.append(f'"{fs:g}" = {vector(d)}')
            if kind == "gyro":
                lines += ["", "[gyro.offset_by_fs]"]
                for fs, group in sorted(others.items()):
                    lines.append(f'"{fs:g}" = {vector(float(np.median([f.gyro[i].mean for f in group])) for i in range(3))}')
            curved = {fs: g for fs, g in others.items() if all(axes(f)[i].allan is not None for f in g for i in range(3))}
            if drifts and curved:
                lines += ["", f"[{kind}.drift_scale_by_fs]"]
                for fs, group in sorted(curved.items()):
                    k = [
                        fit_drift_scale(
                            [axes(f)[i].allan for f in group],  # type: ignore[misc]
                            float(np.median([axes(f)[i].white_low for f in group])),
                            drifts[i],
                        )
                        for i in range(3)
                    ]
                    lines.append(f'"{fs:g}" = {vector(k)}')
    return "\n".join(lines) + "\n"


def with_top_level(params: str, lines: typing.Sequence[str]) -> str:
    """The parameter file with these top-level `key = value` lines, replacing any with the same
    key; they go before the first table."""
    keys = tuple(ln.split("=", 1)[0].strip() + " " for ln in lines)
    kept = [ln for ln in params.splitlines() if not (ln.split("=", 1)[0].strip() + " ").startswith(keys)]
    first_table = next((i for i, ln in enumerate(kept) if ln.startswith("[")), len(kept))
    kept[first_table:first_table] = list(lines)
    return "\n".join(kept) + "\n"


def matrix(rows: np.ndarray) -> str:
    return "[" + ", ".join(vector(row) for row in rows) + "]"


def fit_files(paths_: list[pathlib.Path], sensor: str, **kwargs: typing.Any) -> list[SensorFit]:
    fits = []
    for path in paths_:
        cap = capture.read_capture(path)
        ids = [i for i, s in cap.sensors.items() if s.name == sensor]
        if ids:
            fits.append(fit_capture(cap, ids[0], **kwargs))
    return fits

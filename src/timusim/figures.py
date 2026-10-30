"""Article figures. Each function returns a matplotlib Figure; the figures CLI saves them."""

import dataclasses
import itertools
import typing

import matplotlib
import matplotlib.patches

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from timusim import bench, capture, fit, v2  # noqa: E402

# Categorical slots 1-3 of the reference palette (validated light; aqua is below 3:1 contrast,
# so every series also has its own line style and a legend).
REAL = "#2a78d6"
SIM = "#eb6834"
MODEL = "#1baf7a"
INK = "#52514e"
# The other simulators take slots 3 to 6, after real and timusim.
OTHER = {
    "imusim": ("#1baf7a", "-."),
    "wimusim": ("#eda100", ":"),
    "gnss-ins-sim": ("#e87ba4", (0, (5, 1, 1, 1, 1, 1))),
    "matlab": ("#008300", (0, (1, 1))),
}
MARKERS = {"timusim": "o", "imusim": "s", "wimusim": "^", "gnss-ins-sim": "D", "matlab": "v"}
BAND = "#e9e8e4"

AXES = "xyz"
UNITS = {"accel": ("g", "mg", 1e3), "gyro": ("dps", "mdps", 1e3)}
Curve = typing.Callable[[np.ndarray, float], tuple[np.ndarray, np.ndarray]]


def _style(ax: plt.Axes) -> None:
    ax.grid(True, which="major", color="#e4e3df", linewidth=0.6)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(INK)
    ax.tick_params(colors=INK, labelsize=8)


def _sensor(cap: capture.Capture, name: str) -> capture.SensorStream:
    return next(s for s in cap.sensors.values() if s.name == name)


def _signal(s: capture.SensorStream, kind: str, trim_s: float) -> tuple[np.ndarray, float]:
    keep = s.longest_run(s.t_s >= trim_s)
    data = s.accel_g if kind == "accel" else s.gyro_dps
    return data[keep], s.odr_hz


# A line on a 2x3 grid: label (None for repeats), colour, line style, accel and gyro samples, rate.
Series = tuple[str | None, str, str, np.ndarray, np.ndarray, float]


def series(cap: capture.Capture, label: str | None, color: str, style: str, *, sensor: str, trim_s: float) -> Series:
    s = _sensor(cap, sensor)
    accel, fs = _signal(s, "accel", trim_s)
    gyro, _ = _signal(s, "gyro", trim_s)
    return label, color, style, accel, gyro, fs


def _psd_curve(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    f, p = bench.psd(x - x.mean(), fs)
    return f[1:], np.sqrt(p[1:])


def _grid(lines: typing.Iterable[Series], curve: Curve, ylabel: str, xlabel: str) -> tuple[Figure, np.ndarray]:
    fig, axes = plt.subplots(2, 3, figsize=(9, 5), sharex=True, layout="constrained")
    for label, color, style, accel, gyro, fs in lines:
        for row, data in enumerate((accel, gyro)):
            for col in range(3):
                xs, ys = curve(data[:, col], fs)
                axes[row, col].loglog(xs, ys * 1e3, color=color, linestyle=style, linewidth=1.1, label=label)
    for row, kind in enumerate(("accel", "gyro")):
        for col in range(3):
            ax = axes[row, col]
            ax.set_title(f"{kind} {AXES[col]}", fontsize=9, color=INK)
            if col == 0:
                ax.set_ylabel(ylabel.format(UNITS[kind][1]), fontsize=8, color=INK)
            if row == 1:
                ax.set_xlabel(xlabel, fontsize=8, color=INK)
            _style(ax)
    return fig, axes


def psd(real: capture.Capture, sim: capture.Capture | None = None, *, sensor: str = "finger", trim_s: float = 60.0) -> Figure:
    """Noise PSD per axis, accel and gyro, real against simulated."""
    lines = [series(real, "real", REAL, "-", sensor=sensor, trim_s=trim_s)]
    if sim is not None:
        lines.append(series(sim, "sim", SIM, "--", sensor=sensor, trim_s=trim_s))
    return psd_lines(lines)


def psd_lines(lines: list[Series]) -> Figure:
    fig, axes = _grid(lines, _psd_curve, "{}/√Hz", "Hz")
    axes[0, 0].legend(fontsize=8, frameon=False)
    return fig


def allan(real: list[capture.Capture], sim: list[capture.Capture], *, sensor: str = "finger", trim_s: float = 60.0) -> Figure:
    """Allan deviation per axis; one line per capture, real and simulated."""
    lines = [series(c, "real" if i == 0 else None, REAL, "-", sensor=sensor, trim_s=trim_s) for i, c in enumerate(real)]
    lines += [series(c, "sim" if i == 0 else None, SIM, "--", sensor=sensor, trim_s=trim_s) for i, c in enumerate(sim)]
    return allan_lines(lines)


def allan_lines(lines: list[Series]) -> Figure:
    fig, axes = _grid(lines, fit.allan_curve, "Allan deviation, {}", "τ, s")
    axes[0, 0].legend(fontsize=8, frameon=False)
    return fig


def allan_model(
    real: typing.Iterable[capture.Capture],
    params: dict,
    *,
    white: dict[str, list[float]] | None = None,
    sensor: str = "finger",
    trim_s: float = 60.0,
) -> Figure:
    """Allan deviation of long holds against the fitted noise model: white noise plus the Markov
    drift terms and random walk. white is the output level below the filters, as the drift was
    fitted on; the params density, at the filter input, when not given. Filters are left out of
    the model, so it starts where they no longer shape the curve. The holds are read one at a
    time, so each can be dropped before the next."""
    curves: list[dict[tuple[str, int], tuple[np.ndarray, np.ndarray]]] = []
    for cap in real:
        _, _, _, accel, gyro, fs = series(cap, None, REAL, "-", sensor=sensor, trim_s=trim_s)
        curves.append({(kind, col): fit.allan_curve(data[:, col], fs) for kind, data in (("accel", accel), ("gyro", gyro)) for col in range(3)})
        del cap, accel, gyro
    fig, axes = plt.subplots(2, 3, figsize=(9, 5), sharex=True, layout="constrained")
    for row, kind in enumerate(("accel", "gyro")):
        _, small, k = UNITS[kind]
        noise = params[kind]
        level = (white or {}).get(kind, noise["white_density"])
        for col in range(3):
            ax = axes[row, col]
            for i, c in enumerate(curves):
                tau, dev = c[(kind, col)]
                ax.loglog(tau, dev * k, color=REAL, linewidth=1.0, label="real" if i == 0 else None)
            tau = np.logspace(np.log10(fit.DRIFT_MIN_TAU_S), np.log10(max(c[(kind, col)][0].max() for c in curves)), 60)
            drift = fit.Drift(
                tuple((term["sigma"][col], term["tau_s"][col]) for term in noise.get("markov", [])),
                noise.get("rate_random_walk", [0.0] * 3)[col],
            )
            model = np.sqrt(fit.white_avar(level[col], tau) + drift.avar(tau))
            ax.loglog(tau, model * k, color=MODEL, linestyle="--", linewidth=1.6, label="model")
            ax.set_title(f"{kind} {AXES[col]}", fontsize=9, color=INK)
            if col == 0:
                ax.set_ylabel(f"Allan deviation, {small}", fontsize=8, color=INK)
            if row == 1:
                ax.set_xlabel("τ, s", fontsize=8, color=INK)
            _style(ax)
    axes[0, 0].legend(fontsize=8, frameon=False)
    return fig


def filter_shape(cap: capture.Capture, *, sensor: str = "finger", kind: str = "accel", axis: int = 0, trim_s: float = 60.0) -> Figure:
    """Measured noise PSD with the best Butterworth and the best one-pole model of the
    ODR-rate stage: the shape near Nyquist tells them apart. Where LPF1 alone shapes the band,
    its cutoff factor is fitted as well."""
    s = _sensor(cap, sensor)
    x, fs = _signal(s, kind, trim_s)
    f, p = bench.psd(x[:, axis] - x[:, axis].mean(), fs)
    path = fit.paths(s.header, s.odr_hz / float(s.header["odr_hz"]))[kind]
    lsb = float(s.header["accel_lsb_g" if kind == "accel" else "gyro_lsb_dps"])
    _, small, k = UNITS[kind]

    fig, ax = plt.subplots(figsize=(5.5, 3.6), layout="constrained")
    ax.semilogy(f[1:], np.sqrt(p[1:]) * k, color=REAL, linewidth=1.0, label="measured")
    band = (f >= fit.FIT_LOW_HZ) & (f < 0.98 * fs / 2)
    # Vary the family of the stage that shapes the band: the ODR-rate stage when the config
    # has one, the internal-rate stage otherwise; the other stage takes its best shape.
    odr_stage = path.odr_stage_hz > 0
    scales = fit.LPF1_SCALES if kind == "gyro" and not odr_stage else (1.0,)
    q = fit.quantization_psd(lsb, fs)
    for family, color, style in (("butterworth", SIM, "--"), ("one_pole", MODEL, ":")):
        fits = []
        for other in fit.shapes(path.internal_hz > 0 if odr_stage else False):
            for order, scale in itertools.product(range(1, fit.MAX_ORDER + 1), scales):
                varied = fit.Shape(family, order)
                internal, stage = (other, varied) if odr_stage else (varied, fit.Shape())
                scaled = dataclasses.replace(path, internal_hz=path.internal_hz * scale)
                model = fit.model_psd(f[band], fs, scaled, internal, stage)
                log_d = 0.5 * np.median(np.log10(p[band]) - np.log10(model))
                full = 10 ** (2 * log_d) * model + q
                rms = float(10 * np.sqrt(np.mean((np.log10(p[band]) - np.log10(full)) ** 2)))
                cutoff = path.odr_stage_hz if odr_stage else path.internal_hz * scale
                fits.append((rms, order, cutoff, full))
        rms, order, cutoff, full = min(fits, key=lambda t: t[0])
        name = "Butterworth" if family == "butterworth" else "one-pole"
        ax.semilogy(
            f[band], np.sqrt(full) * k, color=color, linestyle=style, linewidth=1.6,
            label=f"{name} order {order} at {cutoff:.1f} Hz ({rms:.1f} dB rms)",
        )
    ax.axvline(fs / 2, color=INK, linewidth=0.6)
    ax.set_xlabel("Hz", fontsize=8, color=INK)
    ax.set_ylabel(f"{kind} {AXES[axis]}, {small}/√Hz", fontsize=8, color=INK)
    _style(ax)
    ax.legend(fontsize=7, frameon=False, loc="lower left")
    return fig


def _color(name: str) -> str:
    return SIM if name == "timusim" else OTHER[name][0]


def v2_scores(results: dict[str, list[v2.Score]]) -> Figure:
    """V2 per position: the real mean of the reported holds with its acceptance window, and each
    simulator's mean, filled inside the window and hollow outside it."""
    positions = [p for p in v2.AXES if any(s.position == p for s in next(iter(results.values())))]
    names = list(results)
    offsets = np.linspace(-0.27, 0.27, len(names)) if len(names) > 1 else np.zeros(1)
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.4), layout="constrained")
    for ax, metric, label, k in (
        (axes[0], "tilt_deg", "tilt error, °", 1.0),
        (axes[1], "norm_error_g", "norm error, mg", 1e3),
    ):
        for i, pos in enumerate(positions):
            real = next(s for s in results[names[0]] if s.position == pos and s.metric == metric)
            lo, hi = (real.real_mean - real.window) * k, (real.real_mean + real.window) * k
            ax.fill_between([i - 0.4, i + 0.4], lo, hi, color=BAND, linewidth=0, zorder=0)
            ax.hlines(real.real_mean * k, i - 0.4, i + 0.4, color=REAL, linewidth=2)
            for name, dx in zip(names, offsets):
                s = next(x for x in results[name] if x.position == pos and x.metric == metric)
                color = _color(name)
                ax.plot(
                    i + dx, s.sim_mean * k, linestyle="none", marker=MARKERS[name], markersize=6,
                    color=color, markerfacecolor=color if s.passed else "white", markeredgewidth=1.4,
                )
        ax.set_xticks(range(len(positions)), positions)
        ax.set_xlim(-0.6, len(positions) - 0.4)
        ax.set_xlabel("up axis", fontsize=8, color=INK)
        ax.set_ylabel(label, fontsize=8, color=INK)
        if metric == "tilt_deg":
            ax.set_ylim(bottom=0)
        else:
            ax.axhline(0, color=INK, linewidth=0.6)
        _style(ax)
    marker = lambda name, color, face: plt.Line2D(
        [], [], linestyle="none", marker=MARKERS.get(name, "o"), markersize=6, color=color, markerfacecolor=face, markeredgewidth=1.4
    )
    handles = [matplotlib.patches.Patch(color=BAND), plt.Line2D([], [], color=REAL, linewidth=2)]
    handles += [marker(n, _color(n), _color(n)) for n in names] + [marker("", INK, "white")]
    labels = ["window", "real", *names, "outside window"]
    fig.legend(handles, labels, loc="outside right center", fontsize=8, frameon=False)
    return fig

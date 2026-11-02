"""Draws output/hold.png and output/movement.png from the CSVs timusim-quickstart wrote."""

import csv
import pathlib

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from scipy import signal

OUTPUT = pathlib.Path(__file__).parent / "output"
SENSORS = {"palm": "Palm (LSM6DSL)", "finger": "Finger (LSM9DS1)"}
AXES = {"x": "#2a78d6", "y": "#e4572e", "z": "#1b9e77"}
INK = "#3b3a37"


def load(name: str) -> dict[str, dict[str, np.ndarray]]:
    with (OUTPUT / f"{name}.csv").open() as f:
        rows = list(csv.DictReader(f))
    out = {}
    for sensor in SENSORS:
        picked = [r for r in rows if r["sensor"] == sensor]
        out[sensor] = {k: np.array([float(r[k]) for r in picked]) for k in rows[0] if k != "sensor"}
    return out


def style(ax: plt.Axes) -> None:
    ax.grid(True, color="#e4e3df", linewidth=0.6)
    ax.spines[["top", "right"]].set_visible(False)
    ax.tick_params(colors=INK, labelsize=8)


def hold() -> None:
    data = load("hold")
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), layout="constrained")
    for col, (sensor, title) in enumerate(SENSORS.items()):
        d = data[sensor]
        fs = 1.0 / np.median(np.diff(d["t_s"]))
        keep = (d["t_s"] >= 20) & (d["t_s"] < 25)
        for a, color in AXES.items():
            x = d[f"g{a}_dps"]
            axes[0, col].plot(d["t_s"][keep], x[keep] - x.mean(), color=color, linewidth=1, label=a)
            f, p = signal.welch(x - x.mean(), fs, nperseg=int(8 * fs))
            axes[1, col].loglog(f[1:], np.sqrt(p[1:]), color=color, linewidth=1, label=a)
        axes[0, col].set_title(f"{title}: gyro, mean removed, 5 s", fontsize=9, color=INK)
        axes[0, col].set_xlabel("Time, s", fontsize=8, color=INK)
        axes[0, col].set_ylabel("dps", fontsize=8, color=INK)
        axes[1, col].set_title(f"{title}: gyro spectrum", fontsize=9, color=INK)
        axes[1, col].set_xlabel("Frequency, Hz", fontsize=8, color=INK)
        axes[1, col].set_ylabel("dps/√Hz", fontsize=8, color=INK)
    for ax in axes.flat:
        style(ax)
    axes[0, 0].legend(title="Axis", fontsize=8, title_fontsize=8, frameon=False)
    fig.savefig(OUTPUT / "hold.png", dpi=150)


def movement() -> None:
    data = load("movement")
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, layout="constrained")
    for col, (sensor, title) in enumerate(SENSORS.items()):
        d = data[sensor]
        for a, color in AXES.items():
            axes[0, col].plot(d["t_s"], d[f"a{a}_g"], color=color, linewidth=1, label=a)
            axes[0, col].plot(d["t_s"], d[f"ideal_a{a}_g"], color=INK, linewidth=0.7, linestyle=":")
            axes[1, col].plot(d["t_s"], d[f"g{a}_dps"], color=color, linewidth=1, label=a)
            axes[1, col].plot(d["t_s"], d[f"ideal_g{a}_dps"], color=INK, linewidth=0.7, linestyle=":")
        axes[0, col].set_title(f"{title}: accelerometer", fontsize=9, color=INK)
        axes[0, col].set_ylabel("g", fontsize=8, color=INK)
        axes[1, col].set_title(f"{title}: gyroscope", fontsize=9, color=INK)
        axes[1, col].set_ylabel("dps", fontsize=8, color=INK)
        axes[1, col].set_xlabel("Time, s", fontsize=8, color=INK)
    for ax in axes.flat:
        style(ax)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    handles.append(plt.Line2D([], [], color=INK, linewidth=0.7, linestyle=":"))
    axes[0, 0].legend(handles, labels + ["Ideal input"], fontsize=8, frameon=False)
    fig.savefig(OUTPUT / "movement.png", dpi=150)


if __name__ == "__main__":
    hold()
    movement()
    print(f"wrote {OUTPUT / 'hold.png'} and {OUTPUT / 'movement.png'}")

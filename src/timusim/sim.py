"""Runs the C++ simulator and loads its truth CSV."""

import dataclasses
import pathlib
import subprocess

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_BINARY = ROOT / "build" / "sim" / "timusim-sim"


@dataclasses.dataclass
class Truth:
    index: np.ndarray
    t_s: np.ndarray
    specific_force_g: np.ndarray
    rate_dps: np.ndarray
    orientation_wxyz: np.ndarray
    position_m: np.ndarray
    joints_deg: np.ndarray


def run(
    scenario: pathlib.Path,
    out: pathlib.Path,
    *,
    seed: int | None = None,
    truth: bool = True,
    binary: pathlib.Path = DEFAULT_BINARY,
) -> pathlib.Path | None:
    """Writes OUT (capture) and, unless truth is False, OUT.truth.csv; returns the truth path."""
    truth_path = out.with_suffix(".truth.csv") if truth else None
    cmd = [str(binary), str(scenario), "-o", str(out)]
    cmd += ["--truth", str(truth_path)] if truth_path else ["--no-truth"]
    if seed is not None:
        cmd += ["--seed", str(seed)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"{binary.name} {scenario}: {result.stderr.strip() or f'exit {result.returncode}'}")
    return truth_path


def read_truth(path: pathlib.Path) -> dict[str, Truth]:
    names = np.loadtxt(path, delimiter=",", skiprows=1, usecols=0, dtype=str)
    values = np.loadtxt(path, delimiter=",", skiprows=1, usecols=range(1, 19))
    out = {}
    for name in np.unique(names):
        v = values[names == name]
        out[str(name)] = Truth(
            index=v[:, 0].astype(np.int64),
            t_s=v[:, 1],
            specific_force_g=v[:, 2:5],
            rate_dps=v[:, 5:8],
            orientation_wxyz=v[:, 8:12],
            position_m=v[:, 12:15],
            joints_deg=v[:, 15:18],
        )
    return out

"""The RESEARCH.md acceptance rule: a metric passes when the simulated mean is within
max(tolerance, 2 SD of the real values) of the real mean."""

import dataclasses
import typing

import numpy as np


@dataclasses.dataclass(frozen=True)
class Verdict:
    real_mean: float
    real_sd: float
    sim_mean: float
    window: float
    passed: bool


def judge(real: typing.Sequence[float], simulated: typing.Sequence[float], *, absolute: float = 0.0, relative: float = 0.0) -> Verdict:
    """The tolerance is absolute plus relative times the real mean's size."""
    r, s = np.asarray(real, dtype=float), np.asarray(simulated, dtype=float)
    mean = float(r.mean())
    sd = float(r.std(ddof=1)) if len(r) > 1 else 0.0
    window = max(absolute + relative * abs(mean) if relative else absolute, 2 * sd)
    if not len(s):
        return Verdict(mean, sd, float("nan"), window, False)
    return Verdict(mean, sd, float(s.mean()), window, bool(abs(s.mean() - mean) <= window))

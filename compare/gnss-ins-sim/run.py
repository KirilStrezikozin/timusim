# Copyright (C) 2026 Kiril V. Strezikozin
# SPDX-License-Identifier: Apache-2.0
"""A still hold through gnss-ins-sim's sensor error model, pathgen.acc_gen and gyro_gen: constant
bias, one first-order Gauss-Markov drift term and white noise on the ideal reading.
Usage: run.py JOB.json OUT.npz"""

import json
import pathlib
import sys

import numpy as np

SOURCE = pathlib.Path(__file__).resolve().parents[2] / "external" / "gnss-ins-sim"
sys.path.insert(0, str(SOURCE))

from gnss_ins_sim.pathgen import pathgen  # noqa: E402

G = 9.80665


def main(job_path: str, out_path: str) -> None:
    job = json.loads(pathlib.Path(job_path).read_text())
    np.random.seed(job["seed"])
    fs = job["odr_hz"]
    n = int(round(job["seconds"] * fs))
    up = np.asarray(job["up"], float) / np.linalg.norm(job["up"])

    def err(kind: str, si: float, white_key: str) -> dict[str, np.ndarray]:
        spec = job[kind]
        markov = spec.get("markov", {"sigma": [0.0] * 3, "tau_s": [np.inf] * 3})
        return {
            "b": np.asarray(spec["bias"]) * si,
            "b_drift": np.asarray(markov["sigma"]) * si,
            "b_corr": np.asarray(markov["tau_s"], float),
            # The white term is a density: acc_gen and gyro_gen divide it by sqrt(dt).
            white_key: np.asarray(spec["sd"]) * si / np.sqrt(fs),
        }

    accel = pathgen.acc_gen(fs, np.tile(up * G, (n, 1)), err("accel", G, "vrw"))
    gyro = pathgen.gyro_gen(fs, np.zeros((n, 3)), err("gyro", np.pi / 180, "arw"))
    np.savez(out_path, accel_g=accel / G, gyro_dps=np.degrees(gyro), odr_hz=fs)


if __name__ == "__main__":
    main(*sys.argv[1:3])

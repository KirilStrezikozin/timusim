# Copyright (C) 2026 Kiril V. Strezikozin
# SPDX-License-Identifier: Apache-2.0
"""A still hold through WIMUSim's IMU model, utils.simulate_imu in its "generate" use: constant
bias and white Gaussian noise on the ideal reading. Usage: run.py JOB.json OUT.npz"""

import json
import pathlib
import sys

import numpy as np
import scipy.spatial.transform

SOURCE = pathlib.Path(__file__).resolve().parents[2] / "external" / "wimusim"
sys.path.insert(0, str(SOURCE))

import torch  # noqa: E402
from wimusim import consts, utils  # noqa: E402


def main(job_path: str, out_path: str) -> None:
    job = json.loads(pathlib.Path(job_path).read_text())
    torch.manual_seed(job["seed"])
    n = int(round(job["seconds"] * job["odr_hz"]))
    g = torch.tensor(consts.E_DEFAULT["g"], dtype=torch.float64)
    g_norm = float(g.norm())

    # WIMUSim reads -g rotated into the sensor frame, so the sensor-to-world rotation takes up to -g.
    up = np.asarray(job["up"], float) / np.linalg.norm(job["up"])
    rot, _ = scipy.spatial.transform.Rotation.align_vectors([-g.numpy() / g_norm], [up])
    x, y, z, w = rot.as_quat()
    q = torch.tensor([w, x, y, z], dtype=torch.float64).repeat(n, 1)
    p = torch.zeros((n, 3), dtype=torch.float64)

    def spec(kind: str, si: float) -> tuple[torch.Tensor, torch.Tensor]:
        bias = torch.tensor(job[kind]["bias"], dtype=torch.float64) * si
        sd = torch.tensor(job[kind]["sd"], dtype=torch.float64) * si
        return bias, torch.randn((n, 3), dtype=torch.float64) * sd

    ba, eta_a = spec("accel", g_norm)
    bg, eta_g = spec("gyro", np.pi / 180)
    acc, gyro = utils.simulate_imu(p, q, ba=ba, bg=bg, eta_a=eta_a, eta_g=eta_g, g=g, ts=1.0 / job["odr_hz"])
    np.savez(
        out_path,
        accel_g=acc.detach().numpy() / g_norm,
        gyro_dps=np.degrees(gyro.detach().numpy()),
        odr_hz=job["odr_hz"],
    )


if __name__ == "__main__":
    main(*sys.argv[1:3])

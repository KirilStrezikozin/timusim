# Copyright (C) 2026 Kiril V. Strezikozin
# SPDX-License-Identifier: Apache-2.0
"""A still hold through MATLAB's imuSensor (timusim_hold.m), run in the Ubuntu distrobox that
holds the MATLAB install. Usage: run.py JOB.json OUT.npz

TIMUSIM_MATLAB_BOX names the distrobox (default timusim-matlab); TIMUSIM_MATLAB_ROOT the install
(default external/matlab/R2026b)."""

import os
import pathlib
import shutil
import subprocess
import sys

import numpy as np
import scipy.io

HERE = pathlib.Path(__file__).resolve().parent
ROOT = pathlib.Path(os.environ.get("TIMUSIM_MATLAB_ROOT", HERE.parents[1] / "external" / "matlab" / "R2026b"))
BOX = os.environ.get("TIMUSIM_MATLAB_BOX", "timusim-matlab")


def main(job_path: str, out_path: str) -> None:
    distrobox = shutil.which("distrobox")
    if distrobox is None:
        raise SystemExit("distrobox not on PATH: run from `nix develop .#matlab`")
    job = pathlib.Path(job_path).resolve()
    mat = pathlib.Path(out_path).resolve().with_suffix(".mat")
    call = f"addpath('{HERE}'); timusim_hold('{job}', '{mat}')"
    cmd = [distrobox, "enter", BOX, "--", str(ROOT / "bin" / "matlab"), "-licmode", "onlinelicensing", "-batch", call]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0 or not mat.exists():
        raise SystemExit(f"matlab failed ({result.returncode}):\n{result.stdout}\n{result.stderr}")
    d = scipy.io.loadmat(mat)
    np.savez(out_path, accel_g=d["accel_g"], gyro_dps=d["gyro_dps"], odr_hz=float(d["odr_hz"].squeeze()))
    mat.unlink()


if __name__ == "__main__":
    main(*sys.argv[1:3])

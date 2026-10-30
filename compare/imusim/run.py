# Copyright (C) 2026 Kiril V. Strezikozin
# SPDX-License-Identifier: GPL-3.0-or-later
"""A still hold through IMUSim: affine sensors with white noise, sampled by a 16-bit ADC at the
job's ODR. Usage: run.py JOB.json OUT.npz"""

import json
import os
import pathlib
import shutil
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
SOURCE = HERE.parents[1] / "external" / "imusim"
BUILD = HERE / ".build"
EXTENSIONS = {
    "quaternions": [],
    "quat_splines": [],
    "vectors": [],
    "natural_neighbour": ["natural_neighbour/utils.c", "natural_neighbour/delaunay.c", "natural_neighbour/natural.c"],
}
ADC_BITS = 16
VREF = 1.0


def build() -> None:
    """IMUSim's setup.py requires mayavi, so its Cython modules are built here, in a copy."""
    if (BUILD / ".built").exists():
        return
    import Cython.Build
    import setuptools

    shutil.rmtree(BUILD, ignore_errors=True)
    shutil.copytree(SOURCE / "imusim", BUILD / "imusim")
    maths = BUILD / "imusim" / "maths"
    for name in EXTENSIONS:
        (maths / f"{name}.c").unlink(missing_ok=True)
    exts = [
        setuptools.Extension(
            f"imusim.maths.{name}",
            [f"imusim/maths/{name}.pyx", *(f"imusim/maths/{c}" for c in extra)],
            include_dirs=[np.get_include()],
        )
        for name, extra in EXTENSIONS.items()
    ]
    cwd = pathlib.Path.cwd()
    try:
        os.chdir(BUILD)
        setuptools.setup(
            name="imusim-build",
            script_args=["build_ext", "--inplace", "-q"],
            ext_modules=Cython.Build.cythonize(exts, language_level=3, quiet=True),
        )
    finally:
        os.chdir(cwd)
    (BUILD / ".built").touch()


def main(job_path: str, out_path: str) -> None:
    build()
    sys.path.insert(0, str(BUILD))
    from imusim.behaviours.imu import BasicIMUBehaviour
    from imusim.environment.gravity import STANDARD_GRAVITY
    from imusim.maths.quaternions import Quaternion, QuaternionFromAxisAngle
    from imusim.platforms.accelerometers import NoisyTransformedAccelerometer
    from imusim.platforms.adcs import QuantisingADC
    from imusim.platforms.gyroscopes import NoisyTransformedGyroscope
    from imusim.platforms.imus import StandardIMU
    from imusim.platforms.magnetometers import IdealMagnetometer
    from imusim.platforms.radios import IdealRadio
    from imusim.platforms.timers import IdealTimer
    from imusim.simulation.base import Simulation
    from imusim.trajectories.base import StaticTrajectory

    job = json.loads(pathlib.Path(job_path).read_text())

    def affine(spec: dict, si: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Noise, gain and offset in ADC volts, so that one code is one LSB of the chip. The gain
        is the job's 3x3 gain when given."""
        scale = VREF / (2 ** (ADC_BITS - 1) * spec["lsb"] * si)
        column = lambda v: scale * si * np.asarray(v, float).reshape(3, 1)
        gain = np.asarray(spec.get("gain", np.eye(3)), float)
        return column(spec["sd"]), scale * gain, VREF + column(spec["bias"])

    class StillIMU(StandardIMU):
        def __init__(self, simulation, trajectory):
            self.adc = QuantisingADC(self, bits=ADC_BITS, vref=VREF)
            self.timer = IdealTimer(self)
            self.accelerometer = NoisyTransformedAccelerometer(self, *affine(job["accel"], STANDARD_GRAVITY))
            self.gyroscope = NoisyTransformedGyroscope(self, *affine(job["gyro"], np.pi / 180))
            self.magnetometer = IdealMagnetometer(self)
            self.radio = IdealRadio(self)
            StandardIMU.__init__(self, simulation, trajectory)

    # The rotation that takes up in the sensor frame to world up, which is minus gravity.
    up = np.asarray(job["up"], float).reshape(3, 1)
    up /= np.linalg.norm(up)
    sim = Simulation(seed=job["seed"])
    sim.time = 0.0
    world_up = -sim.environment.gravitationalField(np.zeros((3, 1)), 0) / STANDARD_GRAVITY
    axis = np.cross(up.ravel(), world_up.ravel())
    angle = float(np.arccos(np.clip(up.ravel() @ world_up.ravel(), -1.0, 1.0)))
    if np.linalg.norm(axis) < 1e-9 and angle > np.pi / 2:
        # Opposite directions: half a turn about any axis perpendicular to up.
        axis = np.cross(up.ravel(), np.eye(3)[np.argmin(np.abs(up.ravel()))])
    rotation = Quaternion() if np.linalg.norm(axis) < 1e-9 else QuaternionFromAxisAngle(axis / np.linalg.norm(axis), angle)
    if not np.allclose(rotation.rotateFrame(world_up), up, atol=1e-9):
        rotation = rotation.conjugate
    imu = StillIMU(sim, StaticTrajectory(rotation=rotation))
    BasicIMUBehaviour(imu, 1.0 / job["odr_hz"])
    sim.run(job["seconds"], printProgress=False)

    accel = np.asarray(imu.accelerometer.rawMeasurements.values).T * job["accel"]["lsb"]
    gyro = np.asarray(imu.gyroscope.rawMeasurements.values).T * job["gyro"]["lsb"]
    np.savez(out_path, accel_g=accel, gyro_dps=gyro, odr_hz=job["odr_hz"])


if __name__ == "__main__":
    main(*sys.argv[1:3])

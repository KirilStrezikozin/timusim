"""Live placement against earlier holds: records the board into a scratch directory and prints,
every few seconds, how far one sensor's gravity is from each reference hold. With a hinge axis
it also splits the offset from the first reference into the turn about the hinge and the angle of
gravity to the hinge, which tell an arm off its stop from a box tilted or turned."""

import pathlib
import subprocess
import sys
import tempfile
import time

import numpy as np

from timusim import capture, manifest, v4


def angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip(a @ b / np.linalg.norm(a) / np.linalg.norm(b), -1.0, 1.0))))


def report(up: np.ndarray, refs: dict[str, np.ndarray], hinge: np.ndarray | None) -> str:
    parts = [f"off {name} {angle_deg(up, ref):5.1f}" for name, ref in refs.items()]
    if hinge is not None:
        first, ref = next(iter(refs.items()))
        parts.append(f"turn from {first} {v4.gravity_angle(hinge, ref, up):+6.1f}")
        parts.append(f"to hinge {angle_deg(up, hinge):5.1f} (ref {angle_deg(ref, hinge):5.1f})")
    return "  ".join(parts) + f"  up {np.round(up / np.linalg.norm(up), 3)}"


def run(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("refs", type=pathlib.Path, nargs="+", help="hold captures to place against")
    parser.add_argument("--sensor", default="palm_lsm9")
    parser.add_argument(
        "--hinge", help="hinge axis in the sensor's frame, x,y,z; 'v4' takes V4 session 1's, positive from A towards B"
    )
    parser.add_argument("--every-s", type=float, default=2.0)
    parser.add_argument("--port", default="/dev/ttyACM0")
    args = parser.parse_args(argv)

    refs = {p.stem.rsplit("-", 1)[-1]: v4.hold_up(capture.read_capture(p).stream(args.sensor)) for p in args.refs}
    hinge = None
    if args.hinge == "v4":
        hinge = v4.fit_geometry(manifest.DATA_DIR, log=lambda _: None).hinge[args.sensor]
    elif args.hinge:
        hinge = np.array([float(c) for c in args.hinge.split(",")])
        hinge /= np.linalg.norm(hinge)
    with tempfile.TemporaryDirectory(prefix="timusim-place-") as scratch:
        recorder = subprocess.Popen(
            [str(pathlib.Path(sys.executable).parent / "timusim-capture"), "--test", "place", "--port", args.port,
             "--data-dir", scratch],
            stderr=subprocess.DEVNULL,
        )
        try:
            while not list(pathlib.Path(scratch).glob("*/*.bin")):
                time.sleep(0.5)
            live = next(pathlib.Path(scratch).glob("*/*.bin"))
            while recorder.poll() is None:
                time.sleep(args.every_s)
                try:
                    s = capture.read_capture(live).stream(args.sensor)
                except (KeyError, ValueError):
                    continue
                print(time.strftime("%H:%M:%S"), report(s.accel_g[-int(s.odr_hz) :].mean(axis=0), refs, hinge), flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            recorder.terminate()
            recorder.wait()
    return 0

"""Tilt or bias? A board that tilts keeps |a| while its components move; a bias drift moves |a|
too. 100-s means of each long hold: component swing against norm swing, and when it happens."""
import numpy as np
from timusim import capture, manifest

DATA = manifest.DATA_DIR
BIN = 100.0
for e in manifest.load(DATA / manifest.MANIFEST_PATH.name):
    if e.get("config") != "long" or e.get("excluded"):
        continue
    cap = capture.read_capture(DATA / e["file"])
    for s in cap.sensors.values():
        if s.name not in ("finger", "palm"):
            continue
        keep = s.longest_run(s.t_s >= 60.0)
        t, a = s.t_s[keep], s.accel_g[keep]
        n = int((t[-1] - t[0]) // BIN)
        m = np.array([a[(t >= t[0] + i * BIN) & (t < t[0] + (i + 1) * BIN)].mean(axis=0) for i in range(n)])
        norm = np.linalg.norm(m, axis=1)
        u = m / norm[:, None]
        tilt = np.degrees(np.arccos(np.clip(u @ u[0], -1, 1)))
        comp = np.max(np.ptp(m, axis=0)) * 1e6
        # largest 100-s step in direction
        steps = np.degrees(np.arccos(np.clip(np.sum(u[1:] * u[:-1], axis=1), -1, 1)))
        k = int(np.argmax(steps))
        print(f"{e['file'][11:]:24s} {s.name:6s} component swing {comp:6.0f} ug | norm swing {np.ptp(norm)*1e6:5.0f} ug | "
              f"direction moves {tilt.max():.3f} deg, biggest 100-s step {steps[k]:.3f} deg at {(k + 1) * BIN / 60:.0f} min", flush=True)
    del cap

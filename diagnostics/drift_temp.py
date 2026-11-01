"""Finger and palm accel drift in each long hold against the sensor's temperature: 100-s means,
range of each, and the slope and correlation of accel on temperature."""
import numpy as np
from timusim import capture, manifest

DATA = manifest.DATA_DIR
holds = [e for e in manifest.load(DATA / manifest.MANIFEST_PATH.name) if e.get("config") == "long" and not e.get("excluded")]
BIN = 100.0
for e in holds:
    cap = capture.read_capture(DATA / e["file"])
    for sid, s in cap.sensors.items():
        if s.name not in ("finger", "palm") or sid not in cap.temps:
            continue
        keep = s.longest_run(s.t_s >= 60.0)
        t, a = s.t_s[keep], s.accel_g[keep]
        edges = np.arange(t[0], t[-1], BIN)
        idx = np.digitize(t, edges)
        means = np.array([a[idx == i].mean(axis=0) for i in range(1, len(edges))])
        mid = edges[:-1] + BIN / 2
        tt, tc = cap.temps[sid]
        temp = np.interp(mid, tt, tc)
        line = f"{e['file'][11:]:24s} {s.name:6s} temp {temp.min():5.1f}-{temp.max():5.1f} C |"
        for k in range(3):
            y = (means[:, k] - means[:, k].mean()) * 1e6
            if np.ptp(temp) > 0.05:
                slope, _ = np.polyfit(temp, y, 1)
                r = np.corrcoef(temp, y)[0, 1]
                resid = y - np.polyval(np.polyfit(temp, y, 1), temp)
                line += f" {'xyz'[k]} p2p {np.ptp(y):5.0f} ug, {slope:+6.0f} ug/C r {r:+.2f} left {np.ptp(resid):4.0f} |"
            else:
                line += f" {'xyz'[k]} p2p {np.ptp(y):5.0f} ug (temp flat) |"
        print(line, flush=True)
    del cap

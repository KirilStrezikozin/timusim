"""Whether the palm accel z excess settles: the same numbers per 5-min chunk of one capture
(RESEARCH.md 2026-10-29, board settling on a soft support). Usage: palm_z_chunks.py CAPTURE [SENSOR...]"""
import sys, numpy as np
from scipy import signal
from timusim import capture, rig
cap = capture.read_capture(sys.argv[1])
for name in ("palm", "finger"):
    s = cap.stream(name); fs = s.odr_hz
    t = s.t_s
    for lo in range(0, int(t[-1]) - 60, 300):
        m = (t >= lo) & (t < lo + 300)
        a = s.accel_g[m]
        # gaps break block means; keep it simple: skip windows with big gaps
        gap = np.max(np.diff(t[m])) if m.sum() > 2 else 1e9
        n = int(fs); b = a[: len(a) // n * n].reshape(-1, n, 3).mean(1)
        adev = np.sqrt(0.5 * np.mean(np.diff(b, axis=0) ** 2, axis=0)) * 1e6
        f, p = signal.welch(a - a.mean(0), fs, nperseg=int(fs * 20), axis=0)
        d = np.median(np.sqrt(p[(f >= 0.05) & (f < 0.5)]) * 1e6, 0)
        print(f"{name:6s} {lo//60:2d}-{lo//60+5:2d} min  adev1s {np.round(adev).astype(int)}  0.05-0.5Hz {np.round(d).astype(int)} ug/rtHz  max gap {gap:.2f} s")

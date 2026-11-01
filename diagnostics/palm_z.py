"""Palm LSM6DSL accel z excess noise: Allan deviation, PSD bands, tones and the spread of 0.1-s
means of one capture (RESEARCH.md 2026-10-29). Usage: palm_z.py CAPTURE [SENSOR...]"""
import sys, numpy as np
from scipy import signal
from timusim import capture, rig
cap = capture.read_capture(sys.argv[1])
for name in sys.argv[2:] or ["palm", "finger"]:
    s = cap.stream(name)
    k = s.longest_run(s.t_s >= 5)
    a = s.accel_g[k]; fs = s.odr_hz
    def adev(x, n):
        b = x[: len(x) // n * n].reshape(-1, n, x.shape[1]).mean(1)
        return np.sqrt(0.5 * np.mean(np.diff(b, axis=0) ** 2, axis=0)) * 1e6
    print(f"{name}: {len(a)/fs:.0f} s, mean g {np.round(a.mean(0),3)}, adev ug 0.1s {np.round(adev(a,int(fs/10))).astype(int)} 1s {np.round(adev(a,int(fs))).astype(int)} 10s {np.round(adev(a,int(10*fs))).astype(int)}")
    f, p = signal.welch(a - a.mean(0), fs, nperseg=int(fs * 20), axis=0)
    d = np.sqrt(p) * 1e6  # ug/rtHz
    for lo, hi in ((0.05, 0.5), (0.5, 5), (5, 50), (50, 200)):
        m = (f >= lo) & (f < hi)
        print(f"   {lo:>5}-{hi:<4} Hz density ug/rtHz {np.round(np.median(d[m], 0)).astype(int)}")
    # tones on z: bins far above the local median
    z = d[:, 2]; med = signal.medfilt(z, 101)
    pk = np.flatnonzero((z > 4 * med) & (f > 0.2))
    if len(pk): print("   z peaks Hz:", np.round(f[pk][:12], 2), "ratio", np.round((z / med)[pk][:12], 1))
    b = a[: len(a) // int(fs / 10) * int(fs / 10), 2].reshape(-1, int(fs / 10)).mean(1)
    h, e = np.histogram((b - np.median(b)) * 1e3, bins=12)
    print("   z 0.1-s means, mg, histogram:", list(zip(np.round(e[:-1], 2).tolist(), h.tolist())))

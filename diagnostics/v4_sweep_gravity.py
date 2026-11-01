"""Gravity per sensor at every rest of the session-1 sweeps: angle to the hinge (out of the arm's
plane) and hinge angle from the hold at A, against rest number."""
import numpy as np
from timusim import capture, manifest, rig, v4

D = manifest.DATA_DIR
g = v4.fit_geometry(D, log=lambda s: None)
corr = v4.accel_params(D)
F = {"hinge_horizontal": ("013340", "013957", "014928"), "tilted": ("015758", "020038", "020504"), "hinge_vertical": ("021404", "021630", "021901")}
cap = lambda f: capture.read_capture(D / f"2026-10-28/V4-default-s1-{f}.bin")
for o, (fa, fb, fs) in F.items():
    ca, cb, cs = cap(fa), cap(fb), cap(fs)
    l9 = cs.stream("palm_lsm9")
    rs = v4.rests(l9, g.hinge["palm_lsm9"], o, v4.hold_up(ca.stream("palm_lsm9")), v4.sweep_rest_deg(g, o))
    print(f"\n{o}: angle of gravity to the hinge, deg (90 = hinge level); hold A / hold B, then each sweep rest")
    for n in rig.SENSORS:
        h = g.hinge[n]
        s = cs.stream(n)
        t = rig.board_time(s)
        ang = lambda u: np.degrees(np.arccos(np.clip(u @ h, -1, 1)))
        ua, ub = (v4.hold_up(c.stream(n), corr[n]) for c in (ca, cb))
        A, B = [], []
        for r in rs:
            seg = v4.segment(t, r.start, r.end)
            a = s.accel_g[seg].mean(axis=0)
            a = np.linalg.solve(np.array(corr[n].get("accel_gain", np.eye(3))), a - np.array(corr[n].get("accel_offset", np.zeros(3))))
            u = a / np.linalg.norm(a)
            (B if r.at_b else A).append(ang(u))
        fit_a = np.polyfit(np.arange(len(A)), A, 1)[0] if len(A) > 2 else np.nan
        fit_b = np.polyfit(np.arange(len(B)), B, 1)[0] if len(B) > 2 else np.nan
        print(f"  {n:9s} hold A {ang(ua):6.2f} B {ang(ub):6.2f} | rests A " + " ".join(f"{x:.1f}" for x in A)
              + f"  (trend {fit_a:+.2f}/rest)\n  {'':9s}{'':27s} rests B " + " ".join(f"{x:.1f}" for x in B) + f"  (trend {fit_b:+.2f}/rest)")

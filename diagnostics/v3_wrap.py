"""V3 session 2: the samples around each accel x- event the old rule counted as a wrap, with the
reference input the output is compared against (RESEARCH.md 2026-10-30, wrap redefined)."""
import pathlib, numpy as np
from timusim import capture, manifest, rig, v3, v4
D = manifest.DATA_DIR
g1 = v4.fit_geometry(D, log=lambda s: None); g = v4.remounted(g1, D, log=lambda s: None)
for e in rig.entries(D, "V3"):
    if e.config != "v3": continue
    cap = capture.read_capture(e.path); ref = cap.stream("palm")
    for narrow in ("finger", "palm_lsm9"):
        s = cap.stream(narrow); geo = g if e.session >= v4.REMOUNT_SESSION else g1
        m = v3.mapping(geo, "palm", narrow, v3.fit_delay(ref, s, v3.mapping(geo, "palm", narrow).rotation))
        force, _ = v3.reference_input(ref, s, m); fs = v3.full_scale(s, "accel")
        codes = s.accel_codes.astype(np.int32)
        evs = [x for x in v3.events(s, "accel", force, fs) if x.axis == 0 and x.sign == -1]
        print(f"s{e.session} {e.path.name} {narrow} accel x- events {len(evs)} wrapped {sum(x.wrapped for x in evs)}")
        # locate wrapped spans again
        hit = -codes[:, 0] >= rig.CLIP_CODE; gap = int(v3.MERGE_S * s.odr_hz); spans = []
        for a, b in rig.runs(hit):
            if spans and a - spans[-1][1] < gap: spans[-1][1] = b
            else: spans.append([a, b])
        for a, b in spans:
            past = -force[a:b, 0] > fs
            flips = np.flatnonzero(past & (codes[a:b, 0] > 0))
            if not len(flips): continue
            i = a + flips[0]
            seg = slice(max(i - 6, 0), i + 6)
            print(f"   span {a}-{b} ({(b-a)/s.odr_hz*1e3:.0f} ms), {len(flips)} flipped samples; around first:")
            print("     code ", codes[seg, 0].tolist())
            print("     ref g", np.round(force[seg, 0], 2).tolist(), " full scale", round(fs, 2))
            print("     step to next code", int(np.max(np.abs(np.diff(codes[a:b, 0])))))

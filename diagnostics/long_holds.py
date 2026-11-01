"""Long holds: Allan deviation at long taus, zero level and temperature, rig vs bench vs model."""
import json, pathlib, tomllib
import numpy as np
from timusim import capture, fit, manifest, v1

DATA = manifest.DATA_DIR
TAUS = np.array([1.0, 10.0, 100.0, 1000.0])
PARAMS = {"finger": DATA / "v1/lsm9ds1.toml", "palm": DATA / "v1-palm/lsm6dsl.toml", "palm_lsm9": DATA / "v1/lsm9ds1.toml"}
MODEL_OF = {"finger": "finger", "palm": "palm", "palm_lsm9": "finger"}

def white_levels(sensor):
    es = v1.entries(DATA, sensor=sensor)
    fits = fit.fit_files([e.path for e in es if e.session == 1 and e.config == "default"], sensor, allan=False)
    return {k: [float(np.median([getattr(f, k)[i].white_low for f in fits])) for i in range(3)] for k in ("accel", "gyro")}

def model(sensor, white):
    noise = tomllib.loads(PARAMS[sensor].read_text())
    out = {}
    for kind in ("accel", "gyro"):
        n = noise[kind]
        for col in range(3):
            d = fit.Drift(tuple((t["sigma"][col], t["tau_s"][col]) for t in n.get("markov", [])), n.get("rate_random_walk", [0.0] * 3)[col])
            out[(kind, col)] = np.sqrt(fit.white_avar(white[kind][col], TAUS) + d.avar(TAUS))
    return out

holds = [e for e in manifest.load(DATA / manifest.MANIFEST_PATH.name) if e.get("config") == "long" and not e.get("excluded")]
rows = []
for e in holds:
    cap = capture.read_capture(DATA / e["file"])
    for s in cap.sensors.values():
        keep = s.longest_run(s.t_s >= 60.0)
        fs = s.odr_hz
        r = {"file": e["file"], "test": e["test"], "power_up": e.get("power_up"), "sensor": s.name,
             "hours": float(s.t_s[keep][-1] - s.t_s[keep][0]) / 3600}
        for kind, x in (("accel", s.accel_g[keep]), ("gyro", s.gyro_dps[keep])):
            for col in range(3):
                tau, dev = fit.allan_curve(x[:, col], fs, points=60)
                r[f"{kind}{col}"] = np.interp(np.log(TAUS), np.log(tau), dev, right=np.nan).tolist()
            r[f"{kind}_zero"] = x.mean(axis=0).tolist()
        sid = s.id
        if sid in cap.temps:
            c = cap.temps[sid][1]
            r["temp"] = [float(c.min()), float(c.max())]
        rows.append(r)
    del cap
    print("read", e["file"], flush=True)
out = manifest.DATA_DIR / "diagnostics" / "long_holds.json"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(rows))

models = {s: model(MODEL_OF[s], white_levels(MODEL_OF[s])) for s in ("finger", "palm")}
models["palm_lsm9"] = models["finger"]
print("\nAllan deviation relative to the model (real/model) at tau =", TAUS.tolist())
for sensor in ("palm", "finger", "palm_lsm9"):
    for kind in ("accel", "gyro"):
        for col in range(3):
            m = models[sensor][(kind, col)]
            for test in ("V1", "V1rig"):
                rs = [r for r in rows if r["sensor"] == sensor and r["test"] == test]
                if not rs:
                    continue
                ratio = np.array([np.array(r[f"{kind}{col}"]) / m for r in rs])
                print(f"{sensor:9s} {kind}{'xyz'[col]} {test:5s} n={len(rs)} median " + " ".join(f"{v:5.2f}" for v in np.nanmedian(ratio, 0))
                      + "  range " + " ".join(f"{a:.2f}-{b:.2f}" for a, b in zip(np.nanmin(ratio, 0), np.nanmax(ratio, 0))))
print("\nGyro zero level (dps) and temperature per hold")
for r in rows:
    print(f"{r['sensor']:9s} {r['test']:5s} pu {r['power_up']} {r['hours']:.1f} h zero " + " ".join(f"{v:+.3f}" for v in r["gyro_zero"])
          + (f"  temp {r['temp'][0]:.1f}-{r['temp'][1]:.1f} C" if "temp" in r else ""))

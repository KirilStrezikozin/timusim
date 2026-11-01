"""Is the accel drift model high, or is the real median low because long-tau Allan estimates
scatter? Synthesize the model's white + Markov + random walk at each real hold's gap-free length
many times and see where the real holds fall in that distribution."""
import json, pathlib, tomllib
import numpy as np
from timusim import bench, manifest

DATA = manifest.DATA_DIR
TAUS = np.array([1.0, 10.0, 100.0, 1000.0])
FS = 20.0
rows = json.loads((manifest.DATA_DIR / "diagnostics" / "long_holds.json").read_text())
params = {"finger": DATA / "v1/lsm9ds1.toml", "palm": DATA / "v1-palm/lsm6dsl.toml"}
rng = np.random.default_rng(0)

def synth(n, white, markov, rw):
    x = white * np.sqrt(FS / 2) * rng.standard_normal(n)
    for sigma, tc in markov:
        a = np.exp(-1 / (FS * tc))
        e = sigma * np.sqrt(1 - a * a) * rng.standard_normal(n)
        m = np.empty(n); m[0] = sigma * rng.standard_normal()
        for i in range(1, n):  # noqa: slow but fine at 20 Hz
            m[i] = a * m[i - 1] + e[i]
        x += m
    if rw:
        x += np.cumsum(rw / np.sqrt(FS) * rng.standard_normal(n))
    return x

import scipy.signal
def synth_fast(n, white, markov, rw):
    x = white * np.sqrt(FS / 2) * rng.standard_normal(n)
    for sigma, tc in markov:
        a = np.exp(-1 / (FS * tc))
        e = sigma * np.sqrt(1 - a * a) * rng.standard_normal(n)
        zi = np.array([a * sigma * rng.standard_normal()])
        x += scipy.signal.lfilter([1.0], [1.0, -a], e, zi=zi)[0]
    if rw:
        x += np.cumsum(rw / np.sqrt(FS) * rng.standard_normal(n))
    return x

for sensor in ("finger", "palm"):
    p = tomllib.loads(params[sensor].read_text())["accel"]
    holds = [r for r in rows if r["sensor"] == sensor]
    for col in range(3):
        markov = [(t["sigma"][col], t["tau_s"][col]) for t in p.get("markov", [])]
        rw = p.get("rate_random_walk", [0.0] * 3)[col]
        white = p["white_density"][col]
        real = np.array([r[f"accel{col}"] for r in holds])  # g
        sims = []
        for r in holds:
            n = int(r["hours"] * 3600 * FS)
            sim_h = []
            for _ in range(40):
                x = synth_fast(n, white, markov, rw)
                taus = tuple(t for t in TAUS[1:] if t <= n / FS / 3)
                _, dev = bench.allan_deviation(x, FS, taus)
                sim_h.append(np.concatenate([[np.nan], dev, [np.nan] * (3 - len(dev))]))
            sims.append(np.array(sim_h))
        print(f"\n{sensor} accel {'xyz'[col]}: Markov {[(f'{s*1e6:.0f} ug', f'{t:.0f} s') for s, t in markov]}, random walk {rw*1e6:.2f} ug/sqrt(s)")
        for k, tau in enumerate(TAUS[1:], start=1):
            pct = [np.mean(s[:, k] < rv[k]) * 100 for s, rv in zip(sims, real) if np.isfinite(rv[k]) and np.isfinite(s[:, k]).all()]
            tag = [r["test"][-3:] if r["test"] == "V1rig" else "" for r in holds]
            sim_med = np.nanmedian(np.concatenate([s[:, k] for s in sims]))
            print(f"  tau {tau:6.0f} s: real median {np.nanmedian(real[:, k])*1e6:6.1f} ug, model-sim median {sim_med*1e6:6.1f} ug; "
                  f"each real hold's percentile among its sims: " + " ".join(f"{q:.0f}{t and '*'}" for q, t in zip(pct, tag)))
print("\n* = rig hold")

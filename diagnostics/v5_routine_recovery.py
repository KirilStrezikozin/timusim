"""V5 routine fit on simulated trials: session 1's measured repetitions replayed with a known
routine gain, roughness drive and mount jolts; the fit should return them."""

import tomllib

from timusim import capture, manifest, rig, v5

D = manifest.DATA_DIR
W = D / "v5" / "work" / "routine-recovery"
W.mkdir(parents=True, exist_ok=True)
fit = v5.Fit.read(D / "v5" / "fit.toml")
mounts = (fit.palm_mount, fit.finger_mount)
hand = tomllib.loads(v5.HAND_FILE.read_text())
params = {n: D / rig.PARAMS[n] for n in v5.SENSORS}
base_drives = [d for d in fit.drives if d.get("name") != "roughness"]
at = v5.fitted_scales(fit.tremor)
lit = v5.literature()
true_gain = 1.5
true_rough = {"name": "roughness", "frequency_hz": 7.5, "bandwidth_hz": 1.0, "harmonics": [1.0], "gate": "speed",
              "gain": {j: g * v5.REFERENCE_DENSITY_PER_STIFFNESS * v5.oscillator(j, at[v5.family(j)], lit)["stiffness"]
                       for j, g in {"wrist_flex": 0.2, "wrist_dev": 0.6, "sway_ry": 0.3, "mcp_flex": 0.1, "pip_flex": 0.1, "mcp_abd": 0.1}.items()}}
true_jolts = {"palm": {"frequency_hz": 30.0, "bandwidth_hz": 20.0, "level_dps": 0.3},
              "finger": {"frequency_hz": 30.0, "bandwidth_hz": 20.0, "level_dps": 1.0}}
real = [capture.read_capture(t.path) for t in v5.trials(D) if t.session == 1]
caps = []
for i, cap in enumerate(real):
    reps = v5.measure_reps(cap, mounts)
    text = v5.scenario(hand, v5.Chips.of(cap), params, v5.timeline_moves(reps, true_gain), seed=31 + i,
                       start_support=fit.table_support, tremor=fit.tremor, mounts=mounts, drives=base_drives + [true_rough],
                       jolts=true_jolts)
    caps.append(v5.simulate(text, W / f"trial-{i}.toml"))


def load(stem: str) -> tuple[dict, dict]:
    c = capture.read_capture(D / "v5" / "work" / f"{stem}.bin")
    f, p = {}, {}
    for n, m in zip(v5.SENSORS, mounts):
        s = c.stream(n)
        k = s.t_s >= v5.PHASES["table"][0]
        f[n], p[n] = v5.axis_psd(s.gyro_dps[k], s.odr_hz, m)
    return f, p


f, none = load("unit-none")
units = v5.UnitRuns(f, none, {(j, s): load(f"unit-{j}-{k}")[1] for j in v5.FIT_JOINTS for k, s in enumerate(v5.scales(j))})
fit.drives = base_drives
gain, rough, jolts, _ = v5.fit_routine(hand, params, mounts, caps, units, at, fit, W, print)
print(f"routine gain {gain:.3f} true {true_gain}")
print(f"roughness {rough['frequency_hz']:.2f} Hz width {rough['bandwidth_hz']:.2f} (true 7.5, 1.0)")
for j, g in true_rough["gain"].items():
    print(f"  {j}: gain {rough['gain'][j]:.4g} true {g:.4g} ({rough['gain'][j] / g - 1:+.0%})")
for n, j in true_jolts.items():
    got = jolts[n]
    print(f"jolt {n}: {got['frequency_hz']:.1f} Hz width {got['bandwidth_hz']:.1f} level {got['level_dps']:.3g} "
          f"(true {j['frequency_hz']:g}, {j['bandwidth_hz']:g}, {j['level_dps']:g})")

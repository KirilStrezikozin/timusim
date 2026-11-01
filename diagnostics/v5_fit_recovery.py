"""V5's session-1 fit on simulated trials: three trials from a known tremor, mounts and table
support, then the mount fit and the tremor fit on a two-point stiffness grid, against the truth
(RESEARCH.md, V5 procedure). Needs the V2 parameter files and the night-9 V1rig capture for the
chip config; about 20 sims, a few minutes. Work files go to data/v5/recovery."""
import tomllib, numpy as np
from timusim import capture, manifest, rig, v5
D = manifest.DATA_DIR
W = D / "v5" / "recovery"; W.mkdir(parents=True, exist_ok=True)
params = {n: D / rig.PARAMS[n] for n in v5.SENSORS}
ref = capture.read_capture(D / "2026-10-31/V1rig-default-s3-030232.bin")
chips = v5.Chips.of(ref)
hand = tomllib.loads(v5.HAND_FILE.read_text())
lit = v5.literature()
up, flex = [0, 0, 1.0], [-1.0, 0, 0]
true_mounts = (rig.fit_rotation(np.array([up, flex]), np.array([[0, 0, 1.0], [0, 1.0, 0]])),
               rig.fit_rotation(np.array([up, flex]), np.array([[0, 0, -1.0], [1.0, 0, 0]])))
scale = {"wrist_flex": 2.0, "wrist_dev": 2.0, "sway_ry": 2.0, "mcp_flex": 1.0, "pip_flex": 1.0, "mcp_abd": 1.0, "sway_rx": 1.0, "sway_rz": 1.0}
weight = {"wrist_flex": 1.0, "wrist_dev": 0.4, "sway_ry": 0.4, "mcp_flex": 0.5, "pip_flex": 0.5, "mcp_abd": 0.3,  # MCP and PIP equal: one fitted group
          "sway_rx": 20.0, "sway_rz": 10.0}
true_tremor = {j: (o := v5.oscillator(j, scale[j], lit)) | {"torque_density": np.sqrt(weight[j]) * v5.REFERENCE_DENSITY_PER_STIFFNESS * o["stiffness"]}
               for j in v5.FIT_JOINTS}
true_support = 0.3
true_central = {"name": "central", "frequency_hz": 11.5, "bandwidth_hz": 1.0, "harmonics": [1.0], "supported": True,
                "gain": {"wrist_flex": 2.0 * true_tremor["wrist_flex"]["torque_density"], "wrist_dev": 0.0, "sway_ry": 0.0}}
caps = [v5.simulate(v5.scenario(hand, chips, params, v5.timeline_moves(), seed=seed, start_support=true_support,
                                tremor=true_tremor, mounts=true_mounts, drives=[true_central]), W / f"trial-{seed}.toml")
        for seed in (11, 12, 13)]
mounts = v5.fit_mounts(caps)
for name, fitted, true in zip(("palm", "finger"), mounts, true_mounts):
    print(f"{name} mount error {np.degrees(np.arccos(np.clip((np.trace(fitted.T @ true) - 1) / 2, -1, 1))):.2f} deg")
v5.STIFFNESS_SCALES, v5.DRIFT_SCALES = (1.0, 2.0), (1.0, 4.0)
units = v5.unit_runs(hand, chips, params, mounts, W, print)
fc = v5.central_hz(caps, mounts)
print(f"central peak measured {fc:.2f} Hz (true 11.5)")
tremor, drives, support, _ = v5.fit_tremor(units, v5.fit_targets(caps, mounts, "hold"), v5.fit_targets(caps, mounts, "table"), print, fc)
for j in v5.FIT_JOINTS:
    print(f"{j}: density {tremor[j]['torque_density']:.4g} true {true_tremor[j]['torque_density']:.4g}"
          f" ({tremor[j]['torque_density'] / true_tremor[j]['torque_density'] - 1:+.1%})")
print(f"table support {support:.3f} true {true_support}")
d = drives[0]
print(f"central {d['frequency_hz']:.2f} Hz width {d['bandwidth_hz']:.2f} (true 11.5, 1.0); gain wrist_flex "
      f"{d['gain']['wrist_flex']:.4g} true {true_central['gain']['wrist_flex']:.4g}, wrist_dev {d['gain']['wrist_dev']:.3g}, sway_ry {d['gain']['sway_ry']:.3g}")

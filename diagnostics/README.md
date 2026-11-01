# Diagnostics

One-off checks behind findings in `RESEARCH.md`, kept so each number there can be regenerated.
They read the captures through the manifest and print; none of them writes scores. Run them from
the repo root with `uv run python diagnostics/<script>.py`, after `timusim-reproduce` where they
read fitted parameters.

| Script | Finding |
|---|---|
| `long_holds.py` | 2026-10-28: long-tau Allan deviation, zero level and temperature of every long hold, rig against bench against the fitted model; writes `data/diagnostics/long_holds.json` |
| `drift_check.py` | 2026-10-28: whether the real long holds sit inside the spread of the model's own long-tau Allan estimates (needs `long_holds.json`) |
| `drift_temp.py` | 2026-10-28: accel drift in each long hold against temperature |
| `drift_tilt.py` | 2026-10-28: tilt or bias, the finger breakout tilting on its tape in `V1-long-s5`..`s7` |
| `v4_sweep_gravity.py` | 2026-10-28: gravity at every V4 session-1 sweep rest, the arm rocking out of plane and sinking |
| `palm_z.py` | 2026-10-29: palm LSM6DSL accel z excess noise in one capture |
| `palm_z_chunks.py` | 2026-10-29: the same per 5-min chunk, the board settling on a soft support |
| `v3_wrap.py` | 2026-10-30: the V3 session-2 accel x- events the old rule counted as wraps |
| `v5_fit_recovery.py` | 2026-10-31: the V5 session-1 fit on simulated trials, against the known mounts, tremor and table support |

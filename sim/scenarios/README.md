# Scenario and parameter files

Both are TOML, read by `sim/scenario.hpp`. Paths in a scenario are relative to its file.

## Scenario

| Key | Meaning |
| --- | --- |
| `name`, `seed` | header name; seed of the noise and tremor draws |
| `[realism]` `chip_transfer`, `noise`, `tremor` | layers, each on top of the previous; all default on |
| `[geometry]` `palm_sensor`, `mcp`, `finger_sensor` | positions, m; required |
| `[geometry]` `proximal_m`, `middle_m`, `dip_per_pip` | segment lengths and DIP coupling; required |
| `[geometry]` `palm_mount`, `finger_mount` | `{ axis = [x, y, z], deg = a }`, sensor to segment |
| `[geometry]` `twist_at_90_deg` | finger twist at 90° flexion |
| `[geometry]` `finger_segment` | `"distal"` (default) or `"middle"`; `finger_sensor` is from the DIP or the PIP |
| `[palm]`, `[finger]` `chip` | chip config, below |
| `[palm]`, `[finger]` `params` | parameter file; absent is an ideal chip |
| `[start]`, `[[moves]]` | poses: `palm`, `wrist`, `joints_deg`, `support`; a move has `hold_s` or `duration_s` and `profile` (`min_jerk`, `cosine`) |
| `tremor_file`, `[tremor]` | joint oscillators per DOF: `inertia`, `stiffness`, `damping`, `torque_density`, `muscle_tau_s`, `muscle_order` |
| `[[drives]]` | shared narrow-band torque: `frequency_hz`, `bandwidth_hz` (half power), `harmonics` amplitudes (default `[1.0]`), `gate` (`support`, the default, scales it with support; `none`; `speed`, by the DOF's scripted speed in rad/s), `gain` table by DOF name, N m/√Hz at the peak |
| `[jolt.palm]`, `[jolt.finger]` | mount jolts: rate added at the sensor while its segment moves, `frequency_hz`, `bandwidth_hz`, `level_dps` (dps/√Hz at the peak per rad/s of segment speed) |
| `geometry.contact_pivot` | point of the hand (palm frame, m) the table holds still while the hand rests on it (support below 1, full contact at the start support); default the wrist; `geometry.finger_contact_pivot`, the finger's own held point (sensor's segment frame from its proximal joint), default none |
| `[[moves]]` `profile = "beta"`, `shape = [a, b]` | velocity ∝ s^(a−1)(1−s)^(b−1); [3, 3] is min-jerk, a < b peaks early |
| `[[masses]]` | `kg`, `at = "palm"` or `"finger"`, optional `offset = [x, y, z]` m from that sensor |

### Chip config

`model = "lsm9ds1"` (default) or `"lsm6dsl"`. Defaults are the capture firmware's.

| Key | LSM9DS1 | LSM6DSL |
| --- | --- | --- |
| `odr_hz` | 119, 238, 476, 952 | 104, 208, 416, 833 |
| `accel_range_g` | 2, 4, 8, 16 | 2, 4, 8, 16 |
| `gyro_range_dps` | 245, 500, 2000 | 125, 250, 500, 1000, 2000 |
| `gyro_bw` | `strong`, `medium`, `light`, `ultra_light` | `lp2_only`, `lp1_strong`, `lp1_normal`, `lp1_light`, `lp1_aggressive` |
| `gyro_out_path` | `lpf1`, `lpf1_lpf2` | none |
| `accel_bw` | `off`, `odr_div_50`, `odr_div_100`, `odr_div_9`, `odr_div_400` | `low_latency_odr_div_{50,100,9,400}`, `low_noise_odr_div_{50,100,9,400}` |

## Parameter file

`chip::Params` in `lib/chip/include/chip/model.hpp`; `timusim-v1` writes them.

| Key | Meaning |
| --- | --- |
| `odr_scale`, `oversample` | actual over nominal ODR; internal steps per sample (LSM9DS1 only) |
| `accel_aa`, `accel_lp`, `gyro_lpf1`, `gyro_lpf2` | `{ kind, order }` per stage, `kind` one of `butterworth`, `one_pole`, `moving_average` |
| `[gyro_lpf1_by_odr]` | `"<ODR>" = { kind, order, cutoff_scale }`, or an array of three for x, y, z |
| `[gyro_lpf2_by_bw]` | `"<register value>" = { kind, order }` |
| `gyro_offset`, `accel_offset`, `temperature_c` | zero levels, header temperature |
| `accel_gain`, `gyro_gain`, `gyro_g_sensitivity` | 3x3 rows |
| `[accel]`, `[gyro]` | `white_density`, `turn_on_sigma`, `markov = [{ sigma, tau_s }, ...]`, `rate_random_walk` |
| `[accel.by_fs]`, `[gyro.by_fs]` | white density per full scale |
| `[gyro.offset_by_fs]`, `[*.drift_scale_by_fs]` | zero-rate level and drift level per full scale |

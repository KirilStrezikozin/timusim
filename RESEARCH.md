# Validation protocol

Fixed before the first capture. Changes after that date are listed at the end with a reason.

## Claim

From a start and an end hand pose, the simulator produces LSM9DS1 accelerometer and gyroscope
streams for a palm sensor and a distal finger sensor whose statistics fall within the tolerances
below of real captures, including the physiological tremor band.

## Setup

- Sensors: two LSM9DS1 breakouts, palm (back of the hand) and finger (distal segment). The LSM6DSL
  on the B-L475E-IOT01A board is the second-chip check: from 2026-10-23 it is modelled and scored
  in V1 by the same fitting procedure, to test that the procedure is not specific to one chip.
- Firmware: `examples/stm32-capture`, full ODR from the FIFO. Default config: 476 Hz, ±4 g,
  ±2000 dps, gyro LPF1 only.
- Added masses, weighed with mounts and wires: finger board 8 g, palm breakout 8 g. The STM32
  board with breadboard and straps weighs 124 g and stays off the hand except in the mass-loading
  check (V5).
- Every capture gets a manifest entry: date, session, test, config, board IDs, firmware hash,
  temperature at start, mount notes.

## Data split

- Bench tests (V1): capture 1 per config fits chip parameters, capture 2 is reported.
- Saturation and jig tests (V3, V4): session 1 fits the mount geometry only; sessions 2 and 3 are
  reported, on different days and power-ups.
- Hand tests (V4, V5): session 1 fits model parameters; sessions 2 and later are reported.
  At least 3 sessions on different days.
- Nothing from a reported capture feeds back into a fitted parameter. A refit after looking at
  test data is logged below and the affected results are marked.

## Metrics

Signals are per sensor and per axis in the IIMU frame, accel in g, gyro in dps. PSDs use Welch's
method, Hann window, 8 s segments, 50% overlap.

### V1. Bench still (chip model, per config)

| Metric | Definition |
| --- | --- |
| bias | mean over the capture, per axis |
| std | standard deviation, per axis |
| noise floor | median PSD between 5 Hz and 0.5 × cutoff, per axis |
| low-frequency PSD | median PSD between 0.05 and 0.5 Hz from 100 s segments, per axis |
| cutoff | frequency where the PSD falls 3 dB below the noise floor |
| Allan deviation | overlapping, at τ = 0.01, 0.1, 1, 10, 60 s, per axis |
| temperature | mean and drift over the capture |

### V2. Static geometry (accelerometer)

| Metric | Definition |
| --- | --- |
| tilt error | angle between the mean accel of a hold and the predicted gravity direction |
| norm error | \|mean accel\| − 1 g per hold |

### V3. Saturation

| Metric | Definition |
| --- | --- |
| clip level | output code while clipped, against full scale |
| wrap | any sign flip while the input stays past full scale |
| recovery | samples from leaving full scale to a non-clipped reading |

Procedure (fixed 2026-10-28, before any V3 capture):

- Rig: the V4 rig as it is, base flat, arm free on its pivot. The finger LSM9DS1, the LSM6DSL
  board and the second LSM9DS1 (`palm_lsm9`) are screwed to the arm (18.5, 15 and 15 cm from the
  pivot), so all three see one rigid-body rotation about the hinge axis.
- Configs, one capture each per session:
  - `v3`: finger LSM9DS1 at ±2 g and ±245 dps (narrow), LSM6DSL at ±16 g and ±2000 dps
    (reference).
  - `v3_board`: the roles swapped, LSM6DSL at ±2 g and ±250 dps (its lowest gyro range),
    finger LSM9DS1 at ±16 g and ±2000 dps.
  `palm_lsm9` is narrow (±2 g, ±245 dps) in both and scored as a second narrow sensor against
  the same reference. Other settings (ODR, filters) as in the default config.
- Motion, about 2 min per capture: 10 s still, then 10 hard pushes A→B and 10 B→A, each fast
  enough to pass ±245 dps and ending on the stop (the impact passes ±2 g), each followed by at
  least 2 s still.
- The arm has one degree of freedom, so only the gyro axis along the hinge and the accel axes in
  the arm plane can clip. Axes without events are reported as having none and are not scored.
- Sessions: the mount rotations come from V4 session 1 (same rig, same day as V3 session 1);
  V3 session 1 fits only each narrow sensor's delay relative to the reference
  (cross-correlation of the gyros over unclipped samples). Sessions 2 and 3 are reported, on
  different days and at different power-ups. The rig is not changed between sessions.
- An event is a run of samples at the narrow sensor's largest or smallest code on one axis; runs
  less than 0.5 s apart merge. Events where the reference sensor also clips are dropped.
- Sim input: the reference gyro mapped through the mount rotation, shifted by the relative
  delay, resampled to the narrow sensor's measured ODR, and fed through the narrow sensor's chip
  model (filter, noise, quantization, clipping). For accel, the reference accel is moved to the
  narrow sensor's position with the rigid-body term α×Δr + ω×(ω×Δr) (Δr from the measured arm
  positions, ω and α from the reference gyro).
- Scored: clip level and wrap for accel and gyro; recovery for the gyro only, where the input
  leaves full scale when the mapped reference gyro falls back inside ±245 dps. Accel recovery is
  reported, not scored: the reference differs from the narrow sensor's input by its own filter
  and the lever-arm estimate.
- A capture is redone only for a reason seen without its readings, as in V2, or for missing
  samples.

### V4. Hinge jig

| Metric | Definition |
| --- | --- |
| closure error | ∫ω of the relative gyro over a 0→θ→0 sweep, per sweep |
| gyro scale | integrated angle between the stops, against the measured stop angle θ |
| lever-arm residual | RMS of radial accel − ω²r over each sweep |
| mount agreement | RMS of an arm sensor's gyro − `palm_lsm9` gyro mapped through the mount, with the link held at a stop |
| static decomposition | tilt error (V2) of both boards at both stops, base flat and tilted |

Procedure (fixed 2026-10-28, before any V4 capture):

- Rig: a base screwed to a rigid plank, with a pivot and an arm turning in one plane between two
  hard stops. θ by phone, measured three times on 2026-10-28 before session 1: 170.6°, 171.6°,
  170.8°; mean 171.0°, SD 0.5°. That is the angle at first contact; at rest the arm can sink
  further into a stop under its own weight, so the rest angle θ_rest is measured per base
  orientation and used wherever the arm is at rest. On the arm, from the pivot axis (ruler, pivot centre to chip
  centre, ±1 mm): the LSM6DSL board at 15 cm, the second LSM9DS1 (`palm_lsm9`) at 15 cm and the
  finger LSM9DS1 at 18.5 cm. No sensor on the base: the base is screwed down and still, so the
  arm gyro is the relative gyro. A thin hard pad (tape) on each stop; no foam.
- Firmware: default config, three sensors.
- Base orientations, phone-measured: flat (hinge axis vertical), tilted about 30° about an axis
  in the base plane, and on edge (hinge axis horizontal).
- Captures per session, per orientation:
  1. static at stop A: 90 s, arm pressed onto the stop by its own weight or by tape;
  2. static at stop B: 90 s;
  3. sweeps: 10 sweeps A→B→A by hand, pushing the arm near the pivot and slowing it before each
     stop, at least 3 s still at A between sweeps.
  Once per session, base flat: 4. base rotation: the arm taped at stop A, the whole plank turned
  by hand about varied axes for 60 s.
- Sessions: session 1 fits the geometry only: each sensor's mount rotation, the hinge axis in
  each sensor frame (from the sweeps' gyro direction) and the arm direction. Sessions 2 and 3 are
  reported, on different days and at different power-ups. Radii and θ are measured, not
  fitted. If the rig is rebuilt or a board remounted, session 1 is recorded again.
- Sim input: the hinge angle per sweep, reconstructed from the integrated arm gyro
  (`palm_lsm9`) and rescaled so each rest at A reads 0 and each rest at B reads θ_rest; in the
  base-rotation capture, `palm_lsm9`'s measured ω drives the arm. The sim then runs the same geometry through the
  chip models.
- Scoring:
  - closure error: per sweep, from the rest at A before it to the rest at A after it.
  - gyro scale: the integrated angle from rest at A to rest at B, per half sweep, compared in
    degrees. The real ratio to θ_rest is reported too, with the phone's uncertainty.
  - lever-arm residual: base flat only, where gravity has no radial component; per sweep and per
    arm sensor, excluding samples from each stop contact until |ω| stays under 5 dps for 0.2 s,
    and any clipped sample.
  - mount agreement: from the base-rotation capture, for the finger and the LSM6DSL board each
    against `palm_lsm9`.
  - static decomposition: tilt error of each sensor at both stops in all three orientations,
    with up predicted from the measured base tilt, θ_rest and the session-1 mounts; first 30 s
    of each hold dropped.
- A capture is redone only for a reason seen without its readings, as in V2, or for missing
  samples.

### V5. Hand statistics

Phases: 60 s on the table, 60 s free hold, then the scripted motion routine.

| Metric | Definition |
| --- | --- |
| band power | gyro PSD summed over axes, integrated over 0.1–3, 3–8, 8–13, 13–25 and 25–100 Hz |
| dominant frequency | peak of the summed gyro PSD between 3 and 25 Hz |
| gyro std | standard deviation of \|ω\| per phase |
| median peak | median over 1 s windows of max \|ω\| |
| accel noise | standard deviation of \|a\| on the table |
| linear acceleration | 95th percentile of \|\|a\| − 1 g\| per phase |
| handedness | correlation from `imu/conformance.hpp` HandednessCheck over the routine |

Mass-loading check: the free hold repeated with the 124 g STM32 board strapped to the back of the
hand. Metric: shift of the dominant frequency between the two conditions. The sim gets only the
added mass.

## Acceptance

A sim statistic passes when it lies within

    real mean ± max(tolerance × |real mean|, 2 × real SD)

where the real mean and SD are over the reported captures or sessions. Band powers compare in
dB, and so do the noise floor and low-frequency PSD. Tolerances (confirmed 2026-10-20; the author may adjust them, and every change is logged
under Changes with its date and whether test data had been seen):

| Metric | Tolerance |
| --- | --- |
| bias, std, noise floor, low-frequency PSD | 20% |
| cutoff | 10% |
| Allan deviation | 25% per τ |
| tilt error, closure error | 1° absolute |
| norm error | 10 mg absolute |
| band power | 3 dB |
| dominant frequency | 1 Hz absolute |
| gyro std, median peak, linear acceleration | 25% |
| mass-loading shift | sign must match, size within 50% |
| clip level, wrap | exact: same code, same sign behaviour |
| recovery | 2 samples absolute |
| gyro scale | 1° absolute on the integrated angle |
| lever-arm residual, mount agreement | 25% |

Each rung of the ladder (ideal kinematics, + chip transfer, + noise and bias, + tremor and sway,
+ wrist pivot and twist) is scored on every metric; the ladder table reports pass counts per rung.

## Changes

- 2026-10-20: tolerances confirmed as listed.
- 2026-10-21, V1, after session 2 was scored (test data seen): gyro LPF1 shape and cutoff factor
  on Table 46 fitted per ODR, and gyro zero level fitted per full scale, both from session-1
  captures only. A capture missing under 1 s of samples (0.1% for 2-hour holds) is used as its
  longest gap-free stretch. Sessions 2 and later are all reported, not only capture 2. V1 results
  from 2026-10-21 on are marked as refit after test data.
- 2026-10-22, V1, after sessions 2-3 were scored (test data seen): drift changed from one
  Gauss-Markov term to a sum of terms on a half-decade grid plus rate random walk, with a drift
  level per full scale. Fitted on the 2-hour holds (fit-only, never reported) and session-1
  captures.
- 2026-10-22, V1: low-frequency PSD metric added to test the bias instability in the spectrum
  as well as in the Allan deviation; 20% tolerance proposed by Claude, adjustable by the author.
  Accel cutoff tolerance kept at 10% for now (author).
- 2026-10-23, V1, before any LSM6DSL capture was scored (no test data seen): the LSM6DSL is
  modelled on the same signal path and fitted and scored by the same V1 procedure, unchanged.
  Its cutoffs come from AN5040 Tables 8, 9 and 11. Two stages are lumped because the fit cannot
  separate them: the accel analog filter with digital LPF1, and gyro LPF1 with the fixed LPF2,
  each at the lower or overall cutoff. The internal rate is taken as 6.66 kHz, the top ODR, at
  which LPF2 is bypassed. Palm sweep configs (`palm_*`) change one LSM6DSL setting with the
  LSM9DS1 at default, so each counts as a default capture for the other sensor. Captures record
  the board power-up (`power_up`) for the turn-on bias; the 2026-10-21 and 2026-10-22 nights are
  power-ups 1 and 2.
- 2026-10-23, V1, after zero levels of reported captures across power-ups 1-3 were looked at
  (test data seen): the gyro turn-on sigma is the spread of the zero level across power-ups,
  one 2-hour hold standing for each power-up, instead of the spread across session-1 captures
  of one power-up, which is in-run drift the Markov terms already carry.
- 2026-10-23, V2, before any V2 capture (no test data seen): one capture per hold, the first
  30 s dropped as settling and the next 60 s averaged. Session 1 fits the accel zero level and
  3x3 gain by least squares over at least six holds; later sessions, each re-taped, are
  reported. Tilt error is scored per position with the 1° tolerance; the norm error is reported
  without a tolerance until the author sets one.
- 2026-10-24, V1, after the 2026-10-23 scores (test data seen), agreed with the author: the drift
  terms are fitted to the 0.02-2 Hz spectrum of the long holds as well as their Allan curves,
  because the sim's accel low-frequency PSD came out 1.2-1.6 dB high on both chips; the gyro zero
  level is the mean over power-ups instead of the session-1 median; the gyro LPF1 shape and
  cutoff factor are fitted per axis, because the palm gyro cutoff differs by axis.
- 2026-10-24, V1, after the rescore with the change above (test data seen): the drift fit holds
  white noise at the output level below the filters, the fitted density with the aliased internal
  band and quantization, instead of the density at the filter input. For the accel path the two
  differ by about 1 dB, which the Markov terms had taken up and the sim then added again on top
  of its own aliased noise. Residuals are relative to the model, refitted from the measured
  values, because relative to a noisy measurement they bias the fit low. The Markov grid
  reaches 3162 s for the 345-minute hold of power-up 4.
- 2026-10-24, comparison, no change to timusim's procedure: IMUSim (eeetem/imusim, Python 3
  port) and WIMUSim are scored on the same V1 reported captures with the same rule. Each gets
  what its model can express, from that config's session-1 captures: measured ODR, gyro zero
  level, per-axis standard deviation and, for IMUSim, a 16-bit ADC at the chip LSB.
- 2026-10-24, V2, before any V2 capture (no test data seen): the norm error is scored per
  position with a 10 mg tolerance, set by the author (5-10 mg; the upper end, since each later
  session is re-taped). V1 runs three simulations per reported capture, so the gyro turn-on draw
  is not shared by every config's single simulation.
- 2026-10-25, V1, before the night-5 captures: sweep captures may change one setting on each
  sensor at once (config `<finger>+palm_<palm>`), so both sensors get a reported session from
  one capture. The sensors sit on separate I2C buses; no pair exceeds the serial rate already
  used by `952hz` with the palm at 416 Hz.
- 2026-10-25, V2, during session 2 (no V2 score computed yet): a reported hold is redone only
  for a reason seen without its readings (a knock, an alarm, tape slipping, the board held in
  reset, the phone level showing it off); the redone hold stays in the manifest as `excluded`
  with that reason. Session-1 holds may also be excluded from their readings (a lean of about 1
  degree or more against the other placements), since they only feed the fit. Each sensor's
  `up` is the axis its reading shows near +1 g; the STM32 board's printed axes disagree with the
  LSM6DSL in x and z, and labels given from them were corrected this way.
- 2026-10-25, V2, after session 2 and before any V2 score: V2 is reported over sessions 2 and
  3; session 3 is recorded on another day, at another power-up, with every position re-taped,
  whatever the session-2 scores turn out to be.
- 2026-10-25, comparison, after the V2 session-2 scores (test data seen), no change to timusim's
  procedure: gnss-ins-sim (Aceinna) joins the V1 comparison under the same rule. Its model has
  one first-order Gauss-Markov drift term, so it also gets one per axis, fitted with white noise
  to that config's session-1 Allan curves. The comparison extends to V2: noise and rate come from
  the session-1 holds; IMUSim, which has a gain matrix, gets the zero level and 3x3 gain fitted as
  timusim's are; WIMUSim and gnss-ins-sim, which have none, get the zero level alone (the mean
  of reading minus up over those holds). Each reported hold is simulated three times at its
  declared orientation and scored with the V2 rule.
- 2026-10-25, comparison, before any MATLAB score, no change to timusim's procedure: MATLAB's
  imuSensor (R2026b, Navigation and Sensor Fusion and Tracking Toolboxes) joins both
  comparisons. Its model has a gain matrix, quantization and saturation, white noise, a filtered
  bias instability and rate random walk, so it gets the chip LSB and range, the V2 zero level and
  3x3 gain as IMUSim does, and in V1 a 1/f bias instability (the default fractalcoef filter, with
  enough poles to stay 1/f out to the longest Allan tau) and a rate random walk, fitted with
  white noise to each config's session-1 Allan curves.
- 2026-10-28, V3 and V4, before any V3 or V4 capture (no test data seen): procedures written
  under V3 and V4, with tolerances for clip level, wrap, recovery, gyro scale, lever-arm residual
  and mount agreement, which were missing. V3 drives the sim with a wide-range reference sensor on
  the same rigid arm, so gyro recovery can be scored; accel recovery is reported only. V4 uses the
  arm rig the author built: stops 168° apart instead of 90° (the metrics use the measured angle),
  both arm sensors on one link at 15 and 18.5 cm, and the second LSM9DS1 on the base. A third
  base orientation, on edge with the hinge axis horizontal, is added because with the base flat
  gravity reads the same at both stops. The lever-arm residual is scored with the base flat
  only, where gravity has no radial component, and drops the settling after each stop contact
  (the arm rings when released).
- 2026-10-28, V3 and V4, before any V3 or V4 capture (no test data seen): the second LSM9DS1
  (`palm_lsm9`) is mounted on the arm at 15 cm, not on the base, so the rig serves V3 and V4
  without rebuilding. With the base screwed down the arm gyro is the relative gyro; mount
  agreement compares the finger and the LSM6DSL board against `palm_lsm9`, which also drives the
  arm in the base-rotation sim. In V3 `palm_lsm9` is narrow in both configs and scored
  against the same reference as the other narrow sensor, so each session scores two LSM9DS1
  units, against the LSM6DSL in `v3` and against the finger LSM9DS1 in `v3_board`. The LSM6DSL's narrow gyro range in `v3_board` is ±250 dps, its lowest, not ±245 dps.
  θ measured: mean 171.0°, SD 0.5° (three phone readings), replacing the earlier 168°.
- 2026-10-28, V3, after a first session-1 capture whose readings were not looked at (excluded in
  the manifest, motion not possible on the rig): the rig is the V4 arm, a plate pivoting on a
  box, with one degree of freedom, so the three-axis flicks and taps cannot be done. V3 motion is
  hard pushes of the free arm into the stops; only the hinge gyro axis and the in-plane accel
  axes clip, and the rest are reported without events. Mount rotations come from V4 session 1,
  which turns the whole box by hand; V3 session 1 fits only the delays.
- 2026-10-28, V3 session 1 (fit only, nothing reported): two `v3` captures were excluded because
  the board reset on hard stop impacts; after the wires were secured, a 75 s reset test (28 hard
  pushes about 2 s apart, without the 2 s rests) is used as the `v3` session-1 capture, since
  session 1 only fits the delays.
- 2026-10-28, V4, during session 1 (no V4 score computed; session-1 holds looked at for
  stillness): θ = 171.0° is the angle at first contact. With the hinge horizontal the arm rests
  about 5° further into the stop under its own weight (the author's measurement), θ_rest = 176°;
  the gravity angle between the session-1 holds, 175.7-177.7° across the three sensors, agrees
  but is not used. θ_rest is measured per base orientation and replaces θ in the static
  prediction and in the rescaling of the reconstructed hinge angle; gyro scale reports its real
  ratio to θ_rest. The procedure's orientations are named by the hinge: with the box on its base
  the hinge is horizontal (the "on edge" case), and the box on its side gives the hinge-vertical
  "flat" case.
- 2026-10-28, V4, after session 1 and before any reported capture (no V4 score computed): the
  arm comes to rest at a stop a few degrees differently from sweep to sweep (up to about 5° at A
  in the session-1 sweeps). Where gravity has a component in the arm plane (hinge horizontal and
  tilted), each rest's hinge angle is taken from its own gravity reading, against the session-1
  holds and hinge axis, and the reconstructed angle is rescaled between those values; with the
  hinge vertical, rests at A read 0 and rests at B read θ_rest = 171° (the author's measurement:
  the arm does not sink there). The sweeps' first-contact angle is 171° in every orientation.
- 2026-10-28, V3 and V4, after session 1 and before any reported capture (V3/V4 code run on
  session 1 against itself to exercise it; no reported data exists): definitions as implemented
  in `v3.py`, `v4.py` and `rig.py`.
  - V3: a sample is clipped at or past 99% of the largest code (32440); full scale for the input
    is 32768 LSB. Clip level is the code the output sits on: the most frequent clipped code of
    the event, or its extreme when no code repeats (accel impact spikes). The LSM6DSL gyro sits
    on 32736, below its largest code, so a run "at the largest code" would miss it. Wrap and
    recovery use the reference's estimate of the input; recovery runs from that estimate falling
    back inside full scale to the first output sample below the clip threshold. An axis and sign
    with events in only the real or only the simulated captures is reported as event counts, not
    scored.
  - V4: rests are stretches of at least 0.5 s with every gyro axis within 3 dps of its median.
    Rest angles from gravity are taken against that session's hold at A (not session 1's, since
    each session re-places the box). Static decomposition takes up at A from `palm_lsm9`'s own
    hold reading and predicts the other sensors through the mounts and stop B by turning it by
    θ_rest about the hinge axis; `palm_lsm9` at B is reported, not scored, because that unit has
    no V2 zero level and gain of its own. Gyro scale is scored per direction (A→B, B→A). A stop
    contact is the first sample within 3° of the arriving rest's angle. Mount agreement uses
    samples above 10 dps. The base-rotation sim is driven by `palm_lsm9`'s measured rate and
    accel mapped through the mounts. Chip parameters: V2's for the finger and the LSM6DSL, the
    finger's V1 LSM9DS1 parameters for `palm_lsm9`, each at its own measured ODR.
- 2026-10-28, V4, found in the session-1 check run, no change made (for the author to decide
  before session 2):
  - The arm rocks out of its plane at the stops: gravity's angle to the fitted hinge axis
    changes between the holds at A and B by about 9-10° with the hinge horizontal, 2-7° tilted
    and 1.5° vertical, which a rotation about one fixed axis cannot do (play in the pivot). A
    pure-hinge prediction misses the holds at B by 8-14° on every sensor, so static
    decomposition cannot pass as defined.
  - Against the LSM6DSL, both LSM9DS1 units read the hinge rate 2% lower for one direction of
    rotation than the other (gain about 0.98 and 0.96 relative, in all three sweep captures).
    The chip model has one gain per axis, so LSM9DS1 gyro scale and closure will fail.
  - With the hinge vertical the LSM6DSL integrates about 166° between the sweep rests, against
    θ_rest = 171° (phone, first contact). The radius fitted from the hinge-vertical sweeps is
    about 3 cm shorter than the ruler for all three sensors.
- 2026-10-28, V1rig, before any of its captures: a new bench test on the V4 rig (night 7,
  power-up 8, arm taped at A). `palm_lsm9`, the second LSM9DS1, runs every V1 config for 10 min
  with the finger and the LSM6DSL at default, then all three hold the default config in 240-min
  segments. Purpose: whether the LSM9DS1 parameters fitted on the finger unit carry over to
  another unit without refitting. `palm_lsm9`'s captures are scored against the finger's V1
  parameters (bench session 1) at `palm_lsm9`'s measured ODR, with the V1 rule; nothing is
  fitted on `palm_lsm9`. The finger's own rig captures, scored the same way, separate the rig's
  environment from the unit. The pool is separate from V1 (test `V1rig`), so bench V1 results
  are unchanged.
- 2026-10-28, V1rig result (night 7, power-up 8). `palm_lsm9`'s 11 V1 configs scored against
  the finger's V1 LSM9DS1 parameters: 499/660 metrics pass. The finger's 11 captures on the same
  night, scored the same way (default config): 59/60 pass, the one fail being accel z noise floor
  1.5 dB under the real one. So the rig environment barely changes the finger's results, and the
  failures belong to the unit. Most common fails across the 11 configs: gyro bias 33 (every
  config and axis; a unit's zero-rate offset is its own), accel Allan deviation at 60 s 23, gyro
  noise floor and low-frequency level 19 each (`palm_lsm9` about 2 dB noisier than the model), gyro
  Allan deviation at 60 s 14, accel low-frequency level 11. Each `palm_lsm9` config has one capture,
  so its window is the tolerance alone with no 2 SD term, a stricter test than the finger's
  11-capture default. Holds `V1rig-long-s1-054430` and `-s2-` (240 min, all three sensors) are
  recorded and not yet analysed; the 92 s third hold is excluded.
- 2026-10-28, V4, the author's decisions on the session-1 findings, before session 2 (only
  session 1 seen, as a check of the code):
  - Pivot play is a limit of the rig; it cannot be tightened. Static decomposition predicts stop
    B as before and then turns it by session 1's measured rocking at B for that orientation (the
    turn, in `palm_lsm9`'s frame, from the hinge-only prediction to session 1's hold at B: 9.9°
    hinge horizontal, 10.1° tilted, 8.4° hinge vertical). Sessions 2 and 3 then test whether the
    rocking repeats.
  - The LSM9DS1 gyro is allowed a gain per turning direction. The chip model gets
    `gyro_negative_gain` (per axis, on negative rates, before `gyro_gain`). Session 1 fits, per
    sensor, the gain on the axis nearest the hinge for A→B and for B→A: integrated angle over the
    rest-to-rest angle from gravity in the hinge-horizontal sweeps, where gravity is normal to the
    hinge. Result: LSM6DSL 1.0070 / 1.0050, finger 0.9858 / 0.9568, `palm_lsm9` 0.9922 / 0.9640.
    The tilted sweeps are not used: there gravity is 34° from the hinge, the rocking moves its
    projected angle, and the LSM6DSL ratio comes out 1.6% off the hinge-horizontal one.
  - Sweep rests are not at the stops: the arm rebounds and rests 165.4° apart (gravity, hinge
    horizontal, SD 0.7°). With the hinge vertical gravity cannot place them, so they are taken
    as the mean of the author's first measurement (168°) and the LSM6DSL's integrated span
    corrected by its gain (164.8°): 166.4°. The LSM6DSL's hinge-vertical gyro scale is then
    reported, not scored, since its own reading set the reference.
  - The radius is the ruler's (±0.5 cm). The fit was short because the hinge-vertical base is a
    few degrees off vertical, which leaves 0.07-0.08 g of gravity in the arm's plane, turning with
    the arm. With that gravity removed and stop contacts left out (|α| ≥ 10 rad/s²), the fit gives
    palm 14.6, finger 17.8, `palm_lsm9` 13.1 cm. Only its direction is used.
  - Hinge-vertical rests are labelled A or B by the integrated rate (B is the stop reached
    turning positive), not by assuming the capture starts at A.
- 2026-10-28, V4, static decomposition changed before session 2 (author's suggestion; only
  session 1 seen): each sensor is its own reference. Up at B is predicted from that sensor's
  hold at A in the same session, corrected by its V2 zero level and gain, turned by θ_rest about
  its own hinge axis and then by its own session-1 rocking at B. Only B is scored (A is the
  reference). Mounts drop out of the static score; mount agreement still tests them. The
  `palm_lsm9` LSM9DS1 was fitted to the rig after V2, so it has no zero level and gain of its own
  and its static score stays reported only. Session-1 rocking at B per sensor (palm, finger,
  `palm_lsm9`): hinge horizontal 9.6, 9.3, 9.9°; tilted 10.0, 8.1, 10.1°; hinge vertical 6.9,
  5.1, 8.4°. Before this, a single `palm_lsm9` reference put that unit's 1-4° accel error on every
  sensor, while palm and finger (both V2-corrected) agree on gravity within 0.5° across the six
  holds.
- 2026-10-28, V4, after the session-1 gravity at every sweep rest was looked at (session 1 only):
  - The three sensors' gravity moves together rest to rest (rigid on the arm); the arm itself
    changes attitude. With the hinge vertical, gravity's angle to the hinge at A grows about 2° over
    12 sweeps (the loose arm sinks, as the author saw). The 8-10° rocking between A and B appears
    only in the holds, where the arm presses into the stop under its weight; between sweep rests
    at A and B it is under 1° with the hinge horizontal.
  - The sweep replay therefore takes up from each sensor's own gravity at every sweep rest
    (corrected by its V2 zero level and gain), turned about the hinge between rests, with the
    out-of-plane change from one rest to the next spread over the turn. It took up from the hold
    at A before, which with the hinge vertical put about twice the real in-plane gravity into the
    sim and doubled its lever-arm residual.
  - Tilted, the sweep rests are placed like the hinge-vertical ones (the author's choice, to be
    kept only if it helped): the mean of 168° and the LSM6DSL's gain-corrected span, 166.6°. The
    LSM6DSL's tilted gyro scale is then reported only. With gravity, all three sensors read 2.6°
    more than the sim A→B; with this reference they pass.
  - Mount agreement in the base rotation stays failing (palm 1.92 dps real against 0.68 sim,
    finger 0.59 against 0.39). The sensors are rigid, so this is not the loose arm: allowing any
    3x3 linear map between palm and `palm_lsm9` instead of a rotation brings palm to 0.72 dps.
    The chips differ in gain and cross-axis on all three gyro axes, and the models only have the
    hinge axis's gain, the only axis the rig gives a reference for.
- 2026-10-28, V1rig long holds (night 7, both 240-min holds), compared with the nine bench long
  holds and each chip's drift model (`palm_lsm9` against the finger's), as Allan deviation over
  the model at τ = 1, 10, 100, 1000 s. Each hold is used as its longest gap-free stretch, as on
  the bench: 2.4 h and 2.0 h of the 4 h (the dropped frames split them; the bench 4-hour holds of
  power-up 5 were 2.8 h and 2.1 h the same way).
  - At τ = 1 s the rig is noisier than the bench and the model on accel for all three sensors
    (1.2-2.5x, bench 1.0x) and on the palm gyro x and y (1.5-1.7x); the finger gyro is unchanged
    (1.0x). The bench boards were taped to the table; on the rig they sit on a loose arm on a box,
    which picks up vibration.
  - At τ ≥ 100 s the finger's accel stays under the model on the rig (0.3-0.5x) as on the bench
    (0.4-1.1x): the model's accel drift is high for this unit, rig or not. The palm accel z drifts
    3.5-7.5x the model on the rig; on the bench the same axis has single holds up to 15x.
  - Gyro zero levels on the rig are within the bench spread for palm and finger (finger +1.48 to
    +1.56, +1.10 to +1.14, -0.43 to -0.40 dps against bench +1.32 to +1.51, +0.98 to +1.24, -0.48
    to -0.35), so the rig does not move them. `palm_lsm9` sits at +5.7 to +5.9, +2.6, -2.3 dps:
    its own zero-rate offset, the V1rig gyro bias failure. Its gyro x drifts 1.2-2.2x the finger
    model; y and z 1.0-1.3x.
- 2026-10-28, V1 accel drift model, a finding (test data seen; no change made, for the author to
  decide): the finger's accel drift is a random walk (6-12 µg/√s) with no Markov term past 10 s,
  and real holds do not look like it. Simulated at each hold's length, the model puts 8-9 of the
  11 holds at its 0th percentile at τ = 100 s on y and z (real about 30 µg, model 60-70 µg), while
  at τ = 1000 s the power-up 3-5 holds sit at the 88th-100th percentile and the rest at the 0th.
  Temperature does not explain it (0.2-0.5 °C per hold, weak correlations). Splitting each hold's
  100-s means into direction and norm does: in the finger's holds of power-ups 4 and 5 (`V1-long-s5`,
  `-s6`, `-s7`) the components move 3.7-6.8 mg while the norm moves 0.3-0.9 mg, the reading
  turning 0.3-0.5° slowly over hours (100-s steps under 0.05°): the finger breakout tilted on its
  tape, and the palm beside it did not (0.03°). In `V1-long-s8` (power-up 7) both boards turn
  0.12° within the same 100 s at 138 min (knocked or moved together). In every other hold, rig
  ones included, the direction moves 0.07° or less and the norm swings as much as the components:
  that is the chip's own drift. The finger's random walk was fitted to the tilting, and the low
  finger accel `drift_scale_by_fs` factors (0.32-0.77) likely compensate for an inflated default.
- 2026-10-28, V1, change after test data was seen (author's decision: an observation of a
  problem in the fit, made whatever it does to the scores): a long hold's accel is left out of
  the accel drift fit for a sensor whose board turned during it, recorded in the manifest as
  `accel_tilted` with the measured turn; its gyro stays in the gyro drift fit. Marked: the finger
  in `V1-long-s5`, `-s6`, `-s7`; both sensors in `V1-long-s8`. A turn counts when the 100-s mean
  reading moves in direction by well over the 0.07° the untouched holds show, while its norm moves
  far less than its components. V1 is refitted and V1, V2, V1rig, V3 and V4 rescored; the earlier
  parameter files and scores are kept for comparison.
- 2026-10-28, V2 for `palm_lsm9`, procedure, before its V2 score (only the holds' readings seen,
  to label and check placements): the breakout is taken off the V4 arm and taped to the table on
  long wires, the bench way, instead of turning the whole rig box; palm and finger stay on the rig
  and declare no up. Session 1 (8 holds, ±z twice, power-ups 8 and 9 after an accidental unplug)
  fits the zero level and 3x3 gain on the finger's V1 LSM9DS1 parameters (the unit has no V1 of
  its own); session 2 (6 holds, power-up 10) is reported; session 3 follows on a new power-up.
  Labels follow the readings as before; the breakout's printed z is the chip's -z. One session-2
  hold (`222735`) was redone because the author found it taped loosely, the wires likely pulling
  it. Its simulated holds use the finger's sim slot. A 50° hold (`215413`, no up declared) is
  kept for the finding below and is not in the fit.
- 2026-10-28, V2 `palm_lsm9`, a finding before its score (no change made): the z axis is not
  linear. Read sideways (±x, ±y, 4 holds) it sits at +20 to +23 mg; the ±z pairs (2 placements,
  2 power-ups, repeats within 2 mg) put the zero level at +44 mg. A zero level and 3x3 gain leave
  +11 mg on every ±z hold and -11 mg on every sideways one (x and y residuals 1-4 mg), so the
  unit reads ±1 g on z about 22 mg larger than a line through its zero. The 50° hold reads 12-17
  mg off all of a line, a smooth even term and a per-direction gain, so the shape is open. The
  bench finger LSM9DS1 does not show this (V2 12/12). Expected: tilt within 1° (11 mg is 0.6°),
  norm error near the 10 mg tolerance.
- 2026-10-29, V1 refit result and a palm finding (test data seen; no change made, for the author
  to decide): after the `accel_tilted` refit, V1 finger 640/660 (from 634), V2 12/12 both, V1rig
  finger 59/60, `palm_lsm9` 502/660 (from 499), V4 check 32/34; V1 palm 396/408 -> 365/408. The palm
  drop is not the tilt exclusion: the earlier palm fit predates the night-6 long holds (`V1-long-s8`,
  `-s9`, power-up 7), and in both the LSM6DSL accel z is 3.5x the other holds at τ = 1 s (188-207
  against 55-57 µg) and 10-15x at τ = 10 s, other axes normal. The fit absorbs it (a 10 s Markov
  term of 2.9e-4 g on z, the ±2 g drift factor on z 0.11), and 29 of the 31 new palm failures are
  accel z noise and drift on captures without it. The excess is the palm's alone (the finger
  beside it stays at 50-55 µg) and z only, and it comes and goes: absent in every capture through
  the night-5 long holds (power-up 5), present in most V2 holds from 2026-10-25 14:40 on (68-232
  µg at τ = 1 s, a few at 48-60), in all V1rig captures (103-147) and in all of tonight's (137-403).
  It also wanders at τ = 10 s, too much to be motion along z, so it reads as the chip or its
  supply rather than vibration.
- 2026-10-29, V1, change after test data was seen (author's decision, option (a) of the finding
  above): a long hold's accel is also left out of the accel drift fit for a sensor whose accel was
  not in its usual state, recorded in the manifest as `accel_noisy` with the measured excess.
  Marked: the palm in `V1-long-s8` (already out as `accel_tilted`) and `-s9`. The palm model then
  describes the LSM6DSL as it was through power-up 5; the excess is reported, not modelled. Palm V1
  is refitted and V2 palm, V3 and V4 rescored; the earlier files are kept for comparison.
- 2026-10-29, palm LSM6DSL z excess, bench diagnostics (power-ups 11-12; not test data): with the
  STM32 board off the rig, still on a blanket, the finger beside it as reference, 5 min each. Palm
  z at τ = 1 s 222 and 252 µg against x, y 44-51 and the finger 52-54; the excess is below 0.5 Hz
  (321-334 against 62-81 µg/√Hz), white noise above 0.5 Hz is normal on every axis. It is not
  periodic (no line at the 5 s header period, folded over an hour of `V1-long-s9`: 0.03 mg), does
  not follow the gyro (motion) or the chip temperature (r = 0.18 after detrending), and moving the
  USB cable to another laptop port changes nothing. Left as a property of this unit since
  2026-10-25 afternoon; when it started matches the first V2 holds, the first time the board was
  taped and handled, but the cause is not established.
  With a heavy weight on the board (same blanket, 15 min, 5-min windows): palm z at τ = 1 s 138,
  120, 126 µg and 224-259 µg/√Hz at 0.05-0.5 Hz, about half the unweighted excess, still 2-2.5x x
  and y; it does not settle over the 15 min. In the last window the finger and palm x rose too
  (finger 75-101 µg): an outside disturbance, the only one in these captures. The long-hold notes
  give the same setup for `s7` (clean) and `s8` (noisy), so the setup as recorded does not explain
  the onset; that the weight halves it points to the board's mounting or stress over the chip.
  Flat on the hard table, USB cable not pulling, no weight (9 min): palm z at τ = 1 s per minute
  347, 169, 169, 82, 86, 70, 73, 72, 70 µg, the finger 43-67 (one 101) throughout. The excess
  dies away over the first 3 minutes after the board is set down and stays near the clean level
  (52-58) after; on the soft blanket it did not settle in 15 min, weighted or not. The excess is
  the board settling on a soft or loose support (creep felt along z), not the chip's noise: the
  model, fitted without it, describes the chip on a firm support.
- 2026-10-29, V1 palm refit with `accel_noisy` (result): V1 palm 393/408 (365 with `-s9`'s accel
  in the fit, 396 for the 2026-10-25 fit before the night-6 holds); V2 palm 12/12, V4 check 32/34,
  V3 check 32/50, all unchanged. Against the 2026-10-25 fit the remaining differences are gyro y
  Allan at 60 s, now failing on four configs (sim 0.81 against real 0.44-0.60 mdps): the night-6
  holds are still in the gyro drift fit and raise gyro y's slow terms. Not acted on.
- 2026-10-30, V2 `palm_lsm9` session 3 (power-up 13), holds and a finding, before its session-3
  score: on this power-up z reads +34 to +58 mg on every sideways hold (+x, -x, -y), against +20
  to +32 in sessions 1-2, while the ±z pair still averages +44 mg. The ±x holds were first taken
  for leans and redone, partly against a box the author found bent; since z is high on opposite
  sides alike, it is z's zero level with no gravity on it that moved, not a tilt (the ±x pairs
  differ by 8 mg, 0.5°). The 22 mg z "nonlinearity" of sessions 1-2 is then not a fixed curve
  but a power-up-dependent zero level on the sideways holds. Author's decision after these
  readings: the session counts its first +x (`234729`) and -x (`235047`, its earlier exclusion
  withdrawn); the repeats (`235342`, `000632`, `001137`, `002042`) are excluded as repeats.
- 2026-10-30, V2 `palm_lsm9` score over sessions 2 and 3 (`--sims 3`, finger's V1 LSM9DS1
  parameters under the unit's own session-1 zero level and gain): 10/12. Tilt passes in all six
  positions (real 2.1-2.8°, sim 1.9-2.5°; the session-3 z shift widens the windows to 1.0-1.7°);
  norm error fails on ±z (real +52 / -36 mg, sim +41 / -24 mg) from the z zero-level behaviour
  above, which a single zero level and 3x3 gain cannot express. Session 3 changed no verdict.
- 2026-10-30, V4/V3 sessions 2 and 3, rig change before any session-2 score (author's decision):
  the pivot is screwed in for sessions 2 and 3; in session 1 it was not. With it loose the arm's
  out-of-plane seat at stop B varies between placements (gravity A->B at hinge vertical 1.45° in
  session 1, 3.64° in a session-2 trial), screwed in it is 0.39°. Up at B is still predicted with
  session 1's rocking, so the static check may miss by about 1° from the rig change; the author
  may revisit the tolerance after seeing it. `palm_lsm9` is back on the arm after its V2 (its
  hold-A gravity moved 0.67° from session 1 against 0.26-0.30° for palm and finger).
- 2026-10-30, V3/V4, before any session-2 score (author's decision): `palm_lsm9` was re-mounted
  on the arm after its V2 and its mount turned about 2° (the session-2 base rotation `010137`
  puts palm's and finger's mounts relative to it 2.14° and 2.03° from session 1's, while palm
  and finger keep theirs relative to each other). The mount is rig geometry, not a chip
  parameter, so `palm_lsm9`'s mount (and through it the V3 mappings into its frame) is taken
  from the session-2 base rotation for sessions 2 and 3. Its session-2 base-rotation mount score
  is reported, not scored (fitted on that capture); session 3 is predicted with nothing refitted.
  Every other geometry and chip parameter stays from session 1.
- 2026-10-30, V4, before any session-2 score (author's decision): with its own V2 zero level and
  gain (`data/v2-palm_lsm9`), `palm_lsm9`'s static tilt at B is scored like palm's and finger's;
  its rig parameters are its V2 file on the finger's V1 LSM9DS1 parameters.
- 2026-10-30, V4 session 2, first scores and a bug (test data seen): rests labelled from gravity
  used a signed angle that wraps at 180°; with the stops about 180° apart a rest at B read -177°
  and every hinge-horizontal sweep rest was labelled A, so no sweep was scored. Fixed (angle taken
  in [-90°, 270°)); a code error, not a procedure change. Scores then 17/35 (V3 51/82). The
  hinge-horizontal and tilted session-2 captures have the box the other way round from session 1:
  up at hold A reads where session 1's hold B did (3.5° apart) and the turns flip sign. With the
  arm's weight on the other stop and the pivot screwed in, the A->B span is 182.8° (176.4° in
  session 1) and the out-of-plane turn at B -6.0° (+9.5°), so the session-1 rest angle and
  rocking do not carry over: static tilt at B misses by 5.6-15.3° there, 1.0-3.3° hinge vertical.
  V3: 28 of 32 clip-level failures are the model clipping at ±32767/-32768 where the chips sit
  lower (LSM9DS1 gyro ±32763, LSM6DSL gyro ±32736, accel spread over 32468-32764): a model gap.
- 2026-10-30, V3, change after the session-2 scores were seen (author's decision, option (a) of
  three offered): clip level was compared for exact equality, which was not expected to be an
  issue when V3 was written; the chips plateau below the largest code (above). It now passes
  within max(0.5% of full scale = 164 codes, 2 SD of the real events), half the 1% band an
  event's samples are counted in, a tolerance tied to the event definition and not to these
  numbers. Wrap and recovery are unchanged.
- 2026-10-30, V4 session 2 hinge horizontal and tilted redone (power-up 13, pivot screwed in) with
  the box placed as in session 1, checked live against session 1's holds from palm_lsm9 gravity
  (the earlier set of each is excluded: box mirrored / at another heading, found after the
  scores). Hinge horizontal: A 1.9 deg, B 1.4 deg from session 1's (a first B, 4.6 deg off, was held
  at first contact by the screwed pivot and redone with the arm set where session 1's was; a fast
  sweep capture with no pause at B, one B rest usable, was redone at session 1's pace). Tilted:
  A 1.1 deg, B 1.5 deg; session 1's tilted hold B was not at the stop: the arm sits about 11 deg
  short of B there (A->B 158-160 deg about the hinge), and B was taped at that position again.
  V4 session 2 is then 26/35: tilted 10/10, hinge horizontal 9/12 (static tilt at B 3.5-4.1 deg
  real vs 1.0-1.9 sim: the screwed pivot's A->B span is 173.3 deg vs 176.4 in session 1), hinge
  vertical 7/13 (not redone; gyro scale A->B about 2 deg low and lever residual 0.020-0.022 g vs
  0.015 g sim for both LSM9DS1s, static tilt 1-2.5 deg). V3 session 2: 77/82.
- 2026-10-30, V3, change after the session-2 scores were seen (author's decision): a wrap was any
  sign flip of the output while the reference input was past full scale. The 3 flagged session-2
  events (finger x2, palm_lsm9 x1, accel x-) are not wraps: the output falls from about -31000
  toward zero in one sample (21-25k codes, about 1.4 g), touches +65..+8204 for one sample and
  swings back within 2-3 samples, while the reference reads -2.1..-2.8 g; a shock or local ringing
  at the stop impact that the 416 Hz reference on the board does not resolve (none in session 1's
  32 events; the pivot is screwed in since session 2). A wrap is now the output reaching the
  opposite 99% band (the event's own clip threshold) while the input is past full scale, the
  two's complement signature.

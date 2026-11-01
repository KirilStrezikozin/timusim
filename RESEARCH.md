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
  check (V5). V5's palm rig (ST-Link and palm sensor, with straps and wires) weighs 48 g
  (reweighed 2026-11-01).
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

Mass-loading check (restored 2026-11-01 after session 1, see Changes): per session, a capture
with a known weight added on top of the strapped board, table, lift and free hold only. Mechanical
tremor slows with the load as √(K / I); a central oscillation keeps its frequency. Reported per
session: the hold's mechanical peak (3–9.5 Hz) and central peak (9.5–16 Hz) with and without the
weight, real against the fitted model with the same weight added. Not a pass/fail metric.

Procedure (drafted by Claude on 2026-10-31, before any V5 capture; the author confirms or changes
it before session 1, and changes after that are logged under Changes):

- Subject: the author, right hand, index finger. Seated, forearm resting on the table from the
  elbow to the wrist crease, which lies on the table edge, so the hand pivots freely at the wrist.
- Sensors: the STM32 board strapped firmly to the back of the hand, its LSM6DSL (`palm`) over the
  metacarpals, its USB cable slack and taped along the forearm; `finger` on the back of the index
  middle phalanx. `palm_lsm9` is not used.
- Firmware: the `v5` env (default config; palm LSM6DSL and finger, the finger at 0x6B).
- Hand measurements, by ruler (±1 mm) before session 1, into `sim/scenarios/v5/hand.toml`:
  the wrist pivot, taken as the midpoint of the wrist crease; from it, in the palm frame, the
  LSM6DSL chip centre and the index MCP centre; the proximal and middle phalanx lengths; the
  finger chip from the PIP; the DIP-to-PIP coupling (0.7 unless measured); the finger twist at
  90° MCP flexion (0 unless measured). The board as strapped on is weighed, and its centre is
  measured from the LSM6DSL chip.
- Captures: one per trial. `timusim-v5 cue` starts the capture and prints each phase with a
  terminal bell, so the phases sit at fixed times from the first sample:
  1. table, 0–60 s: hand flat on the table, palm down, fingers extended and together, relaxed;
  2. lift, 60–62 s: the hand raised to horizontal;
  3. free hold, 62–122 s: the hand held horizontal past the table edge, palm down, fingers
     extended, the forearm still on the table;
  4. routine, 122–272 s: from the free-hold pose, five moves, five repetitions each, each
     repetition 1 s out, 2 s hold, 1 s back, 2 s hold. The moves are: index curl (MCP 45°,
     PIP 60°); wrist extension 30°; wrist flexion 30°; index abduction 15° towards the thumb;
     and wrist radial deviation 15°. Angles are targets by eye against a protractor card on the
     table, and the sim takes the targets;
  5. the hand back on the table; the capture stops at 280 s.
- Trials: 3 per session, with a minute's rest with the hand on the table between them, then one
  mass-loading capture (`timusim-v5 cue --added-g G`; the weight, weighed, taped on top of the
  board): table, lift and free hold as above, the hand back on the table at 120 s, stopped at
  125 s.
- Sessions: session 1 fits; sessions 2 and 3 are reported, on different days and power-ups.
- Phase windows: table 5–58 s, free hold 67–120 s, routine 122–272 s. A trial is redone if the
  lift (|ω| of `palm` over 30 dps) does not start between 57 and 66 s, for missing samples, or
  for a reason seen without its readings (a move skipped or out of order).
- Session 1 fits, in order, with the chip parameters fixed from V1 and V2:
  1. mounts: each sensor's mount rotation, from its gravity on the table (palm frame z up) and
     its mean gyro axis in the routine's wrist flexion (`palm`) and index curl (`finger`)
     moves, both the flexion axis, as in V4's hinge fit;
  2. tremor: a torque density for each of wrist flexion, wrist deviation, forearm
     pronation-supination (`sway_ry`), finger flexion (MCP and PIP driven at one weight), MCP
     abduction and the two slow drift DOF (`sway_rx`, `sway_rz`), one stiffness scale shared by
     the three wrist and forearm DOF, one shared by the finger joints and one for the drift. Inertia
     is the literature value plus the added masses (the board's weighed mass at its measured
     centre on the palm, 8 g at the finger sensor), and damping is the literature value. The other sway DOF stay off,
     since the forearm is supported. Fitted on the free hold:
     on a grid of the two stiffness scales, 300 s sims with one joint driven alone at unit
     density, and one with no tremor, give band powers that add, since the drives are
     independent and the angles small. The fit compares band powers per axis of the gyro turned
     into the segment frame by the step-1 mounts, so that flexion and abduction, about
     different axes, are told apart. Each axis's power gets 1% of the band's total over the
     three axes added, so that a degree or two of mount error, which leaks about −30 dB of the
     strong axis into the weak ones, does not count. The densities are the non-negative
     least-squares fit of the real band powers in dB. The grid point that minimises
     Σ(ΔdB / 3 dB)² + Σ(Δf / 1 Hz)² over both sensors' band powers and dominant frequencies
     wins. A full sim with the fitted values checks the additivity. On simulated trials
     (2026-10-31, before any V5 capture), the fit returns the mounts within 1.5°, both
     stiffness scales exactly, each density within 7% and the table support within 2%;
     In the same least squares, a central oscillation: one narrow-band drive shared by the wrist
     and forearm DOF, a gain each, through the same muscle filter. Its frequency is the median
     over trials of the free hold's spectral peak between 9.5 and 16 Hz (both sensors, all
     axes); its half-power width (0.2–5 Hz) is fitted;
  3. table support: the tremor drive scale on the table, from the table phase's band powers
     with the fitted densities, by the same least squares;
  4. heartbeat and breathing: two narrow-band drives, not scaled by the support (present on the
     table too). The heartbeat (0.8–2 Hz, its 2nd harmonic at a fitted amplitude) acts on the
     three forearm rotations and, with its own gain, on the finger joints; breathing
     (0.15–0.6 Hz) on the three forearm rotations. Frequencies, widths and gains are fitted on
     the table phase's spectrum at 20 s resolution over 0.15–3.5 Hz, against the spectrum's own
     background (a 0.8 Hz running median), so that the lines and not the broadband level set
     them.
- Sim: each trial's scenario is the hand file, the fitted tremor and the phase timeline, with
  each sensor's chip config and measured ODR from the real capture. Three seeds per real
  capture; the sim value is the mean over all of them.
- Ladder: rung 1, ideal kinematics, has chip transfer, noise and tremor off and no twist;
  rung 2 adds chip transfer; rung 3 noise; rung 4 tremor on the finger joints; rung 5 the wrist
  and forearm tremor with the central oscillation, and the finger twist; rung 6 the slow drift of
  the hand (`sway_rx`, `sway_rz`); rung 7 heartbeat and breathing (`body`).
- Scoring, sessions 2 and 3: every metric per phase and sensor. Accel noise is scored on the
  table phase only and handedness on the routine only.

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
- 2026-10-31, V4/V3 session 3 scores (recorded 2026-10-30 evening, power-ups 14-15, pivot screwed
  in, each placement checked live against session 1's holds; nothing refitted): V4 over sessions
  2 and 3 is 30/37 (session 2 alone 26/35). Tilted 10/10. Hinge horizontal 11/12 (from 9/12): the
  one fail is `palm_lsm9`'s static tilt at B, real 3.2° against 1.9° sim. Hinge vertical 9/13
  (from 7/13): static tilt at B fails for all three sensors (real 1.4-3.6°, sim 0.3-2.2°), and
  so does finger's gyro scale B->A (real 161.7°, sim 159.2°, window 2.4°). Base rotation 0/2:
  finger's and palm's mount rms, the chips' x/y gyro gains, which are not modelled (as in the V4
  check). V3 over sessions 2 and 3 is 82/84 (from 79/82): finger's gyro z- recovery (real -1.15,
  SD 0.76, against sim 1, window 2) and palm's accel y- clip level (real -32490 over 4 events,
  sim -32768, window 164).
- 2026-10-31, V1rig sessions 2 and 3 (night 8, power-up 13, and night 9, power-up 16): every V1
  config on all three sensors at once (`trio_*` firmware envs, `timusim-protocol`; 952 Hz clean on
  the shared bus both nights, so no split), then one 120-min default hold each night (`-s4-`,
  `-s5-`). All 24 captures are clean (0 missing). Scored as before over sessions 1-3: finger
  573/600 (541 over sessions 1-2), palm 393/420 (372), `palm_lsm9` 545/660 (522). Most configs now
  have 2-3 captures, so the windows carry a 2 SD term: of the fail->pass changes with session 3,
  18 of 35 (finger), 5 of 23 (palm) and 13 of 34 (`palm_lsm9`) have the sim within the old
  window, and the rest pass only because the window widened. `palm_lsm9` still fails on the
  gyro: bias 31 (every config and axis), noise floor and low-frequency level 16 each, std 10.
  That is the same unit difference as night 7. The finger's and palm's fails are spread out,
  the largest group being palm's accel Allan at 10-60 s (13). The night-9 build is the night-8
  firmware (`data/firmware/README.md`).
- 2026-10-31, code review (Claude, at the author's request). It was meant to change no fitted
  parameter, score or figure, and none changed. Before the review, every step of
  `timusim-reproduce` (comparisons skipped) was run as a baseline; after it, everything was run
  again. Every score and parameter file is identical. The 220 PNG figures are byte-identical
  too. The V1rig steps were rerun on the baseline's manifest without night 9's captures, which
  were recorded in between, so that their comparison uses the same data. ctest 6/6, pytest
  52 passed, 3 skipped. What changed:
  - V1, V2, V3 and V4 used four copies of the acceptance rule; they now share one,
    `acceptance.judge` (window max(tolerance, 2 SD), SD with ddof 1).
  - `fit.with_top_level` replaces the top-level keys of a parameter file in one place (V2's
    accel and V4's gyro corrections). Unused and duplicated helpers are removed.
  - The rig code shares one parameter-file map, one clip threshold and one score row type for
    V3 and V4. `sim.point_scenario` is shared by V1's still scenarios and the rig's replay
    scenarios.
  - `timusim-place`, the live placement used for V4 sessions 2 and 3, moved from the scratchpad
    into the package. `timusim-inspect --hold SENSOR` prints a hold's gravity.
  - The one-off checks behind findings in this file are in `diagnostics/`, with a README mapping
    each one to its finding. `data/firmware/` keeps the diff of each dirty firmware build.
  - C++: the scenario parser is split into one function per section, with the same error
    messages. The chip model's fallbacks by key share one helper. This is the only change in
    the night-9 firmware build, and it is in the simulator, not the firmware.
  - Not changed: the firmware sources (a change needs a reflash and hardware tests), the
    libraries copied from the glove project, and the comparison drivers.
- 2026-10-31, gyro gain calibration (test `GyroCal`), procedure drafted by Claude at the author's
  request before any capture; the author confirms or changes it before the first one. This
  change comes after V4's session-2 and session-3 scores were seen: base rotation fails because
  each chip's gyro has gain and cross-axis on all three axes, and the models have only the
  hinge axis's gain (above). The fit uses its own captures, independent of V4. V4 and V3 are then
  rescored with the matrices.
  - Why a new capture: V2's holds are still, so the gyro sees nothing to calibrate against. On
    the rig the hinge is always the same sensor axis. Turns between still poses, checked
    against gravity from the V2-calibrated accelerometer, fix all nine entries (Tedaldi et al.
    2014; the glove project's `cal/gyro_gain.hpp` fits only the diagonal this way).
  - Setup: all three sensors stay on the V4 arm, taped at stop A, with the pivot screwed in and
    the mounts unchanged, so one capture covers palm, finger and `palm_lsm9`. Rig firmware
    `default` config, the same chip settings as V4. Firm table.
  - Capture: one continuous recording of about 6 min. It starts with 30 s still, box on its base.
    Then come at least 16 poses, each one held still for at least 5 s with the box resting on the
    table or on a block, never held by hand. Between poses the box is turned by hand, in one
    smooth motion of 1-3 s, kept under about 150 dps (inside full scale on every chip). The
    turns go about all three box axes, each axis at least four times and in both directions.
    The poses spread gravity over the box's faces and edges, so no axis is turned about only
    while it is vertical (a turn about the vertical is invisible to gravity). The recording ends
    with 30 s still. Two captures in one session, at one power-up: the first is fitted and the
    second checked.
  - Fit, per sensor: a reading is m = G ω + b, with ω the true rate in the accelerometer's frame.
    A per-axis gain on negative rates (`gyro_negative_gain`) comes in before G, on every sensor
    (the author, before any capture: one procedure for both chips; V4 measured the LSM6DSL's
    direction difference only on the hinge axis, 0.2%).
    b is each still's mean gyro, interpolated linearly across the turn after it, so zero-rate
    drift and the power-up offset drop out. A still is at least 2 s with every gyro axis within
    3 dps of its median (V4's rest rule); a turn is the samples between two stills. Each turn's
    rate G^-1(m - b) is integrated to a rotation, which carries the gravity of the still before it
    into a prediction for the still after it. G and the negative gains are fitted by least
    squares on those angle misfits, starting from the identity.
  - Result: G is written as `gyro_gain` and the negative gains as `gyro_negative_gain` into each
    sensor's parameter file, in place of V4's hinge-axis gain. V4's gyro gain against gravity
    from the sweeps is then reported as a check of the calibration and no longer fitted.
    Reported per sensor: G, each turn's misfit on the fitted capture, and on the second capture
    with nothing refitted.
  - Acceptance of the calibration, before V4 uses it: on the second capture, the median turn
    misfit is under 0.5° and no turn's misfit is over 2°. A turn with missing samples is
    dropped, along with any still they fall in. A capture is redone only for a reason seen
    without its readings (a pose that moved, a bumped table) or when its remaining turns no
    longer cover every axis in both directions (the author, 2026-10-31, after capture 1 lost
    about 20 samples per sensor to one bad frame inside one turn, before any fit was run).
  - Session 1, power-up 17 (2026-10-31): fitted 143742, checked 151050. On the check, palm
    passed (median 0.32°, max 1.47°; 1.09° at unit gain), `palm_lsm9` passed (0.39°, 1.41°;
    1.63°) and the finger failed its median (0.67°, max 1.85°; 1.55°). 33 of 143742's 51 turns
    were over 150 dps, against the procedure's pace; the check's 21 turns were all but one
    under it, and on the check the finger's misfit grew with turn size (correlation 0.53), as a
    gain fitted at other rates would leave. Fitted on the check capture alone, the gain agrees
    within 0.4% on the diagonal, 0.6% on the negative gains. Extra terms tried and not taken:
    g-sensitivity (unstable, worse check) and a per-axis gain growing with rate (finger check
    0.55°). The author had 143742 redone at the procedure's pace as 154121, the fitted capture;
    151050 stays the check, though recorded before it. This redo comes from a deviation from the
    procedure, seen in the readings after the check had been scored.
  - 154121 (fitted): one bad frame at 139.4 s lost 132-148 samples per sensor inside one turn,
    which is dropped. One turn (248 s) peaked at 346-368 dps. The author's rule, from here on and
    for every GyroCal capture: a turn whose rate peaks over 150 dps is dropped. It also drops one
    turn of 151050 for the finger and two for `palm_lsm9`, whose misfits had been 0.24-0.62°.
  - Fitted on 154121 (16 turns left), every sensor failed the check: palm 0.51° median (max
    1.64°), finger 0.83° (4.96°), `palm_lsm9` 0.82° (4.34°). Pace was not the cause: slow turns
    did no better than 143742's 51 mostly fast ones. Nor was the accelerometer: refitted on the
    stills of 154121 and 151050 together (|a| = 1, a symmetric gain and an offset on top of V2),
    |a| tightens from ±1.4-6.2 mg to ±0.3-0.5 mg and V2 moves by at most 0.35% and 10 mg
    (`palm_lsm9` z zero, as in its power-up dependence), but the check stays at finger 0.91°,
    `palm_lsm9` 0.85°, while palm passes (0.38°). The 12 parameters are loosely pinned by 16
    turns (predicted 1 SD of Gxy 1.2%).
  - The author's changes, before the next fitted capture (155630) and any fit on it: a fitted
    capture has at least 40 turns, all under 150 dps, at least 6 each way and 12 in total about
    each sensor axis (a turn counts for an axis if it tilts gravity at least 30° about it); and
    each session's accelerometer correction is refitted, as above, on the stills of all its
    GyroCal captures before the gyro fit. Pooled over 154121 and 151050 with that correction, the
    fit reaches finger 0.36° and `palm_lsm9` 0.33° in-sample, which is not an acceptance result.
  - Session 1 result, fitted on 155630 (41 turns per sensor), checked on 151050: palm PASS
    (median 0.26°, max 1.24°; 1.05° at unit gain), finger FAIL (0.57°, 1.91°; 1.61°),
    `palm_lsm9` FAIL (0.56°, 1.32°; 1.38°). The LSM9DS1s fit their own capture worse than the
    check (0.73°, 0.76°), so the remainder is a chip effect the model lacks, not too few turns.
    The author keeps the 0.5° tolerance: the LSM9DS1 gains are not accepted. V4 and V3 are
    rerun two ways, both reported: strict (only palm's calibrated gain, the LSM9DS1s on V4's
    hinge-axis gain), the scored result; and all three calibrated, marked not accepted, to show
    the gains' effect.
  - Reruns (data/{v4,v3}-gyrocal-{strict,all}): V3 82/84 both ways, as before. V4 strict 30/37
    (before 30/37): palm's base-rotation mount rms passes (sim 1.15 -> 1.99 dps, real 2.41) and
    its hinge-horizontal B->A gyro scale fails (sim 171.7° -> 173.6°, real 171.0°). V4 all
    (not accepted) 29/37: hinge-vertical and tilted gyro scales move towards the real ones
    (finger hinge-vertical B->A passes, 159.2° -> 160.4°, real 161.7°), while with the hinge
    horizontal every sensor's sim now overshoots the real span by 1-2% (finger B->A fails,
    164.7° vs 161.7°); finger mount rms stays failing from the other side (0.44 -> 1.01 dps,
    real 0.77); `palm_lsm9` tilted closure fails (4.73° -> 6.66°, real 4.94°). The
    hinge-horizontal overshoot is common to all three sensors, which points at the reference
    there (rests from gravity) or at the sweeps' stop impacts rather than at one chip; not yet
    tested.
  - Then: V4 (every session) and V3 are rerun with the calibrated parameters. The change and the
    before/after scores are logged here, base rotation in particular.
- 2026-11-01, V5, before any V5 capture (the author): the palm sensor is the LSM6DSL on the STM32
  board, strapped firmly to the back of the hand, instead of `palm_lsm9`; it is easier to mount
  and better characterized (own V1/V2 parameters, GyroCal pass), where `palm_lsm9` has only the
  finger unit's parameters, which V1rig showed do not carry over. With the board on the hand in
  every trial there is no light-palm condition, so the mass-loading capture and its metric are
  dropped. The board's weighed mass sits at its measured centre: scenario `[[masses]]` takes an
  optional `offset` from the sensor (point mass; the board's own rotational inertia is left out,
  and the fitted wrist stiffness absorbs a common inertia error).
- 2026-11-01, V5, before any V5 capture (the author): the finger sensor goes on the middle
  phalanx, the only place it can be fixed, so it sees MCP and PIP flexion but not the DIP. The
  chain takes `finger_segment = "middle"` (sensor from the PIP, middle segment frame). Hand
  measured the same day into `sim/scenarios/v5/hand.toml`: proximal 36 mm, middle 34 mm, the
  points in the palm frame listed there; board 124 g as strapped on (wires and straps included).
  Fit recovery on simulated trials with this geometry (`diagnostics/v5_fit_recovery.py`):
  mounts 0.30° palm, 1.32° finger; both stiffness scales exact; densities wrist +0.8%, MCP
  flexion +11.9%, PIP −0.4%, abduction +2.0%; table support 0.296 against 0.3. MCP flexion is
  worse than with the distal mount (+2.4%) because the middle-phalanx sensor separates MCP from
  PIP flexion less well.
- 2026-11-01, V5 session 1, after its captures were seen (the author): the lift window widens
  from 60 ± 3 s to 57–66 s, so that trial 025316 (lift at 65.3 s) is kept. Settling, as the
  first time after the lift that the 0.5 s RMS of |ω| stays under 3× the hold's median for 2 s:
  025316 palm 66.4 s, finger 68.2 s (1.2 s into the 67–120 s hold window; 1.5% of the window
  above 3×, against 0.0% and 0.7% in the other two trials); 025905 palm and finger 63.7 s;
  030716 palm 62.8 s, finger 62.7 s. The early bound stays at 57 s, since the table window ends
  at 58 s.
- 2026-11-01, V5, after session 1's fit was seen (the author): tremor in three directions. The
  session-1 fit (cost 260; 0.48 on simulated trials) drove flexion and abduction only, while the
  real free hold has power about all three palm axes (3–8 Hz, palm frame: flexion axis −3.5 dB,
  forearm axis −8.5 dB, deviation axis −8.0 dB; model −39 dB on the latter two, its 1% axis
  floor), so the fit raised flexion about 15 dB to lift the floor. The additivity held (sum of
  unit runs within 1 dB of the full sim). The order matches Pigg et al. 2020 (postural tremor:
  wrist flexion, then forearm pronation-supination and wrist deviation about equal, one peak
  frequency across DOF), and Jombík et al. 2020 find physiological tremor between all three
  planes. Wrist deviation and forearm pronation-supination (`sway_ry`) join the fit with the
  wrist's stiffness scale; their inertia, stiffness and damping are Lakie's flexion values
  times Peaden & Charles 2014 ratios to flexion (sim/params/tremor_literature.toml); the board
  mass adds to `sway_ry`'s inertia about the forearm axis. Yao et al. 2012's 3-DOF wrist model
  (uncoupled pendulums with Hill muscles under FES) was read and not used: it is driven by
  stimulation pulses rather than noise, and its passive stiffness (0.3–0.4 Nm/rad) is far
  below the postural stiffness that sets tremor frequency.
- 2026-11-01, V5, after the three-direction refit was seen (the author): that refit cost 56.8;
  on simulated trials the wrist, deviation and forearm densities came back within 1.3% but MCP
  flexion went to 0 and PIP to +42%, since the middle-phalanx sensor turns with both about the
  same axis. MCP and PIP flexion now share one fitted power weight (each keeps its own
  literature inertia, stiffness and damping), and the stiffness-scale grid widens from
  0.5–8 to 0.25–8 because the finger scale stopped at the old lower edge (0.5). The
  per-axis band powers of that refit were within about ±5 dB of the real ones, too strong at
  3–13 Hz and too weak at 0.1–3 and 13–25 Hz; the free hold sits 20–35 dB above the bench
  noise floor (V1rig long s5) below 13 Hz, and trial 025316's table phase about 10 dB above
  the other two in every band.
- 2026-11-01, V5, same refit (the author): slow wander of the held hand is modelled. The free
  hold's 0.1–3 Hz power is 25–30 dB above the still chips' (bench floor, V1rig long s5), so it
  is hand motion, not chip drift, and the tremor oscillators reach it only with their tails
  (3–10 dB short). Two drift sources, `sway_rx` and `sway_rz` (the hand turned about the
  wrist's flexion and deviation axes), are critically damped oscillators with the wrist's
  inertia; the fit takes their resonance from 0.5, 1 and 2 Hz (one shared stiffness scale,
  damping scaled with its square root) and a level each. There is no literature value for them;
  Pigg et al. 2020 and Elble & McNames 2016 put voluntary and postural movement below about
  3–4 Hz. They form a sixth rung, `drift`, after `wrist_and_twist`. Finger abduction stays its
  own group: the finger sensor sees it about a different axis from flexion.
- 2026-11-01, V5, after the drift refit was seen (the author): the palm rig mass was wrong. V5's
  palm rig is the ST-Link with the palm sensor, 48 g as strapped on (reweighed), not the 124 g
  STM32 board; the finger stays 8 g. `hand.toml` takes 48 g; the board-centre offset is still
  the one measured for the STM32 board. Every fit before this one used 124 g.
- 2026-11-01, V5, after the drift refit was seen (the author): a central oscillation. Every trial,
  axis and sensor of session 1's free hold shows two peaks, about 6–7 Hz and a narrow one at
  11–13 Hz that moves between trials (10.9, 12.8, 11.5 Hz) and is common to both sensors, which
  one noise-driven resonator per DOF cannot make (Elble & McNames 2016: physiological tremor has a
  mechanical and a central 8–12 Hz part). One narrow-band drive shared by the wrist and forearm
  DOF joins the model (sim `[[drives]]`), its frequency the median measured hold peak, its width
  and gains fitted. With the drive's frequency left free the 5-band fit put it at 13.1 Hz, on the
  8–13/13–25 Hz band edge, to fill a 13–25 Hz deficit, so the frequency is measured instead.
  The trials' spread is about 9 against a perfect model's 3.1, and the earlier fit's 35.4 was not
  trial noise; per-trial fits chose different grid points (cost 40–53 each).
- 2026-11-01, V5, same (the author): heartbeat and breathing. The table phase shows lines at
  1.20 Hz and 2.35–2.40 Hz (heartbeat and its harmonic) and 0.30–0.35 Hz (breathing), +3–6 dB
  over the background in trials 025905 and 030716 and hardly visible in 025316, whose hand was
  not still; no mains or machine lines. Where palm and finger are coherent their delay is about
  0 (±50 ms, no consistent sign), so no delay is modelled. Two drives not scaled by the support
  act through the whole run (the free hold's broadband motion buries them there); they are
  fitted to the table spectrum against its own background, and form a seventh rung, `body`.
- 2026-11-01, V5, same (the author): the mass-loading capture comes back, one per session after
  the three trials, to tell the mechanical peak (moves with the load) from the central one (does
  not). It is reported, not scored.
- 2026-11-01, V5, refit with all of the above (session 1 only; no session 2 or 3 data exist):
  cost 24.2 (√(24.2/32) × 3 dB ≈ 2.6 dB typical miss), from 35.4. Wrist scale 0.71, finger 1,
  drift 0.25 (0.5 Hz, the grid's lower edge); central 11.5 Hz, width 3.1 Hz, nearly all on wrist
  flexion, whose own broadband density fits to about 0, as does finger flexion's (the finger
  sensor's flexion is the wrist's, coherence 0.9–0.98 below 13 Hz); heartbeat 1.20 Hz (2nd
  harmonic 0.51), mostly on forearm roll, breathing 0.28 Hz. On simulated trials with a central
  drive (11.5 Hz, width 1.0) the fit returns every density within 8%, the support within 1%, the
  width as 1.42 Hz and the central gain within 7%. The finger chip's angle to the palm grew
  0.6°, 8.9° and 3.8° over the three trials (slow slip, no rocking seen).
- 2026-11-01, V5 session 2 recorded (power-up 19): trials 135253, 135946, 140542 and the
  mass-loading capture 141336 (+69 g); six false starts (0–1871 samples) excluded in the manifest.
- 2026-11-01, V5, after session 2 was seen (the author): the routine is replayed as performed.
  The by-eye moves differ from the script: curl 84–107° (105), extension 29–45° (30), flexion
  12–23° (30), abduction and radial deviation 15–30° with the sign opposite to the script's
  "towards the thumb"; out in 0.3–1.0 s and back in about 0.6 s (1 s scripted), starting about
  1 s before each cue. Each trial's sim now takes every repetition's angle and timing measured
  from that trial: the angle integrated from the segment-frame gyro (palm, or finger less palm)
  about the move's axis, the timing from its 10/50/90% crossings as a min-jerk move, curl split
  between MCP and PIP as scripted. A sim replaying measured repetitions measures back within
  0.42° and 14 ms.
- 2026-11-01, V5, after session 2 was seen (the author): session 2's free hold has no 11–13 Hz
  peak (session 1's had), so the central level, fitted on session 1 alone, varies by day. Its
  band, 8–13 Hz in the hold and routine, is scored at 6 dB instead of 3 (about the session 1 to
  2 difference, so chosen from the data it judges); the ladder also reports the 3 dB count.
- 2026-11-01, V5, after session 2's mass-loading capture was seen (the author): peaks for the
  mass check are taken on spectra smoothed over 1 Hz, since single bins jumped between trials
  by up to 3 Hz on session 2's flat spectra. Result: the 6.3 Hz peak (unloaded trials 6.25,
  6.25, 6.50 Hz) did not move with +69 g (6.50 Hz), where the model predicts −0.46 Hz; the
  central peak scatters 9.75–14.13 Hz between unloaded trials and cannot be read.
- 2026-11-01, V5, after session 2 was seen (the author): the table support is fitted a second
  time on full-model table sims around the band-power value, scored on the table's per-axis
  band powers and both sensors' accel noise (std of |a|, 20%); linear acceleration is left out,
  since on the table it reads the chips' static offset from 1 g (9 mg for the finger in a sim
  with no motion). Result 0.128 (0.142 on band powers alone); the ladder is unchanged (32/58).
  The finger's accel noise stays about 2× the real (2.2 against 1.16 mg) at any support that
  fits its gyro: the sim's finger accelerates too much per unit of rotation on the table.
- 2026-11-01, V5, after session 2 was seen: what the sim's routine misses. Fitting each
  repetition's angle, min-jerk leaves 1.02% RMS of the angle, a cubic 1.06%, min-jerk with an end
  overshoot 0.93%, an asymmetric beta velocity profile 0.71% (curl and abduction peak early,
  at 0.30 and 0.17 of the move; wrist moves near 0.5); the residual rate is mostly 3–6 Hz.
  During moves the real gyro is 10–18 dB above the sim's at 3–100 Hz with the gross motion
  (0.1–3 Hz) matched; palm and finger are coherent there at 3–13 Hz (0.5–0.6) and not above
  13 Hz (0.24–0.30, the floor for those windows), so the high band is local to each mount
  (strap, skin). In the routine's holds the real tremor is 3–7 dB (palm) and 6–10 dB (finger)
  above the free hold's; the sim's is the same as its free hold.
- 2026-11-01, V5, after session 2 was seen (the author): movement roughness. Three parts,
  fitted on session 1's routine split into move and hold windows (band powers per axis,
  3–100 Hz, by least squares from two sims per trial, full model and without tremor): a routine
  gain on the support (1.56); a drive gated by each joint's scripted speed through the muscle
  filter (sim `[[drives]]` `gate = "speed"`; forearm rotation, never scripted, takes the wrist's
  speed), 7.57 Hz, width 0.70 Hz; and mount jolts, rate added at each sensor while its segment
  moves (sim `[jolt.palm]`, `[jolt.finger]`), held above 13 Hz where the sensors are
  incoherent, fitted at 13 Hz (the bound), width 5.6 Hz, palm 0.34 and finger 1.05 dps/√Hz per
  rad/s. Routine-window cost 559 without, 39.5 with (48 terms); with the jolts allowed down to
  10 Hz it was 55.6 before forearm gating, 59.7 held above 13 Hz. On simulated trials with known
  values (diagnostics/v5_routine_recovery.py) the fit returns the routine gain within 2%, the
  jolts within 1% in frequency and 4–8% in level, and the roughness at 7.9 Hz (7.5), but not its
  split between the wrist DOF (−60% to +44%) or the tiny finger gains: those are gated by the
  same moves and seen by the same two sensors, so only the sensor-level result is determined.
  It forms an eighth rung, `roughness`. Session 2: 41/58 pass (32 at `body`); the routine 15/20
  (7/20), every routine band power now passing. Still failing in the routine: dominant
  frequency (real 3.1–3.2 Hz, sim 7.0–7.4 Hz) and handedness (real 0.87–0.91, sim 0.71–0.74,
  it was 0.98 without roughness).
- 2026-11-01, V5, after session 2 was seen (the author): table contact. On the table the hand
  rests and rocks about a held point rather than swinging about the wrist (sim
  `geometry.contact_pivot`, the hand's held point, and `finger_contact_pivot`, the finger's own,
  lying on the table too; full contact at the start support, none once lifted; rates unchanged).
  One point could not serve both sensors (palm accel noise needs it at the wrist, the finger's
  near the knuckles), hence two. The table support now comes from the gyro band powers alone
  (0.142) and each pivot from its sensor's accel noise on session 1: palm 0.6 cm from the
  wrist, finger 5.6 cm proximal of the PIP (about 2 cm into the palm from the MCP). It forms a
  ninth rung, `contact`. Session 2: the finger's table accel noise falls from 2.2 to 1.49 mg
  (session 1 real 1.46, session 2 real 1.16, so still outside session 2's window); the ladder
  stays at 41/58.
- 2026-11-01, V5, after session 2 was seen (the author): move shape. Each repetition's out and
  back curves are fitted with a beta velocity profile, s^(a−1)(1−s)^(b−1) (min-jerk is a = b = 3,
  bounded 2–8 so the acceleration stays finite), and replayed (sim `profile = "beta"`). Medians:
  wrist moves near min-jerk (peak at 0.40–0.54 of the move); curl out and abduction peak early
  (0.27, 0.19); the curl's return snaps back with a long tail (a = 2, b = 8, 77% on a bound). A
  sim replaying them measures back within 0.63° (start times within 0.2 s, traded against the
  slow tails). Routine-window cost without roughness 447 (559 with min-jerk), with it 37.6
  (39.5). The ladder stays at 41/58: the routine's dominant frequency (real 3.1 Hz, sim 6.9–7.4)
  and handedness (real 0.87–0.91, sim 0.73) still fail.
- 2026-11-01, V5, after session 2 was seen (the author): routine dominant frequency and
  handedness as fit targets. The roughness band is now chosen from a grid of centres and widths
  on the band powers plus each sensor's routine dominant frequency (band powers alone fitted a
  narrow 7.6 Hz band whose peak outranked the moves' 3 Hz): 2.0 Hz, 1.5 Hz wide; session 2's
  routine dominant frequency now passes on both sensors (sim 3.0/3.3 Hz, real 3.1/3.2). One
  session 2 trial with roughness on one joint group at a time showed handedness falls with
  forearm roll (palm 0.83) and the finger joints (finger 0.90), not with the routine gain or
  jolts. Roll's share is therefore also chosen on session 1's simulated handedness (shares 1,
  0.5, 0.25, 0, the other weights refitted): 0.5. Session 2 handedness: palm 0.84 (real 0.91,
  was 0.76), finger 0.78 (real 0.87, was 0.75), both still outside. Without roll the refit moves
  the roughness onto the finger joints and the finger falls further; the finger routine 3–8 Hz
  band is 5 dB above session 2's real, so session 2's roughness was lower than session 1's.
  Mount jolts now also turn the chip by their integrated angle (forgotten below 0.5 Hz); this
  leaves handedness unchanged. The ladder stays at 41/58 (two routine band powers lost to the
  broader roughness, the two dominant frequencies gained).


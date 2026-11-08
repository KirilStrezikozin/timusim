# TIMUSim

A MEMS inertial data simulation environment for hand-wearable wireless devices.

TIMUSim simulates the accelerometer and gyroscope of low-cost 6-DOF MEMS IMUs (LSM9DS1 and LSM6DSL)
worn on the hand. It moves a palm and an index finger through scripted poses, adds physiological
hand tremor as motion at the joints, and passes the resulting motion through a model of the chip:
its zero level, noise, drift, filters, output rate, quantization and clipping. The simulated
sensor speaks the same interface as the real drivers and writes the same sample stream as the
capture firmware, so one set of tools reads real and simulated data.

The repository also holds the procedure that fits the simulation parameters on real captures,
the validation tests against those captures, and the comparison with other IMU simulators.

## Contents

- [Quickstart](#quickstart)
- [Directory structure](#directory-structure)
- [How it works](#how-it-works)
- [Where to look](#where-to-look)
- [Status and limitations](#status-and-limitations)
- [License](#license)

## Quickstart

### Requirements

- CMake 3.13 or newer and a C++20 compiler
- [uv](https://docs.astral.sh/uv/) to run Python for the plots
- Or, with Nix: run `nix develop` and everything above is available

### Try it

Get the code and the three required submodules:

```sh
git clone https://github.com/KirilStrezikozin/timusim.git
cd timusim
git submodule update --init external/eigen external/lsm6dsl-pid external/lsm9ds1-pid
```

Build the example:

```sh
cmake -B build
cmake --build build --target timusim-quickstart -j8
```

Run both simulations (a few seconds each):

```sh
build/examples/quickstart/timusim-quickstart examples/quickstart/hold.toml examples/quickstart/output/hold.csv
build/examples/quickstart/timusim-quickstart examples/quickstart/movement.toml examples/quickstart/output/movement.csv
```

Draw the figures:

```sh
uv run python examples/quickstart/plot.py
```

### Output

Written to `examples/quickstart/output/`:

| File | Contents |
| --- | --- |
| `hold.png` | 60 s static hold: gyro signal and spectrum for the palm and finger sensors |
| `movement.png` | One finger curl and back, 8 s: accelerometer and gyroscope against the ideal input |
| `hold.csv`, `movement.csv` | Every sample, in g and dps, next to the ideal input |

In `hold.png`, the peak around 7-12 Hz is tremor. In `movement.png`, the palm's offset from the ideal line is the chip's zero level.

### Change the simulation

The scenarios are `examples/quickstart/hold.toml` and `examples/quickstart/movement.toml`. To
change a move, edit its `[[moves]]` entries, then rerun the simulation and plot commands. No
rebuild is needed. For example, this holds for 2 s, then curls the finger over 1 s:

```toml
[[moves]]
hold_s = 2

[[moves]]
duration_s = 1.0
joints_deg = { mcp_flex = 45, pip_flex = 60 }
```

To change a chip's rate or range, edit `chip` under `[palm]` or `[finger]`. To turn off a model
layer, add this section to a scenario:

```toml
[realism]
chip_transfer = true
noise = true
tremor = false
```

`sim/scenarios/README.md` lists every scenario key.

## Directory structure

```
lib/                 C++ libraries, header-only
  hand/              hand motion: poses and moves, joint tremor, the palm-finger chain
  chip/              chip signal path, LSM9DS1 and LSM6DSL configs, SimIMU
  imu/               real chip drivers and the IIMU interface SimIMU shares with them
  capture/           sample stream format, shared by the firmware and the simulator
  bus, log, time     support code
sim/                 timusim-sim, the full simulator, and the scenario format
examples/
  quickstart/        the example above
  stm32-capture/     firmware that records the real sensors (PlatformIO, B-L475E-IOT01A)
  stm32-conformance/ firmware that checks a real chip against the IIMU convention
src/timusim/         Python: capture reader, fitting, validation tests V1-V5, comparison, figures
compare/             runners for the other simulators, each in its own environment
diagnostics/         one-off checks behind findings in RESEARCH.md
tests/               C++ tests (ctest) and Python tests (pytest)
data/manifest.toml   list of recorded captures; the captures are not in the repository
external/            submodules and vendored libraries
RESEARCH.md          validation protocol, its changes and findings
```

## How it works

```
scenario.toml
  -> hand model:  poses and moves -> + joint tremor -> palm-finger chain
  -> ideal force and rate at each sensor
  -> chip model:  gain, zero level, drift, noise -> filters -> output rate -> quantize, clip
  -> samples, as the real chip's FIFO gives them
```

1. **Scenario.** A TOML file gives the hand's size, where the sensors sit, a start pose, a list of
   holds and moves, the tremor parameters and each chip's register config. `sim/scenario.hpp`
   reads it.
2. **Hand motion.** Moves run from pose to pose with smooth time profiles. Each joint is a small
   spring-mass-damper turned by random torque, so tremor appears as rotation about the joints, with
   a peak set by the joint's stiffness and inertia. A few narrow-band drives add tremor that several
   joints share. The palm-finger chain (wrist, knuckle, and the finger's middle and end joints)
   turns joint angles into the ideal acceleration and rotation rate at each sensor.
3. **Chip.** The ideal signal gets the chip's gain, zero level, drift and white noise at a high
   internal rate, then goes through the chip's filters, down to the output rate, and is rounded
   and clipped into raw codes. The register config picks the filters and ranges, as on the real
   chip.
4. **Output.** `SimIMU` serves these samples through the same interface as the real drivers,
   with a 32-sample FIFO and a virtual clock, so code written for the hardware runs against it on a
   PC. `timusim-sim` writes the capture stream the firmware writes, plus a CSV of the ideal input.
5. **Fitting and validation.** Chip parameters are fitted on still bench captures, from the noise
   spectrum and Allan deviation. Hand parameters are fitted on a first session on the hand. Later
   sessions are only scored, under the protocol in `RESEARCH.md`, which was fixed before the first
   capture.

## Where to look

| To see | Look at |
| --- | --- |
| The smallest full use of the simulator | `examples/quickstart/main.cpp` |
| Every scenario key | `sim/scenarios/README.md` |
| How tremor is generated | `lib/hand/include/hand/tremor.hpp` |
| How poses become sensor readings | `lib/hand/include/hand/chain.hpp` |
| The chip signal path | `lib/chip/include/chip/model.hpp` and `filter.hpp` |
| How a register config picks filters | `lib/chip/include/chip/lsm9ds1.hpp`, `lsm6dsl.hpp` |
| The simulated chip behind the driver interface | `lib/chip/include/chip/sim_imu.hpp`, `lib/imu/include/imu/iimu.hpp` |
| How chip parameters are fitted | `src/timusim/fit.py` |
| The validation tests and results | `RESEARCH.md`, `src/timusim/v1.py` … `v5.py` |
| The comparison with other simulators | `src/timusim/compare.py`, `compare/` |

To run the tests, after the quickstart build:

```sh
cmake --build build -j8
ctest --test-dir build
uv run pytest tests/python
```

The Python tests take several minutes.

## Status and limitations

TIMUSim is research code and still in progress. It is one approach to simulating hand-worn IMUs,
built around two chips and one hand, not a finished or general tool.

- The chip model is the most tested part. It is fitted and checked on bench, static, saturation and
  rig captures, but on only one or two units of each chip, so the fitted values may be specific to
  those units.
- The hand motion is the newest part and the least tested. It has been compared with one person's
  hand, with one sensor on the finger instead of the three a motion-capture glove uses. Strap and
  skin motion are fitted to the data rather than derived from physics. Treat its parameters as a
  starting point.
- The quickstart's chip and tremor values come from one subject and one unit of each chip, so
  they are examples, not values for any chip or person.
- The quickstart examples has movement roughness disabled, so moves are smoother than real ones.
- The simulator does not model temperature or the magnetometer.

Other simulators suit other needs: [IMUSim](https://github.com/martinling/imusim) includes the
magnetometer and whole-body motion, [WIMUSim](https://github.com/STRCWearlab/WIMUSim) varies body
shape and sensor placement for activity recognition,
[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) simulates GNSS and inertial navigation, and
MATLAB's [imuSensor](https://www.mathworks.com/help/nav/ref/imusensor-system-object.html) includes
the magnetometer and temperature effects.

## License

Apache 2.0, see `LICENSE`. Vendored code keeps its own license.

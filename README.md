# timusim

Simulated LSM9DS1 and LSM6DSL accelerometer and gyroscope data for a palm and a two-joint
finger chain, validated against real captures.

- `lib/`: C++ libraries. `imu`, `bus`, `log`, `math-utils`, `span`, `time` and `eigen-compat`
  are copies of the glove project's libraries; `capture` is the capture stream format, `hand`
  the kinematic chain, trajectories and joint tremor, `chip` the shared signal path, the
  LSM9DS1 and LSM6DSL configs on it and `SimIMU`.
- `sim/`: the `timusim-sim` CLI, scenarios and parameter files.
- `examples/`: STM32 firmware (`stm32-conformance`, `stm32-capture`).
- `external/`: Eigen, LSM6DSL and LSM9DS1 drivers (submodules), tl, fmt, Fusion and toml++
  (vendored); IMUSim, WIMUSim and gnss-ins-sim (optional submodules) and MATLAB (installed
  separately), for `compare/` only.
- `compare/`: drivers that run the V1 still holds through IMUSim and WIMUSim, each in its own
  uv environment.
- `src/timusim/`: Python package for reading captures, metrics and plots.
- `notebook.py`: marimo playground over the package.
- `data/`: captures, gitignored except `data/manifest.toml`.

## Setup

```sh
git submodule update --init
direnv allow          # or: nix develop
uv sync
```

## Build

```sh
cmake -B build && cmake --build build -j8
uv run marimo edit notebook.py
```

## Capture

```sh
cd examples/stm32-capture && pio run -t upload && cd -
uv run timusim-capture --test V1 --config default --session 1 --seconds 600 --notes "foam pad"
uv run timusim-inspect data/2026-10-20/V1-default-s1-101500.bin
```

`stm32-capture` has one PlatformIO env per bench config: `stm32_capture` (default: 476 Hz,
±4 g, ±2000 dps, gyro LPF1), the V1 set `v1_low_range`, `v1_238hz`, `v1_119hz`,
`v1_lpf2_strong`, `v1_lpf2_light`, and extras `v1x_952hz`, `v1x_mid_range`, `v1x_16g`,
`v1x_xl_lp_off`, `v1x_xl_div50`. Those change the finger LSM9DS1; the palm sweep `palm_208hz`,
`palm_833hz`, `palm_lp1_light`, `palm_lp1_aggressive`, `palm_xl_div50`, `palm_low_range` changes
the palm LSM6DSL (default 416 Hz, ±4 g, ±2000 dps, gyro LPF1 FTYPE 00, accel LPF2 ODR/9). Each
capture counts as a default one for the sensor it leaves alone. The `pair_<finger>_<palm>` envs
change one setting on each sensor; their config is `<finger>+palm_<palm>`. Each env writes its config name
into the capture header; `--power-up N` records which board power-up a capture belongs to.

V2 holds the board still at known orientations, one capture per hold, default firmware:

```sh
uv run timusim-capture --test V2 --config default --session 1 --power-up 4 --seconds 90 --up finger=+z
uv run timusim-bench --trim 30 data/2026-10-23/V2-default-s1-101500.bin   # accel bias: +1 g on the up axis
uv run timusim-v2                    # fit accel gain on session 1, simulate, score the reported holds
```

- Level the tabletop first with a phone level, along both edges.
- Six holds per session: flat face up (`+z`) and face down (`-z`) on the tabletop; `+x`, `-x`,
  `+y`, `-y` taped against the table leg, the board edge on a straight edge, plumb checked with
  the phone level. Optional extra holds at 30° and 60° take `--up finger=x,y,z`.
- `--up SENSOR=DIR` is world up in that sensor's frame; give it only for boards at a known
  orientation. The first 30 s are dropped as settling, so leave the board alone for the whole
  90 s, and run nothing heavy on the host while recording.
- Session 1 fits the accel zero level and gain; re-tape for each later session, which is reported.

The firmware streams every FIFO sample of both IMUs as COBS frames with a CRC
(`lib/capture/include/capture/protocol.hpp`). `timusim-capture` writes the raw stream, a JSON
file with the header and a summary, and a `data/manifest.toml` entry. A capture is clean when it
has no bad frames, FIFO overruns or missing sample indices.

## Simulate

```sh
build/sim/timusim-sim sim/scenarios/example.toml -o data/sim/example.bin
uv run timusim-inspect data/sim/example.bin
```

A scenario sets the hand geometry, each sensor's chip config and parameter file, the realism
layers (`chip_transfer`, `noise`, `tremor`), joint oscillators and added masses, and a pose
sequence. The output is the same stream the firmware writes, plus `OUT.truth.csv`.

## Analyse

```sh
uv run timusim-bench data/2026-10-21/V1-default-s1-011744.bin   # V1 metrics per axis
uv run timusim-v1                    # fit on session 1, simulate, score the reported sessions
uv run timusim-v1 --sensor palm      # the same procedure for the LSM6DSL, into data/v1-palm
uv run timusim-figures --out figures
```

## Compare

IMUSim (the Python 3 port at eeetem/imusim, GPL-3.0), WIMUSim and gnss-ins-sim (Aceinna, MIT)
and MATLAB's `imuSensor` are optional. Each runs from its own environment under `compare/`; the
IMUSim driver is GPL-3.0-or-later because it imports IMUSim, everything else stays Apache 2.0,
and no simulator's code is copied into this tree. `timusim-compare` runs the ones that are set
up and names the others.

```sh
git submodule update --init external/imusim external/wimusim external/gnss-ins-sim
uv sync --project compare/imusim     # builds IMUSim's Cython modules on first run, into compare/imusim/.build
uv sync --project compare/wimusim --preview-features extra-build-dependencies   # torch CPU, builds pytorch3d
uv sync --project compare/gnss-ins-sim
uv run timusim-compare               # finger; --sensor palm; --sim imusim to run one
uv run timusim-compare --test V2     # the V2 holds, into data/compare-v2
```

MATLAB runs in an Ubuntu distrobox from the `matlab` dev shell, with Navigation Toolbox or Sensor
Fusion and Tracking Toolbox for `imuSensor`:

```sh
nix develop .#matlab
distrobox create --name timusim-matlab --image docker.io/library/ubuntu:24.04
# MATLAB's Ubuntu dependencies (mathworks-ref-arch/container-images, matlab-deps), as root in the box
wget -O external/matlab/mpm https://www.mathworks.com/mpm/glnxa64/mpm && chmod +x external/matlab/mpm
distrobox enter timusim-matlab -- external/matlab/mpm install --release=R2026b \
    --destination=$PWD/external/matlab/R2026b --products MATLAB Navigation_Toolbox Sensor_Fusion_and_Tracking_Toolbox
distrobox enter timusim-matlab -- $PWD/external/matlab/R2026b/bin/matlab -licmode onlinelicensing   # sign in once
(cd compare/matlab && uv sync)
uv run timusim-compare               # from this shell, so distrobox is on PATH
```

Each simulator gets what its sensor model can express, fitted on the same config's session-1
captures: the measured ODR, gyro zero level, per-axis standard deviation and, for IMUSim, a
16-bit ADC at the chip's LSB; for gnss-ins-sim, one Gauss-Markov drift term per axis; for
MATLAB, the LSB and range and a 1/f bias instability with rate random walk. The
reported captures are simulated at their own orientation and scored with the V1 rule;
`data/compare/scores.csv` puts them next to timusim's V1 scores. For V2, IMUSim and MATLAB get
the accel zero level and 3x3 gain fitted on the session-1 holds, the others the zero level alone; each
reported hold is simulated at its declared orientation and `data/compare-v2/scores.csv` puts the
results next to timusim's V2 scores. `timusim-figures --only v2` draws both.

## Tests

```sh
cd build && ctest
uv run pytest tests/python
```

## License

Apache 2.0, see `LICENSE`. Vendored code keeps its own license.

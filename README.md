# TIMUSim

A MEMS inertial data simulation environment for hand-wearable wireless devices.

TIMUSim simulates low-cost 6-DOF IMUs (LSM9DS1 and LSM6DSL) on a palm and index finger kinematic
chain. It models the chip's noise, drift, filters, quantization and clipping, and physiological
hand tremor as motion at the joints.

Full documentation will follow. This page shows how to run two short simulations.

## Requirements

- CMake 3.13 or newer and a C++20 compiler
- [uv](https://docs.astral.sh/uv/), for the plots
- Or, with Nix: run `nix develop` and everything above is available

## Try it

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

## Output

Written to `examples/quickstart/output/`:

| File | Contents |
| --- | --- |
| `hold.png` | 60 s static hold: gyro signal and spectrum for the palm and finger sensors |
| `movement.png` | One finger curl and back, 8 s: accelerometer and gyroscope against the ideal input |
| `hold.csv`, `movement.csv` | Every sample, in g and dps, next to the ideal input |

In `hold.png`, the peak around 7–12 Hz is tremor. In `movement.png`, the palm's offset from the ideal line is the chip's zero level.

## Change the simulation

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

## Limitations

- The chip and tremor values are fitted on one subject and one unit of each chip, so they are
  examples, not values for any chip or person.
- These examples leave out movement roughness, which is still being fitted, so moves are smoother
  than real ones.
- The simulator does not model temperature or the magnetometer.

## License

Apache 2.0, see `LICENSE`. Vendored code keeps its own license.

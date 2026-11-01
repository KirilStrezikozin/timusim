import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium", app_title="timusim")

with app.setup:
    import pathlib

    import marimo as mo

    from timusim import capture, figures, fit, manifest, sim, v1


@app.cell
def _():
    mo.md("""
    # timusim

    Captures, chip fits and simulation runs. The figures CLI (`timusim-figures`) writes the
    same figures without the notebook.
    """)
    return


@app.cell
def _():
    entries: list[dict] = manifest.load()
    mo.ui.table(
        [{k: e.get(k) for k in ("file", "test", "config", "session", "clean", "notes")} for e in entries],
        selection=None,
    )
    return (entries,)


@app.cell
def _(entries: list[dict]):
    picked = mo.ui.dropdown(
        options=[e["file"] for e in entries],
        value=entries[-1]["file"] if entries else None,
        label="capture",
    )
    sensor = mo.ui.dropdown(options=sorted(v1.CHIP), value=v1.SENSOR, label="sensor")
    mo.hstack([picked, sensor])
    return picked, sensor


@app.cell
def _(picked):
    cap: capture.Capture = capture.read_capture(manifest.DATA_DIR / picked.value)
    mo.md(
        "\n".join(
            f"- {s.name} {s.chip}: {len(s.index)} samples, {s.odr_hz:.3f} Hz, missing {s.gaps.missing_samples}"
            for s in cap.sensors.values()
        )
    )
    return (cap,)


@app.cell
def _(cap: capture.Capture, sensor):
    figures.psd(cap, sensor=sensor.value)
    return


@app.cell
def _(cap: capture.Capture, sensor):
    mo.hstack(
        [
            figures.filter_shape(cap, kind="accel", sensor=sensor.value),
            figures.filter_shape(cap, kind="gyro", sensor=sensor.value),
        ]
    )
    return


@app.cell
def _(cap: capture.Capture, sensor):
    sensor_id: int = cap.stream(sensor.value).id
    params_text: str = fit.params_toml([fit.fit_capture(cap, sensor_id)], source="notebook")
    mo.md(f"```toml\n{params_text}```")
    return


@app.cell
def _():
    mo.md("""
    ## Simulate

    A scenario from `sim/scenarios/`; the C++ simulator must be built (`cmake --build build`).
    """)
    return


@app.cell
def _():
    scenario = mo.ui.dropdown(
        options=sorted(str(p) for p in (sim.ROOT / "sim" / "scenarios").glob("*.toml")),
        label="scenario",
    )
    run_button = mo.ui.run_button(label="run")
    mo.hstack([scenario, run_button])
    return run_button, scenario


@app.cell
def _(run_button, scenario):
    mo.stop(not run_button.value or not scenario.value)
    out: pathlib.Path = sim.ROOT / "data" / "sim" / (pathlib.Path(scenario.value).stem + ".bin")
    out.parent.mkdir(parents=True, exist_ok=True)
    sim.run(pathlib.Path(scenario.value), out)
    figures.psd(capture.read_capture(out), trim_s=0.0)
    return


@app.cell
def _():
    mo.md("""
    ## V1

    Fit on session 1, simulate every config, score the reported sessions, for the sensor picked
    above.
    """)
    return


@app.cell
def _():
    v1_button = mo.ui.run_button(label="run V1")
    v1_button
    return (v1_button,)


@app.cell
def _(sensor, v1_button):
    mo.stop(not v1_button.value)
    work_dir: pathlib.Path = manifest.DATA_DIR / ("v1" if sensor.value == v1.SENSOR else f"v1-{sensor.value}")
    results: dict[str, list[v1.Score]] = v1.run(manifest.DATA_DIR, work_dir, sensor=sensor.value)
    mo.ui.table(
        [
            {
                "config": config,
                "metric": s.metric,
                "real": f"{s.real_mean:.4g}",
                "sim": f"{s.sim_mean:.4g}",
                "window": f"{s.window:.3g}",
                "pass": bool(s.passed),
            }
            for config, scores in results.items()
            for s in scores
        ],
        selection=None,
    )
    return


if __name__ == "__main__":
    app.run()

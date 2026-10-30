import argparse
import datetime
import json
import pathlib
import signal
import sys
import time
import typing

import numpy as np
import serial

from timusim import bench, capture, compare, manifest, v1, v2, v3, v4


def summarize(cap: capture.Capture) -> dict[str, typing.Any]:
    sensors = {}
    for s in cap.sensors.values():
        temps = cap.temps.get(s.id)
        sensors[s.name] = {
            "chip": s.chip,
            "samples": int(len(s.index)),
            "duration_s": round(s.duration_s, 3),
            "odr_nominal_hz": float(s.header.get("odr_hz", "nan")),
            "odr_measured_hz": round(s.odr_hz, 3),
            "overruns": s.gaps.overruns,
            "missing_samples": s.gaps.missing_samples,
            "temp_c": round(float(temps[1].mean()), 2) if temps is not None and len(temps[1]) else None,
        }
    return {
        "fw": cap.header.get("fw", ""),
        "uid": cap.header.get("uid", ""),
        "bad_frames": cap.bad_frames,
        "restarts": cap.restarts,
        "clean": cap.bad_frames == 0 and all(s.gaps.clean for s in cap.sensors.values()),
        "sensors": sensors,
    }


def _print_summary(summary: dict[str, typing.Any]) -> None:
    print(
        f"fw={summary['fw']} uid={summary['uid']} bad_frames={summary['bad_frames']} "
        f"restarts={summary['restarts']} clean={summary['clean']}"
    )
    for name, s in summary["sensors"].items():
        print(
            f"  {name} {s['chip']}: {s['samples']} samples over {s['duration_s']} s, "
            f"ODR {s['odr_measured_hz']} Hz (nominal {s['odr_nominal_hz']}), "
            f"overruns={s['overruns']} missing={s['missing_samples']} temp={s['temp_c']} C"
        )


def capture_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Record the capture firmware's stream into data/.")
    parser.add_argument("--test", required=True, help="validation step, e.g. V1")
    parser.add_argument("--config", default="default", help="sensor config name")
    parser.add_argument("--session", type=int, default=1)
    parser.add_argument("--notes", default="")
    parser.add_argument("--power-up", type=int, default=None, help="board power-up count")
    parser.add_argument(
        "--up", action="append", default=[], metavar="SENSOR=DIR",
        help="V2: world up in that sensor's frame, +x, -x, +y, -y, +z, -z or x,y,z; repeat per sensor",
    )
    parser.add_argument("--seconds", type=float, default=None, help="stop after this long; default Ctrl-C")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    args = parser.parse_args(argv)
    ups: dict[str, tuple[float, float, float]] = {}
    for spec in args.up:
        name, _, direction = spec.partition("=")
        try:
            ups[name] = v2.parse_up(direction)
        except ValueError as err:
            parser.error(str(err))

    # A shell's background job starts with SIGINT ignored; stopping a detached capture needs it.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    started = datetime.datetime.now()
    out_dir = args.data_dir / started.strftime("%Y-%m-%d")
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{args.test}-{args.config}-s{args.session}-{started.strftime('%H%M%S')}"
    bin_path = out_dir / f"{stem}.bin"

    print(f"recording {bin_path}, Ctrl-C to stop", file=sys.stderr)
    with serial.Serial(args.port, args.baud, timeout=0.2) as port, bin_path.open("wb") as out:
        # The USB serial bridge still holds stream from before the port was opened;
        # it arrives first, so drop a second of input before recording.
        drain_until = time.monotonic() + 1.0
        while time.monotonic() < drain_until:
            port.read(4096)
        synced = False
        t0 = time.monotonic()
        last_report = t0
        written = 0
        tail = b""
        try:
            while args.seconds is None or time.monotonic() - t0 < args.seconds:
                chunk = port.read(4096)
                if not synced and chunk:
                    # Drop the partial frame in flight when recording started.
                    cut = chunk.find(b"\x00")
                    if cut < 0:
                        continue
                    chunk, synced = chunk[cut + 1 :], True
                # Hold back bytes after the last delimiter so a stop never cuts a frame.
                data = tail + chunk
                cut = data.rfind(b"\x00") + 1
                out.write(data[:cut])
                tail = data[cut:]
                written += cut
                if time.monotonic() - last_report >= 5.0:
                    last_report = time.monotonic()
                    print(f"  {last_report - t0:6.0f} s  {written / 1024:8.0f} KiB", file=sys.stderr)
        except KeyboardInterrupt:
            pass

    cap = capture.read_capture(bin_path)
    summary = summarize(cap)
    for name, up in ups.items():
        if name not in summary["sensors"]:
            print(f"warning: --up {name}: no such sensor in the capture", file=sys.stderr)
            continue
        summary["sensors"][name]["up"] = list(up)
    fw_config = cap.header.get("config")
    if fw_config is not None and fw_config != args.config:
        print(f"warning: --config {args.config} but the firmware reports {fw_config}", file=sys.stderr)
    meta = {
        "file": bin_path.name,
        "started": started.isoformat(timespec="seconds"),
        "test": args.test,
        "config": args.config,
        "session": args.session,
        "notes": args.notes,
        "power_up": args.power_up,
        "header": cap.header,
        "summary": summary,
    }
    bin_path.with_suffix(".json").write_text(json.dumps(meta, indent=2))

    entry: dict[str, typing.Any] = {
        "file": str(bin_path.relative_to(args.data_dir)),
        "started": started.isoformat(timespec="seconds"),
        "test": args.test,
        "config": args.config,
        "session": args.session,
        "notes": args.notes,
        **({"power_up": args.power_up} if args.power_up is not None else {}),
        "fw": summary["fw"],
        "uid": summary["uid"],
        "clean": summary["clean"],
        "sensors": {
            name: {k: v for k, v in s.items() if v is not None} for name, s in summary["sensors"].items()
        },
    }
    manifest.append(entry, path=args.data_dir / manifest.MANIFEST_PATH.name)
    _print_summary(summary)
    return 0


def inspect_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Summarize a capture file and its gaps.")
    parser.add_argument("path", type=pathlib.Path)
    args = parser.parse_args(argv)

    cap = capture.read_capture(args.path)
    _print_summary(summarize(cap))
    for s in cap.sensors.values():
        for start, end in s.gaps.missing_ranges[:10]:
            print(f"  {s.name}: missing samples {start}..{end - 1}")
    for text in cap.texts:
        print(f"  [text] {text}")
    return 0


def bench_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V1 bench-still metrics per sensor and axis.")
    parser.add_argument("paths", type=pathlib.Path, nargs="+")
    parser.add_argument("--trim", type=float, default=0.0, help="seconds dropped from the start")
    args = parser.parse_args(argv)

    for path in args.paths:
        cap = capture.read_capture(path)
        print(f"{path.name} config={cap.header.get('config', '?')}")
        for sensor_id in cap.sensors:
            m = bench.still_metrics(cap, sensor_id, trim_s=args.trim)
            print(
                f"  {m.sensor} {m.chip} {m.odr_hz:.2f} Hz {m.duration_s:.0f} s "
                f"temp {m.temp_mean_c:.2f} C drift {m.temp_drift_c:+.2f} C"
            )
            for kind, unit, axes in (("accel", "mg", m.accel), ("gyro", "mdps", m.gyro)):
                for name, a in zip("xyz", axes):
                    allan = " ".join(f"{t:g}s:{d * 1e3:.3f}" for t, d in zip(a.allan_tau_s, a.allan_dev))
                    print(
                        f"    {kind} {name}: bias {a.bias * 1e3:9.2f} std {a.std * 1e3:7.3f} {unit}  "
                        f"floor {np.sqrt(a.noise_floor) * 1e3:7.4f} low {np.sqrt(a.low_freq) * 1e3:7.4f} {unit}/rtHz  "
                        f"cutoff {a.cutoff_hz:6.1f} Hz  adev {allan}"
                    )
    return 0


def v1_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V1: fit on session 1, simulate, score the reported sessions.")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help="default data/v1, data/v1-palm")
    parser.add_argument("--sensor", default=v1.SENSOR, choices=sorted(v1.CHIP))
    parser.add_argument("--sims", type=int, default=3, help="sim runs per reported capture")
    parser.add_argument("--jobs", type=int, default=4, help="simulator runs at once")
    parser.add_argument("--all", action="store_true", help="list passing metrics too")
    parser.add_argument(
        "--transfer", metavar="TEST",
        help="score this test's captures (e.g. V1rig) against already fitted parameters, refitting nothing",
    )
    parser.add_argument("--params", type=pathlib.Path, default=None, help="with --transfer; default data/v1/<chip>.toml")
    args = parser.parse_args(argv)

    suffix = "" if args.sensor == v1.SENSOR else f"-{args.sensor}"
    if args.transfer:
        work_dir = args.work_dir or args.data_dir / f"{args.transfer.lower()}{suffix}"
        params = args.params or args.data_dir / "v1" / f"{v1.CHIP[args.sensor]}.toml"
        results = v1.run_transfer(
            args.data_dir, work_dir, params, sensor=args.sensor, test=args.transfer, sims_per_capture=args.sims, jobs=args.jobs
        )
    else:
        work_dir = args.work_dir or args.data_dir / f"v1{suffix}"
        results = v1.run(args.data_dir, work_dir, sensor=args.sensor, sims_per_capture=args.sims, jobs=args.jobs)
    rows = ["config,metric,real_mean,real_sd,sim_mean,window,pass"]
    for config, scores in results.items():
        for s in scores:
            rows.append(f"{config},{s.metric},{s.real_mean:.6g},{s.real_sd:.6g},{s.sim_mean:.6g},{s.window:.6g},{int(s.passed)}")
            if args.all or not s.passed:
                print(
                    f"  {'PASS' if s.passed else 'FAIL'} {config:12s} {s.metric:24s} real {s.real_mean:11.5g} "
                    f"(sd {s.real_sd:.3g})  sim {s.sim_mean:11.5g}  window {s.window:.3g}"
                )
    (work_dir / "scores.csv").write_text("\n".join(rows) + "\n")
    print(f"scores -> {work_dir / 'scores.csv'}")
    return 0


def compare_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Other simulators on the V1 or V2 still holds, scored like timusim.")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help="default data/compare[-v2][-palm]")
    parser.add_argument("--sensor", default=v1.SENSOR, choices=sorted(v1.CHIP))
    parser.add_argument("--test", default="V1", choices=("V1", "V2"))
    parser.add_argument("--sim", action="append", choices=compare.SIMULATORS, help="default all that are set up; repeat")
    parser.add_argument("--sims", type=int, default=3, help="V2: runs per reported hold")
    parser.add_argument("--jobs", type=int, default=4, help="simulator runs at once")
    args = parser.parse_args(argv)

    suffix = "" if args.sensor == v1.SENSOR else f"-{args.sensor}"
    work_dir = args.work_dir or compare.default_work_dir(args.data_dir, args.sensor, args.test)
    simulators = args.sim or compare.available()
    skipped = [s for s in compare.SIMULATORS if s not in simulators]
    if skipped and not args.sim:
        print(f"not set up, skipped: {', '.join(skipped)}")
    if args.test == "V2":
        found = compare.run_v2(
            args.data_dir, work_dir, simulators, sensor=args.sensor, sims_per_hold=args.sims, jobs=args.jobs
        )
        own_path = args.data_dir / f"v2{suffix}" / "scores.csv"
        both = ({"timusim": v2.read_scores(own_path)} if own_path.exists() else {}) | found
        (work_dir / "scores.csv").write_text("\n".join(compare.table_v2(both)) + "\n")
        for name, scores in both.items():
            print(f"{name:14s} {sum(s.passed for s in scores):>3d}/{len(scores)}")
        print(f"scores -> {work_dir / 'scores.csv'}")
        return 0
    results = compare.run(args.data_dir, work_dir, simulators, sensor=args.sensor, jobs=args.jobs)
    own_path = args.data_dir / f"v1{suffix}" / "scores.csv"
    own = compare.read_v1_scores(own_path) if own_path.exists() else None
    (work_dir / "scores.csv").write_text("\n".join(compare.table(results, own)) + "\n")
    names = (["timusim"] if own else []) + list(results)
    configs = next(iter(results.values()))
    print(f"{'config':12s} " + " ".join(f"{n:>10s}" for n in names))
    for config, scores in configs.items():
        counts = [sum(own.get((config, s.metric), (0, False))[1] for s in scores)] if own else []
        counts += [sum(s.passed for s in results[n][config]) for n in results]
        print(f"{config:12s} " + " ".join(f"{c:>6d}/{len(scores):<3d}" for c in counts))
    print(f"scores -> {work_dir / 'scores.csv'}")
    return 0


FIGURE_KINDS = ("filters", "psd", "allan", "long-allan", "compare", "v2")


def figures_main(argv: list[str] | None = None) -> int:
    import tomllib

    import matplotlib.pyplot as plt

    from timusim import figures, fit

    parser = argparse.ArgumentParser(description="Article figures from the captures in the manifest.")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help="timusim-v1 output")
    parser.add_argument("--sensor", default=v1.SENSOR, choices=sorted(v1.CHIP))
    parser.add_argument("--out", type=pathlib.Path, default=pathlib.Path("figures"))
    parser.add_argument("--format", default="pdf", choices=("pdf", "png", "svg"))
    parser.add_argument("--only", action="append", choices=FIGURE_KINDS, help="default all; repeat")
    args = parser.parse_args(argv)

    sensor = args.sensor
    kinds = set(args.only or FIGURE_KINDS)
    suffix = "" if sensor == v1.SENSOR else f"-{sensor}"
    work_dir = args.work_dir or args.data_dir / f"v1{suffix}"
    compare_dir = compare.default_work_dir(args.data_dir, sensor)
    prefix = f"v1{suffix}"
    entries = v1.entries(args.data_dir, sensor=sensor)
    args.out.mkdir(parents=True, exist_ok=True)
    written = []

    def save(name: str, fig: typing.Any) -> None:
        path = args.out / f"{name}.{args.format}"
        fig.savefig(path)
        plt.close(fig)
        written.append(path)

    if "filters" in kinds:
        for e in entries:
            if e.session != 1:
                continue
            cap = capture.read_capture(e.path)
            save(f"{prefix}-{e.config}-accel-filter", figures.filter_shape(cap, kind="accel", sensor=sensor))
            save(f"{prefix}-{e.config}-gyro-filter", figures.filter_shape(cap, kind="gyro", sensor=sensor))
    # v1.run and compare.run number each config's reported captures in manifest order.
    reported: dict[str, list[v1.Entry]] = {}
    for e in entries:
        if e.session != 1:
            reported.setdefault(e.config, []).append(e)
    if kinds & {"psd", "compare"}:
        for config, es in sorted(reported.items()):
            for k, e in enumerate(es):
                real = capture.read_capture(e.path)
                sim_path = work_dir / f"{config}-{k}-0.bin"
                sim_cap = capture.read_capture(sim_path) if sim_path.exists() else None
                if "psd" in kinds:
                    save(f"{prefix}-{config}-s{e.session}-psd", figures.psd(real, sim_cap, sensor=sensor))
                others = [(n, compare_dir / n / f"{config}-{k}.npz") for n in compare.SIMULATORS]
                if "compare" in kinds and sim_cap is not None and any(out.exists() for _, out in others):
                    lines = [
                        figures.series(real, "real", figures.REAL, "-", sensor=sensor, trim_s=compare.TRIM_S),
                        figures.series(sim_cap, "timusim", figures.SIM, "--", sensor=sensor, trim_s=compare.TRIM_S),
                    ]
                    lines += [(n, *figures.OTHER[n], *compare.load(out)) for n, out in others if out.exists()]
                    save(f"{prefix}-{config}-s{e.session}-compare-psd", figures.psd_lines(lines))
                    if k == 0:
                        save(f"{prefix}-{config}-compare-allan", figures.allan_lines(lines))
    if "allan" in kinds:
        for config, es in sorted(reported.items()):
            sims = sorted(work_dir.glob(f"{config}-*.bin"))
            if config == "long" or not sims:
                continue
            fig = figures.allan(
                [capture.read_capture(e.path) for e in es], [capture.read_capture(p) for p in sims], sensor=sensor
            )
            save(f"{prefix}-{config}-allan", fig)
    params_path = work_dir / f"{v1.CHIP[sensor]}.toml"
    longs = [e for e in entries if e.config == "long"]
    if "long-allan" in kinds and longs and params_path.exists():
        params = tomllib.loads(params_path.read_text())
        # The output white level the drift was fitted on, from the same session-1 default captures.
        defaults = [e.path for e in entries if e.session == 1 and e.config == "default"]
        fits = fit.fit_files(defaults, sensor, allan=False)
        white = {
            kind: [float(np.median([getattr(f, kind)[i].white_low for f in fits])) for i in range(3)]
            for kind in ("accel", "gyro")
        }
        caps = (capture.read_capture(e.path) for e in longs)
        save(f"{prefix}-long-allan", figures.allan_model(caps, params, white=white, sensor=sensor))
    v2_scores = args.data_dir / f"v2{suffix}" / "scores.csv"
    if "v2" in kinds and v2_scores.exists():
        own = {"timusim": v2.read_scores(v2_scores)}
        save(f"v2{suffix}-scores", figures.v2_scores(own))
        compare_v2 = compare.default_work_dir(args.data_dir, sensor, "V2")
        others = {n: v2.read_scores(compare_v2 / n / "scores.csv") for n in compare.SIMULATORS if (compare_v2 / n / "scores.csv").exists()}
        if others:
            save(f"v2{suffix}-compare", figures.v2_scores(own | others))
    for path in written:
        print(path)
    return 0


def v2_main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="V2: fit accel gain on session 1, simulate, score the reported holds.")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help="default data/v2, data/v2-palm")
    parser.add_argument("--sensor", default=v1.SENSOR, choices=sorted(v1.CHIP))
    parser.add_argument("--params", type=pathlib.Path, default=None, help="V1 parameter file; default the V1 work dir's")
    parser.add_argument("--sims", type=int, default=1, help="sim runs per reported hold")
    args = parser.parse_args(argv)

    suffix = "" if args.sensor == v1.SENSOR else f"-{args.sensor}"
    work_dir = args.work_dir or args.data_dir / f"v2{suffix}"
    params = args.params or args.data_dir / f"v1{suffix}" / f"{v1.CHIP[args.sensor]}.toml"
    results = v2.run(args.data_dir, work_dir, params, sensor=args.sensor, sims_per_hold=args.sims)
    for r in results:
        print(f"  {r.position:8s} {r.metric:13s} real {r.real_mean:9.4g} sd {r.real_sd:8.3g} sim {r.sim_mean:9.4g}"
              f"  {'PASS' if r.passed else 'FAIL'} window {r.window:.3g}")
    (work_dir / "scores.csv").write_text(v2.scores_csv(results))
    print(f"V2 {sum(r.passed for r in results)}/{len(results)} scores pass; scores in {work_dir / 'scores.csv'}")
    return 0


def _rig_main(test: str, argv: list[str] | None) -> int:
    module = v3 if test == "V3" else v4
    parser = argparse.ArgumentParser(description=f"{test}: fit on session 1, replay the reported captures, score them.")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--work-dir", type=pathlib.Path, default=None, help=f"default data/{test.lower()}")
    parser.add_argument(
        "--check", action="store_true",
        help="score session 1 against itself, to exercise the pipeline; not a result (writes to <work-dir>/check)",
    )
    args = parser.parse_args(argv)

    work_dir = args.work_dir or args.data_dir / test.lower()
    if args.check:
        work_dir = work_dir / "check"
    results = module.run(args.data_dir, work_dir, report_sessions=(lambda s: s == 1) if args.check else (lambda s: s > 1))
    for r in results:
        where = getattr(r, "config", None) or getattr(r, "orientation", "") or "base_rotation"
        verdict = "-" if r.passed is None else "PASS" if r.passed else "FAIL"
        print(f"  {r.sensor:9s} {where:16s} {r.metric:26s} real {r.real_mean:9.4g} sd {r.real_sd:8.3g} "
              f"sim {r.sim_mean:9.4g}  {verdict:4s} window {r.window:.3g} n {r.n_real}")
    (work_dir / "scores.csv").write_text(module.scores_csv(results))
    scored = [r for r in results if r.passed is not None]
    print(f"{test}{' check' if args.check else ''} {sum(bool(r.passed) for r in scored)}/{len(scored)} scores pass; "
          f"scores in {work_dir / 'scores.csv'}")
    return 0


def v3_main(argv: list[str] | None = None) -> int:
    return _rig_main("V3", argv)


def v4_main(argv: list[str] | None = None) -> int:
    return _rig_main("V4", argv)

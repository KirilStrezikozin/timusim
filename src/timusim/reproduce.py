"""The analysis from the manifest's captures to every score file, in dependency order: V1 fits
before V2, V2 before the rig tests (palm_lsm9's rig parameters are its V2 output), and the
other-simulator comparison and figures last. The comparison runs 2 simulators at once: 4 MATLABs
froze the laptop on 2026-10-26."""

import pathlib
import shutil
import time
import typing

from timusim import cli, manifest

Step = tuple[str, typing.Callable[[list[str] | None], int], list[str], str]


def steps(data_dir: pathlib.Path) -> list[Step]:
    d = str(data_dir)
    lsm9, lsm6 = str(data_dir / "v1" / "lsm9ds1.toml"), str(data_dir / "v1-palm" / "lsm6dsl.toml")
    return [
        ("v1-finger", cli.v1_main, ["--data-dir", d, "--sensor", "finger"], "v1"),
        ("v1-palm", cli.v1_main, ["--data-dir", d, "--sensor", "palm"], "v1-palm"),
        ("v2-finger", cli.v2_main, ["--data-dir", d, "--sensor", "finger"], "v2"),
        ("v2-palm", cli.v2_main, ["--data-dir", d, "--sensor", "palm"], "v2-palm"),
        ("v2-palm_lsm9", cli.v2_main, ["--data-dir", d, "--sensor", "palm_lsm9", "--params", lsm9], "v2-palm_lsm9"),
        ("v1rig-finger", cli.v1_main, ["--data-dir", d, "--transfer", "V1rig", "--sensor", "finger"], "v1rig"),
        ("v1rig-palm", cli.v1_main, ["--data-dir", d, "--transfer", "V1rig", "--sensor", "palm", "--params", lsm6], "v1rig-palm"),
        ("v1rig-palm_lsm9", cli.v1_main, ["--data-dir", d, "--transfer", "V1rig", "--sensor", "palm_lsm9"], "v1rig-palm_lsm9"),
        ("v4-check", cli.v4_main, ["--data-dir", d, "--check"], "v4/check"),
        ("v3-check", cli.v3_main, ["--data-dir", d, "--check"], "v3/check"),
        ("v4", cli.v4_main, ["--data-dir", d], "v4"),
        ("v3", cli.v3_main, ["--data-dir", d], "v3"),
        ("compare-finger", cli.compare_main, ["--data-dir", d, "--sensor", "finger", "--jobs", "2"], "compare"),
        ("compare-palm", cli.compare_main, ["--data-dir", d, "--sensor", "palm", "--jobs", "2"], "compare-palm"),
        ("compare-v2-finger", cli.compare_main, ["--data-dir", d, "--sensor", "finger", "--test", "V2", "--jobs", "2"], "compare-v2"),
        ("compare-v2-palm", cli.compare_main, ["--data-dir", d, "--sensor", "palm", "--test", "V2", "--jobs", "2"], "compare-v2-palm"),
    ]


def run(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    parser.add_argument("--only", nargs="+", help="these steps, in chain order")
    parser.add_argument("--skip", nargs="+", default=[], help="e.g. the compare-* steps when the other simulators are not set up")
    parser.add_argument("--snapshot", type=pathlib.Path, help="copy every step's scores.csv here, as <step>.csv")
    parser.add_argument("--figures", type=pathlib.Path, help="then draw the figures (pdf and png) into this directory")
    parser.add_argument("--list", action="store_true", help="print the steps and exit")
    args = parser.parse_args(argv)

    chain = [s for s in steps(args.data_dir) if (not args.only or s[0] in args.only) and s[0] not in args.skip]
    if args.list:
        for name, _, step_argv, _ in chain:
            print(name, " ".join(step_argv))
        return 0
    for name, main, step_argv, out in chain:
        started = time.monotonic()
        print(f"=== {time.strftime('%H:%M:%S')} {name}", flush=True)
        main(step_argv)
        print(f"=== {name} took {(time.monotonic() - started) / 60:.1f} min", flush=True)
        if args.snapshot:
            args.snapshot.mkdir(parents=True, exist_ok=True)
            shutil.copy(args.data_dir / out / "scores.csv", args.snapshot / f"{name}.csv")
    if args.figures:
        for fmt in ("pdf", "png"):
            for sensor in ("finger", "palm"):
                cli.figures_main(["--data-dir", str(args.data_dir), "--sensor", sensor, "--out", str(args.figures), "--format", fmt])
    return 0

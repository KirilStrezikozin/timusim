"""Unattended capture protocols: a settle, a config sweep that reflashes the board for each
config, then long holds, stopping before the next capture once data/STOP exists. To stop
mid-capture, create data/STOP and SIGINT the timusim-capture process: it writes its summary."""

import pathlib
import re
import subprocess
import sys
import time
import typing

from timusim import manifest

FIRMWARE_DIR = pathlib.Path(__file__).resolve().parents[2] / "examples" / "stm32-capture"
# One LSM9DS1 setting on both finger and palm_lsm9 and one palm setting per env.
RIG_SWEEP = (
    "rig",
    "trio_low_range_low_range",
    "trio_238hz_833hz",
    "trio_119hz_208hz",
    "trio_lpf2_strong_lp1_aggressive",
    "trio_lpf2_light_lp1_light",
    "trio_952hz_208hz",
    "trio_mid_range_lp1_light",
    "trio_16g_833hz",
    "trio_xl_lp_off_xl_div50",
    "trio_xl_div50_lp1_aggressive",
)
# If both LSM9DS1s at 952 Hz drop samples on their shared bus, each is recorded on its own.
SPLIT = {"trio_952hz_208hz": ("pair_952hz_208hz", "palm_lsm9_952hz")}
RIG_ENV = "rig"


def log(msg: str) -> None:
    print(f"=== {time.strftime('%H:%M:%S')} {msg}", flush=True)


def env_config(env: str, ini: pathlib.Path = FIRMWARE_DIR / "platformio.ini") -> str:
    """The CAPTURE_CONFIG the env builds with, which the capture is recorded under."""
    block = re.search(rf"^\[env:{re.escape(env)}\]\n(.*?)(?=^\[|\Z)", ini.read_text(), re.M | re.S)
    if block is None:
        raise ValueError(f"no env {env} in {ini}")
    config = re.search(r'-DCAPTURE_CONFIG=\\"([^"\\]+)\\"', block.group(1))
    return config.group(1) if config else "default"


def flash(env: str) -> None:
    subprocess.run(["pio", "run", "-s", "-e", env, "-t", "upload"], cwd=FIRMWARE_DIR, check=True, stdout=subprocess.DEVNULL)


def dropped(entry: dict[str, typing.Any]) -> list[str]:
    """Sensors that lost more than a second of samples or 0.1% of them."""
    return [
        n for n, s in entry["sensors"].items() if s["missing_samples"] > max(s["odr_measured_hz"], 1e-3 * s["samples"])
    ]


class Protocol:
    def __init__(self, data_dir: pathlib.Path, test: str, power_up: int, notes: str, port: str) -> None:
        self.data_dir, self.test, self.power_up, self.notes, self.port = data_dir, test, power_up, notes, port
        self.stop_file = data_dir / "STOP"

    def stopped(self) -> bool:
        if self.stop_file.exists():
            log(f"{self.stop_file} found, stopping")
            return True
        return False

    def capture(self, config: str, session: int, seconds: float, notes: str) -> dict[str, typing.Any]:
        before = len(manifest.load(manifest.path(self.data_dir)))
        subprocess.run(
            [
                str(pathlib.Path(sys.executable).parent / "timusim-capture"),
                *("--test", self.test, "--config", config, "--session", str(session), "--seconds", str(seconds)),
                *("--power-up", str(self.power_up), "--port", self.port, "--data-dir", str(self.data_dir)),
                *("--notes", f"{self.notes}; board power-up {self.power_up}; {notes}"),
            ],
            check=False,
        )
        entries = manifest.load(manifest.path(self.data_dir))
        if len(entries) == before:
            raise RuntimeError(f"{config}: the capture wrote no manifest entry")
        return entries[-1]

    def sweep(self, envs: typing.Sequence[str], session: int, minutes: float, warmup_s: float) -> bool:
        """Each env flashed and recorded once, again if it drops samples, then split if SPLIT has it.
        False once stopped."""
        for env in envs:
            for attempt in (1, 2):
                if self.stopped():
                    return False
                log(f"{env}, session {session}, attempt {attempt}")
                flash(env)
                time.sleep(warmup_s)
                entry = self.capture(env_config(env), session, 60 * minutes, "chip soft reset by flash between captures")
                bad = dropped(entry)
                log(f"{entry['file']}: " + (f"dropped samples on {', '.join(bad)}" if bad else "clean"))
                if not bad:
                    break
            else:
                if env in SPLIT and not self.sweep(SPLIT[env], session, minutes, warmup_s):
                    return False
        return True

    def long_holds(self, session: int, minutes: float, count: int | None) -> None:
        """Back-to-back holds without reflash, `count` of them or until stopped."""
        flash(RIG_ENV)
        time.sleep(60)
        n = 0
        while (count is None or n < count) and not self.stopped():
            log(f"long hold, session {session + n}")
            self.capture("long", session + n, 60 * minutes, f"{minutes:g}-min segments back to back without reflash")
            n += 1


def run(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power-up", type=int, required=True, help="board power-up count")
    parser.add_argument("--test", default="V1rig")
    parser.add_argument("--notes", required=True, help="setup, written into every capture's notes")
    parser.add_argument("--settle-minutes", type=float, default=15)
    parser.add_argument("--sweep-session", type=int, default=None, help="record the rig config sweep as this session")
    parser.add_argument("--sweep-minutes", type=float, default=10)
    parser.add_argument("--warmup-s", type=float, default=60, help="after each flash")
    parser.add_argument("--envs", nargs="+", default=list(RIG_SWEEP), help="sweep envs; default the rig sweep")
    parser.add_argument("--long-session", type=int, default=None, help="first long-hold session; none: no long holds")
    parser.add_argument("--long-minutes", type=float, default=240)
    parser.add_argument("--long-count", type=int, default=None, help="default until data/STOP")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--data-dir", type=pathlib.Path, default=manifest.DATA_DIR)
    args = parser.parse_args(argv)

    for env in args.envs:
        env_config(env)
    p = Protocol(args.data_dir, args.test, args.power_up, args.notes, args.port)
    log(f"power-up {args.power_up}, settling {args.settle_minutes:g} min")
    time.sleep(60 * args.settle_minutes)
    if args.sweep_session is not None and not p.sweep(args.envs, args.sweep_session, args.sweep_minutes, args.warmup_s):
        return 0
    if args.long_session is not None:
        p.long_holds(args.long_session, args.long_minutes, args.long_count)
    log("protocol done")
    return 0


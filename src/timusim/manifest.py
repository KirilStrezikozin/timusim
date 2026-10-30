"""data/manifest.toml: one [[capture]] table per recorded file, checked into git."""

import pathlib
import tomllib
import typing

import tomli_w

DATA_DIR = pathlib.Path(__file__).resolve().parents[2] / "data"
MANIFEST_PATH = DATA_DIR / "manifest.toml"


def load(path: pathlib.Path = MANIFEST_PATH) -> list[dict[str, typing.Any]]:
    if not path.exists():
        return []
    with path.open("rb") as f:
        return list(tomllib.load(f).get("capture", []))


def append(entry: dict[str, typing.Any], *, path: pathlib.Path = MANIFEST_PATH) -> None:
    entries = load(path)
    entries.append(entry)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        tomli_w.dump({"capture": entries}, f)

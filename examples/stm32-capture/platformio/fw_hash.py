import hashlib
import pathlib
import subprocess

Import("env")  # noqa: F821

# What the firmware is built from; changes elsewhere in the repo do not make it dirty.
SOURCES = ("examples/stm32-capture", "lib", "external")


def git(*args: str, ok: tuple[int, ...] = (0,)) -> str:
    try:
        run = subprocess.run(["git", *args], cwd=env["PROJECT_DIR"], text=True, capture_output=True)  # noqa: F821
    except OSError:
        return ""
    return run.stdout if run.returncode in ok else ""


top = pathlib.Path(git("rev-parse", "--show-toplevel").strip() or ".")
paths = [f":(top){p}" for p in SOURCES]
fw_hash = git("rev-parse", "--short", "HEAD").strip() or "none"
diff = git("diff", "HEAD", "--", *paths)
for f in git("ls-files", "--others", "--exclude-standard", "--", *paths).splitlines():
    diff += git("diff", "--no-index", "/dev/null", str(top / f), ok=(0, 1))
if diff:
    # The diff is kept next to the captures, so a dirty build can be rebuilt from its commit.
    fw_hash += "-dirty-" + hashlib.sha1(diff.encode()).hexdigest()[:8]
    saved = top / "data" / "firmware" / f"{fw_hash}.diff"
    saved.parent.mkdir(parents=True, exist_ok=True)
    if not saved.exists():
        saved.write_text(diff)

env.Append(CPPDEFINES=[("FW_HASH", env.StringifyMacro(fw_hash))])  # noqa: F821

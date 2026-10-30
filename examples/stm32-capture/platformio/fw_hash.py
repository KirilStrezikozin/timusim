import subprocess

Import("env")  # noqa: F821


def git(*args: str) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=env["PROJECT_DIR"], text=True).strip()  # noqa: F821
    except (OSError, subprocess.CalledProcessError):
        return ""


fw_hash = git("rev-parse", "--short", "HEAD") or "none"
if git("status", "--porcelain"):
    fw_hash += "-dirty"

env.Append(CPPDEFINES=[("FW_HASH", env.StringifyMacro(fw_hash))])  # noqa: F821

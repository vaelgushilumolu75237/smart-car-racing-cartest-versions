from difflib import unified_diff
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[4]
BASE = ROOT / "tmp/openmv-v5.0.1-h750-audit"
BASE_COMMIT = "631681e5ac5ab332c920b6bf9cd3dba5cf299eac"
OVERLAY = Path(__file__).resolve().parent / "overlay"
OUT = Path(__file__).resolve().parent / "jpeg_preview_guard.patch"
FILES = (
    "ports/stm32/stm_jpeg.c",
    "lib/imlib/imlib.h",
    "ports/stm32/jpeg_preview_guard_cleanup.h",
)

actual_commit = subprocess.check_output(
    ["git", "-C", str(BASE), "rev-parse", "HEAD"], text=True).strip()
if actual_commit != BASE_COMMIT:
    raise SystemExit(f"Unexpected OpenMV base commit: {actual_commit}")


parts = []
for relative in FILES:
    target = OVERLAY / relative
    tracked = subprocess.run(
        ["git", "-C", str(BASE), "cat-file", "-e",
         f"{BASE_COMMIT}:{relative}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    old = (subprocess.check_output(
        ["git", "-C", str(BASE), "show", f"{BASE_COMMIT}:{relative}"])
        .decode("utf-8").splitlines(keepends=True)
        if tracked.returncode == 0 else [])
    new = target.read_text(encoding="utf-8").splitlines(keepends=True)
    if old == new and tracked.returncode == 0:
        continue

    parts.append(f"diff --git a/{relative} b/{relative}\n")
    if tracked.returncode != 0:
        parts.append("new file mode 100644\n")
    parts.extend(unified_diff(old, new,
                              fromfile=f"a/{relative}" if tracked.returncode == 0 else "/dev/null",
                              tofile=f"b/{relative}",
                              n=3))

OUT.write_text("".join(parts), encoding="utf-8", newline="\n")
print(OUT)

"""Open this file in CanMV IDE after copying the whole package to this folder."""
import sys

PACKAGE_ROOT = "/sdcard/cartest-k230"
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)


def run():
    import k230_hal
    import main as vision
    if not callable(getattr(vision, "_main_fast", None)):
        raise RuntimeError("Wrong cached main module; restart IDE runtime before launching CarTest")
    k230_hal.run(vision.main)


if __name__ == "__main__":
    run()


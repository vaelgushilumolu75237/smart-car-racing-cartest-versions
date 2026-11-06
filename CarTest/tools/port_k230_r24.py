"""Reproducibly adapt the preserved r24 runtime into the isolated K230 tree.

Does not edit source trees supplied by --baseline; no board operations.
"""
import argparse
import hashlib
import json
import re
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline", type=Path, required=True)
    args = parser.parse_args()
    target = Path(__file__).resolve().parents[1] / "openmv"
    changes = {}
    for source in args.baseline.glob("*.py"):
        text = source.read_text(encoding="utf-8")
        # Compiler-recognized @micropython.native/viper/asm_thumb must also be
        # renamed: merely replacing the imported module does not disable emitters.
        if "import micropython" in text:
            text = text.replace("    import micropython", "    raise ImportError('K230 portable reference backend')")
            text = re.sub(r"@micropython\.(native|viper|asm_thumb)", "@_k230_identity", text)
            text = "from k230_portable import identity as _k230_identity\n" + text
            text = text.replace('"cpython-shim"', '"k230-portable-reference"')
        # Never attempt to import the ARM-only optional native extensions.
        text = re.sub(r"    import cartest_accel(?: as \w+)?", "    raise ImportError('ARM-only extension')", text)
        text = text.replace('__import__("cartest_path_accel")', "None")
        if source.name == "main.py":
            text = text.replace("import pyb\nimport sensor\n", "from k230_hal import pyb, sensor\n")
            old = '    """Start the grayscale path and return the camera\'s reported ID."""\n'
            assert old in text
            text = text.replace(old, old + '    configure = getattr(sensor_module, "configure", None)\n    if callable(configure):\n        return configure()\n', 1)
            text = text.replace('    calibration_type = ring_width_calibration.RingWidthCalibration\n    collector = calibration_type()\n    expected_sensor_id = ring_width_calibration.SENSOR_ID\n    sensor_id = _configure_gray_sensor(sensor, fast=True)',
                                '    sensor_id = _configure_gray_sensor(sensor, fast=True)\n    calibration_type = ring_width_calibration.RingWidthCalibration\n    collector = calibration_type()\n    expected_sensor_id = ring_width_calibration.SENSOR_ID')
            text = text.replace('    if exposure_cap_us is not None and not fixture_mode:',
                                '    if getattr(sensor, "EXPOSURE_CONTROL_SUPPORTED", True) is False:\n        exposure_cap_us = None\n    if exposure_cap_us is not None and not fixture_mode:')
            text = text.replace('if __name__ == "__main__":\n    main()',
                                'if __name__ == "__main__":\n    from k230_hal import run\n    run(main)')
            text = text.replace('"uart=3@115200/8N1 exposure_us=%s "', '"uart=1@115200/8N1 exposure_us=%s "')
            text = text.replace('config.UART_ID != 3', 'config.UART_ID != 1')
            text = text.replace('V7 UART must remain UART3, 115200 8N1', 'K230 factory UART must remain UART1, 115200 8N1')
            text = text.replace('    print("V7_BUILD=%s backend=fast source=%s route=%s v7_api=%d single_api=%d "',
                                '    print("K230_PORT=%s upstream=%s" % (config.K230_PORT_BUILD, config.TRACK_PACKAGE_BUILD_ID))\n    print("V7_BUILD=%s backend=fast source=%s route=%s v7_api=%d single_api=%d "')
            # Do not print a false assertion that motors could never have run
            # after a real UART session. The camera has no direct motor control.
            text = text.replace('board_storage_write=0 h743_uart_receive_verified=0 motors=0',
                                'h743_uart_receive_verified=0 motor_state=not_observed')
        if source.name == "config.py":
            text += '\n# K230 platform overrides; r24 geometry/control and gates above remain.\nimport k230_settings as _board\nK230_PORT_BUILD = _board.PORT_BUILD\nUART_ID = _board.UART_ID\nMIRROR_X = _board.MIRROR_X\nFLIP_Y = _board.FLIP_Y\n# AUTO_EXPOSURE_MAX_US remains an upstream request, not applied by K230.\n'
        (target / source.name).write_text(text, encoding="utf-8", newline="\n")
        changes[source.name] = {
            "baseline_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "ported_sha256": hashlib.sha256(text.encode()).hexdigest(),
        }
    (target.parent / "docs" / "K230_PORT_TRANSFORM.json").write_text(
        json.dumps(changes, indent=2) + "\n", encoding="utf-8")
    # A deliberately not-ready placeholder, never the H750 private width map.
    (target / "ring_width_calibration.json").write_text(json.dumps({
        "format_version": 3, "ready": False,
        "reason": "K230_RECALIBRATION_REQUIRED", "camera_binding": None,
    }, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

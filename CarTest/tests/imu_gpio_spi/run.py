"""Test production GPIO SPI against an edge-level simulated register slave."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT.parent / "validation"
COMPILER = Path(os.environ.get("CARTEST_ZIG", r"D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe"))


def main():
    report = OUT / "IMU_GPIO_SPI.json"
    if report.exists():
        report.unlink()
    with tempfile.TemporaryDirectory(prefix="cartest-imu-spi-") as directory:
        exe = Path(directory) / "gpio-spi.exe"
        subprocess.run([str(COMPILER), "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                        "-DCARTEST_GYRO350_ENABLED=1", "-Itests/imu_gpio_spi", "-IInc", "-I.",
                        "tests/imu_gpio_spi/test_gpio_spi.c", "-o", str(exe)],
                       cwd=ROOT, check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True, check=True, timeout=10)
    sources = ["App/icm42688_stm32.c", "Inc/icm42688_stm32.h", "Inc/icm42688.h",
               "Inc/gyro350_config.h", "tests/imu_gpio_spi/test_gpio_spi.c",
               "tests/imu_gpio_spi/stm32h7xx_hal.h", "tests/imu_gpio_spi/run.py"]
    data = {"passed": True, "summary": result.stdout.strip(),
            "scope": "production GPIO Mode 0 driver compiled directly with simulated peripheral registers",
            "cases": ["MSB-first F5 read and WHO_AM_I 47", "register write mask and data",
                      "maximum 16-byte payload", "invalid lengths/null pointers",
                      "masked interrupt rejection", "HCLK rejection", "stopped DWT at open",
                      "stopped DWT during transfer", "transaction deadline with unchanged output",
                      "recovery after failure", "cycle counter wraparound"],
            "invariants": ["only CS/SCK/MOSI outputs change", "MISO configured as input",
                           "clock low at select", "minimum 128-cycle low/high phases",
                           "CS high SCK/MOSI low after success and timeout"],
            "tested_source_sha256": {s: hashlib.sha256((ROOT/s).read_bytes()).hexdigest() for s in sources},
            "hardware_verified": False, "hardware_actions": "none"}
    OUT.mkdir(exist_ok=True)
    report.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print(result.stdout.strip())


if __name__ == "__main__":
    main()

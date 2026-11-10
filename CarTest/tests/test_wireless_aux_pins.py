from __future__ import annotations

import re
import hashlib
import json
import subprocess
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Src" / "main.c"
ZIG = Path(r"D:\Codex\T2\T2\CarTest\logs\diagnostics\host-compiler\ziglang\zig.exe")


def extract_function(source: str, name: str) -> str:
    match = re.search(
        r"static\s+void\s+" + re.escape(name) + r"\s*\(\s*void\s*\)\s*\{",
        source,
    )
    if match is None:
        raise AssertionError(f"missing production function: {name}")
    depth = 0
    for index in range(match.end() - 1, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[match.start():index + 1]
    raise AssertionError(f"unterminated production function: {name}")


PRELUDE = r"""
#include <stdint.h>
#include <stdio.h>

typedef struct { uint32_t AHB4ENR; } rcc_t;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t marker; } gpio_t;

static rcc_t rcc;
static gpio_t gpioa;
static GPIO_InitTypeDef captured;
static uint32_t clock_calls;
static uint32_t init_calls;

#define GPIOA (&gpioa)
#define GPIO_PIN_4 (1u << 4u)
#define GPIO_PIN_5 (1u << 5u)
#define APP_WIRELESS_RTS_GPIO_PIN GPIO_PIN_4
#define APP_WIRELESS_CMD_GPIO_PIN GPIO_PIN_5
#define RCC_AHB4ENR_GPIOAEN (1u << 0u)
#define GPIO_MODE_INPUT 0u
#define GPIO_PULLUP 1u
#define GPIO_SPEED_FREQ_LOW 0u
#define __HAL_RCC_GPIOA_CLK_ENABLE() do { rcc.AHB4ENR |= RCC_AHB4ENR_GPIOAEN; ++clock_calls; } while (0)

static void HAL_GPIO_Init(gpio_t *port, const GPIO_InitTypeDef *config)
{
    (void)port;
    captured = *config;
    ++init_calls;
}
"""


def main() -> int:
    source_text = SOURCE.read_text(encoding="utf-8")
    function = extract_function(source_text, "wireless_control_pins_init")
    expected = (
        "#if CARTEST_WIRELESS_ENABLED\n"
        "    wireless_control_pins_init();\n"
        "#endif"
    )
    if expected not in source_text:
        raise AssertionError("wireless pin initialization is not guarded")
    if source_text.index("early_motor_pins_low();") >= source_text.index(
        "wireless_control_pins_init();"
    ):
        raise AssertionError("wireless pins must initialize after safe motor pins")
    harness = PRELUDE + function + r"""
int main(void)
{
    wireless_control_pins_init();
    if (clock_calls != 1u || init_calls != 1u) return 1;
    if (rcc.AHB4ENR != RCC_AHB4ENR_GPIOAEN) return 2;
    if (captured.Pin != (GPIO_PIN_4 | GPIO_PIN_5)) return 3;
    if (captured.Mode != GPIO_MODE_INPUT || captured.Pull != GPIO_PULLUP) return 4;
    if (captured.Speed != GPIO_SPEED_FREQ_LOW || captured.Alternate != 0u) return 5;
    puts("wireless RTS/CMD internal pullups: PASS");
    return 0;
}
"""
    with tempfile.TemporaryDirectory(prefix="cartest-wireless-aux-") as temp:
        temp_path = Path(temp)
        c_path = temp_path / "wireless_aux_pins.c"
        exe_path = temp_path / "wireless_aux_pins.exe"
        c_path.write_text(harness, encoding="utf-8")
        build = subprocess.run(
            [str(ZIG), "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
             str(c_path), "-o", str(exe_path)],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=False,
        )
        if build.returncode:
            print(build.stdout, end="")
            print(build.stderr, end="")
            return build.returncode
        run = subprocess.run([str(exe_path)], capture_output=True, text=True,
                             check=False)
        print(run.stdout, end="")
        print(run.stderr, end="")
        if run.returncode == 0:
            evidence = ROOT.parent / 'validation/WIRELESS_AUX_PINS.json'
            evidence.parent.mkdir(exist_ok=True)
            evidence.write_text(json.dumps({
                'passed': True, 'hardware_actions': 'none',
                'scope': 'Extracted production RTS/CMD input-pullup initialization; host execution only',
                'runner': 'python tests/test_wireless_aux_pins.py',
                'source_sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                                  for name in ('Src/main.c', 'tests/test_wireless_aux_pins.py')},
            }, indent=2), encoding='utf-8')
        return run.returncode


if __name__ == "__main__":
    raise SystemExit(main())

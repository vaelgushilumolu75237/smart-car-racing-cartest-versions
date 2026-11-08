"""Run the production USART1 TX arbitration with a bounded fake FIFO.

Only the TDR store is instrumented. No board connection or motor commands.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def run_source(source, zig):
    start = source.index("    /* With FIFOEN,", source.index("static void service_reverse_control(uint32_t now_ms)\n{"))
    end = source.index("    car_reverse_control_sequence =", start)
    block = source[start:end]
    block, stores = re.subn(r"USART1->TDR\s*=\s*byte;", "mock_write(byte);", block)
    if stores != 3:
        raise ValueError("TX instrumentation boundary changed")
    harness = r'''
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "reverse_control.h"
#define CARTEST_WIRELESS_ENABLED 1
#define USART_ISR_TXE_TXFNF 1u
static struct { uint32_t ISR; } regs;
#define USART1 (&regs)
static car_reverse_control_tx_t reverse_control_tx;
static unsigned reverse_keep_notified, car_control_state, used, capacity;
static unsigned car_camera_uart_tx_bytes;
static uint8_t wire[20000];
static unsigned wire_count, wireless_pos, wireless_pending;
static void mock_write(uint8_t b) {
    assert(used < capacity && wire_count < sizeof(wire));
    wire[wire_count++] = b; ++used;
    if (used == capacity) regs.ISR = 0;
}
static void car_control_reverse_control_keep_started(unsigned *ignored) {(void)ignored;}
static unsigned car_wireless_camera_tx_byte(uint8_t *out, unsigned start) {
    if (!wireless_pos && (!start || !wireless_pending)) return 0;
    *out = wireless_pos == 0 ? 0xd3 : wireless_pos == 1 ? 0x91 : 0;
    ++wireless_pos;
    if (wireless_pos == 32) {wireless_pos=0;wireless_pending=0;}
    return 1;
}
static void pump(uint32_t now_ms, uint8_t keep_permitted) {
    uint8_t byte;
''' + block + r'''
}
static void scenario(unsigned step_ms, unsigned fifo_capacity) {
    unsigned now, max_used=0, v1=0, profiles=0, hello=0, i=0;
    wire_count=0;wireless_pos=0;wireless_pending=1;
    car_reverse_control_init(&reverse_control_tx,0);
    car_reverse_control_set_profile_state(&reverse_control_tx,3,0,0,0);
    for(now=0;now<5000;now+=step_ms) {
        used=0;capacity=fifo_capacity;regs.ISR=USART_ISR_TXE_TXFNF;
        pump(now,1);
        if(used>max_used)max_used=used;
        assert(used<=16 && used<=capacity);
    }
    while(i<wire_count) {
        unsigned n;
        if(wire[i]==0xd3) {
            assert(i+1<wire_count && wire[i+1]==0x91);
            if(i+32>wire_count)break;
            ++hello;i+=32;continue;
        }
        if(i+3>wire_count)break;
        assert(wire[i]==0xa6 && wire[i+1]==0x5a);
        n=wire[i+2]==1?8:20;
        if(i+n>wire_count)break;
        assert(car_reverse_control_crc16(wire+i,n-2)==
               (uint16_t)(wire[i+n-2]|((uint16_t)wire[i+n-1]<<8)));
        if(n==20)++profiles;else ++v1;
        i+=n;
    }
    printf("%u %u %u %u %u %u\n",step_ms,fifo_capacity,v1,profiles,hello,max_used);
}
int main(void) { scenario(1,16);scenario(10,16);scenario(20,16);scenario(5,4);return 0; }
'''
    with tempfile.TemporaryDirectory(prefix="camera-tx-") as directory:
        d = Path(directory)
        (d / "test.c").write_text(harness, encoding="utf-8")
        result = subprocess.run([zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                                 "-DCARTEST_GYRO350_ENABLED=1", "-I", str(ROOT / "Inc"),
                                 str(d / "test.c"), str(ROOT / "App/reverse_control.c"),
                                 "-o", str(d / "test.exe")], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        result = subprocess.run([str(d / "test.exe")], capture_output=True, text=True)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
    return [dict(zip(("loop_ms", "fifo_slots", "v1_frames", "profile_frames", "hello_frames", "max_bytes_per_poll"),
                     map(int, line.split()))) for line in result.stdout.splitlines()]


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--zig", required=True)
    p.add_argument("--baseline", type=Path)
    args = p.parse_args()
    source = (ROOT / "Src/main.c").read_text(encoding="utf-8")
    results = run_source(source, args.zig)
    assert all(row["profile_frames"] > 0 and row["hello_frames"] == 1 for row in results), results
    report = {"passed": True, "current": results,
              "source_sha256": hashlib.sha256((ROOT / "Src/main.c").read_bytes()).hexdigest(), "hardware_actions": "none"}
    if args.baseline:
        report["baseline"] = run_source(args.baseline.read_text(encoding="utf-8"), args.zig)
    print(json.dumps(report, indent=2))

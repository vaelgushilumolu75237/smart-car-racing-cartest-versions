#!/usr/bin/env python3
"""Read-only ST-Link HotPlug export; never flash, reset, halt, or start motors."""
import argparse
import csv
import datetime
import json
import os
from pathlib import Path
import struct
import subprocess
import time
from runtime_log_export import inspect_elf, _parse_elf

ROOT = Path(__file__).resolve().parents[1]
HEADER = struct.Struct("<10I6i2I4q")
SAMPLE = struct.Struct("<IIIiiI")
PHASES = {0:"ready",1:"ramp",2:"full_pwm",3:"complete",4:"stopped_or_fault"}
REASONS = {0:"completed",1:"PE7_stop",2:"encoder",3:"loop_gap",4:"watchdog_reset",5:"hardware_fault"}
KEYS = ("magic version phase stop_reason now_ms started_ms full_duty_ms finished_ms duty_permille sample_count "
        "peak_left_millirps peak_right_millirps peak500_left_millirps peak500_right_millirps "
        "mean_full_left_millirps mean_full_right_millirps full_sample_ms window_ms "
        "full_left_sum full_right_sum window_left_sum window_right_sum").split()

def decode(data):
    if len(data) < HEADER.size:
        raise ValueError("truncated motor test header")
    d = dict(zip(KEYS, HEADER.unpack_from(data)))
    if d["magic"] != 0x4d415857 or d["version"] != 1 or d["phase"] not in PHASES:
        raise ValueError("invalid motor test header")
    if d["sample_count"] > 800 or d["duty_permille"] > 1000:
        raise ValueError("invalid motor test bounds")
    d["phase_name"] = PHASES[d["phase"]]
    d["stop_reason_name"] = REASONS.get(d["stop_reason"], "unknown")
    for key in list(d):
        if key.endswith("_millirps"):
            d[key.replace("_millirps", "_rps")] = d[key]/1000
            d[key.replace("_millirps", "_rpm")] = d[key]*60/1000
    return d

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--elf", type=Path, default=ROOT/"build/DebugMotorMaxSpeed/CarTest.elf")
    p.add_argument("--out", type=Path)
    p.add_argument("--probe", default="37FF71064E573436FF731443")
    p.add_argument("--watch-seconds",type=float,default=0)
    p.add_argument("--cli",type=Path, default=Path(os.environ["LOCALAPPDATA"])/"stm32cube/bundles/programmer/2.23.0/bin/STM32_Programmer_CLI.exe")
    a=p.parse_args()
    layout=inspect_elf(a.elf)
    _,_,symbols=_parse_elf(a.elf)
    symbol=symbols.get("car_motor_max_test")
    if symbol is None or symbol.size != HEADER.size + 800*SAMPLE.size:
        raise SystemExit("Selected ELF is not the isolated motor speed test.")
    out=a.out or ROOT/"logs/motor-max"/datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True,exist_ok=False)
    def read(address,size,name):
        path=out/name
        try:
            result=subprocess.run([str(a.cli),"-c","port=SWD","mode=HOTPLUG","freq=1000","sn="+a.probe,
                                   "--upload",hex(address),str(size),str(path.resolve())],capture_output=True,timeout=15)
        except subprocess.TimeoutExpired as exc:
            (out/(name+".log")).write_bytes((exc.stdout or b"")+(exc.stderr or b""))
            raise RuntimeError("ST-Link read timed out; check probe/power and close the active Debug session. No reset issued.") from None
        (out/(name+".log")).write_bytes(result.stdout+result.stderr)
        if result.returncode or not path.exists() or path.stat().st_size != size:
            raise RuntimeError("ST-Link read failed; see "+str(out/(name+".log")))
        return path.read_bytes()
    rom=read(layout.descriptor_address,len(layout.descriptor_bytes),"descriptor.bin")
    if rom != layout.descriptor_bytes:
        raise SystemExit("Firmware does not match DebugMotorMaxSpeed ELF; no RAM results decoded.")
    deadline=time.monotonic()+max(0,a.watch_seconds)
    index=0
    while True:
        header=read(symbol.value,HEADER.size,f"header-{index:03}.bin")
        d=decode(header)
        print(json.dumps({k:d[k] for k in ("phase_name","duty_permille","sample_count","peak_left_rps","peak_right_rps")}),flush=True)
        if d["phase"] in (3,4) or time.monotonic()>=deadline:break
        index+=1;time.sleep(0.5)
    # Terminal trace is frozen. While running use only already-published samples.
    count=d["sample_count"]
    data=read(symbol.value,HEADER.size+count*SAMPLE.size,"measurement.bin")
    end=decode(data)
    # The selected prefix is stable; sample_count may grow during an active read.
    rows=[]
    for i in range(count):
        t,phase,duty,left,right,dt=SAMPLE.unpack_from(data,HEADER.size+i*SAMPLE.size)
        if duty>1000 or phase not in (1,2) or not 0<dt<=100:
            raise ValueError("invalid trace record")
        rows.append({"time_ms":t,"elapsed_from_start_ms":(t-end["started_ms"])&0xffffffff,
                     "phase":PHASES[phase],"duty_permille":duty,"sample_ms":dt,
                     "left_rps":left/1000,"right_rps":right/1000,"left_rpm":left*.06,"right_rpm":right*.06})
    end.update(build_id=layout.build_id.hex(),captured_sample_count=count,
               complete=end["phase"]==3, elf=str(a.elf.resolve()))
    (out/"summary.json").write_text(json.dumps(end,indent=2),encoding="utf-8")
    if rows:
        with (out/"wheel_speed.csv").open("w",newline="",encoding="utf-8-sig") as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(json.dumps(end,indent=2));print("Saved:",out)

if __name__=="__main__":main()

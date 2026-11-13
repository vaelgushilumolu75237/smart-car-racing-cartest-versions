# Isolated overhead maximum wheel-speed test

Select the FIRST Debug entry (DebugMotorMaxSpeed), or flash
`build/DebugMotorMaxSpeed/CarTest.elf` manually. Existing C Base/Limit/Limit2
profiles retain their route parameters. No OpenMV/K230 files need changing.

Support the chassis securely with both rear wheels clear. After power-up release
PE7 for at least 20 ms, then press PE7 once to start. Press again to stop early.
The test ramps forward PWM from zero to 1000/1000 over 2000 ms, holds for 5000 ms,
then stops automatically. It executes once per reset; pressing again after
completion cannot repeat it. It does not depend on camera data or wireless.
Do not use the test profile to drive on the track.

The test uses the existing TIM3 left IN2/right IN1 forward pair and encoder
calibration (left 59995 counts/rev; right 60239 counts/rev, signs +1/-1).
No wheel-speed PI limits the test. Encoder invalid/stall, loop gap, watchdog
reset and PE7 stop cancel output. The SysTick path independently removes PWM
at the 7-second deadline or on loss of the main-loop heartbeat.

`car_motor_max_test` stores up to 800 encoder samples (nominally one every
10 ms), instantaneous peak wheel speeds, maximum 500-ms block means, and the
mean at full duty excluding its first 500 ms. Values remain until reset.
Read them while running or after stopping; do not halt or single-step while
motors run. This measures unloaded wheel speed under the tested battery state,
not vehicle ground speed or a motor data-sheet rating.

With the Debug session stopped/disconnected and the probe still attached:

```powershell
python tools/read_motor_max_test.py --watch-seconds 60
```

The script uses read-only SWD HotPlug, checks the ROM build descriptor against
the selected ELF before decoding RAM, and writes `summary.json`,
`wheel_speed.csv`, raw reads and CLI logs into `logs/motor-max/<timestamp>/`.
It does not start motors, halt, flash or reset. Firmware stores the complete
trace locally so a late ST-Link read after stopping still recovers the run.

Host verification:

```powershell
python tests/motor_max/run.py --zig <path-to-zig.exe>
powershell -NoProfile -ExecutionPolicy Bypass -File tools/build.ps1 -Preset DebugMotorMaxSpeed
```

Host tests cover ramp/hold/deadline, stop and fault latching, timer wrap,
trace limits, both speed statistics, the actual PE7 button module and actual
encoder observer stall detection. Software checks are not a physical test.

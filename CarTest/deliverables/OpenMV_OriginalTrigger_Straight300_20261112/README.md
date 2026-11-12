# OriginalTrigger + Straight300

Original confirmed INSIDE ring observation immediately starts H743 ENTRY. H743 commands center 1530us for 300ms from the first accepted entry event, then uses the existing gyro steering. Repeated events do not restart the timer. No entrance hand-push calibration and no additional camera150ms delay.

ENTRY/ORBIT speed remains75% of original profile ring target and steering displacement remains130%. Selected C original:3.960rps. Ordinary driving and existing stop/IMU/transport protections remain. Actual mechanical centering and road behavior require vehicle testing. Post-entry visual evidence still gates orbit yaw zero; net350degrees then original exit/recovery flow.

Update BOTH paired H743 ELF and the OpenMV runtime .py files. For selected C original use H743/C_RingFast/CarTest.elf. Copy all .py files from OpenMV to the existing camera program directory, replacing matching files; retain existing calibration/configuration data files. No automatic flashing/upload/reset/motor operation was performed.

Software evidence: camera590 tests with4 existing skips; gyro12 legacy variants and3 production authority profiles; production0..299/300ms tests cover both turn directions, unchanged ring speed, early orbit rejection and no timer restart; wireless runtime passed; all6 real H743 builds and all runtime MicroPython cross compiles passed. Hardware testing pending.

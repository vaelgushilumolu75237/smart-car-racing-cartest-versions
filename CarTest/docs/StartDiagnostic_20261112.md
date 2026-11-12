# Delay150 with H743 start diagnostics

This release retains the 150ms camera entrance delay. C original uses 3.960 rps during ENTRY/ORBIT, with 130% relative steering gain. No entrance hand-push calibration is needed. Width and IMU calibration are still required.

Manual-start rejection previously returned only ACK 15. The exact original controller predicate now has a side-effect-free blocker query; actual start uses that same predicate. This is a diagnostic correction, not proof that the vehicle starting failure is fixed. All original protections remain.

Send STARTCHECK followed by newline through the existing SeekFree assistant. No ARM or START is required for this read-only query. Page 5 uses zero-based channels: channel0=3, channel1=0 means the query succeeded; channel14 is the blocker mask. A zero mask means this check found no blocker, not permission to drive. Failed START also puts its reason mask in channel14.

Mask values (additive):
1 invalid config; 2 Follow not compiled; 4 IMU not ready; 8 IMU fault/stale;
16 camera protocol not V4; 32 ordinary-route flag missing; 64 latest image invalid;
128 latest and trusted sequence differ; 256 trusted route not fresh;
512 missing run request; 1024 pending manual stop; 2048 loop gap;
4096 encoder fault; 8192 clock/timer fault; 16384 hard/latched fault;
32768 ISR stop; 65536 combined gyro/follow authority rejected;
131072 PE7/startup release or pending key condition; 262144 camera operation busy or Follow active.

Install H743/C_RingFast/CarTest.elf manually for C original. The camera remains the paired Delay150 package (unchanged from the prior release). Do not upload the .mpy validation evidence. Firmware builds and software tests do not establish hardware success. No hardware flash/reset/motor action was performed.

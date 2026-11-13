# r24 ring entry: straight 150 ms / speed 75% / steering 125%

Based on the current r24 continuous-calibration workspace. Production behavior changes are confined to App/gyro_ring.c, Inc/gyro_ring.h and App/gyro350_authority.c.

- The existing ring recognition event starts a centered 1530 us straight segment for 150 ms, measured by the H743 timer. No new vision, width calibration, odometry, manual marking or entry-location confirmation is required to finish this segment. Existing manual stop, communication and IMU protections remain effective.
- A matching valid camera ORBIT event received during this segment is retained without shortening the segment. At expiry, the gyro yaw origin is set to the current yaw if that event was received. Otherwise the original ENTRY logic continues. The existing four-second ENTRY turning timeout starts after the added segment.
- ENTRY (including the added straight segment) and ORBIT outer-wheel targets are 75% of the previous profile values. B: 3.000 rps; C: 3.960 rps; C+20%: 4.752 rps; C+30%: 5.148 rps; C+40%: 5.544 rps.
- ENTRY after the delay and ORBIT servo PWM offset from 1530 us is multiplied by 1.25 and clamped to existing servo limits. This is an output offset adjustment, not a measured mechanical angle.
- Ordinary straight/bend targets, ring recognition, gyro yaw-rate targets, 350 degree net-angle exit, centered exit speed/timing, recovery, automatic width calibration, UART/wireless protocol and protections retain the prior implementation. Profile A still has no ring handling.

## Installation

Open the existing current workspace and select the corresponding Debug configuration containing `entry 150ms / steer +25%`. Build outputs are in build/<preset>/CarTest.elf; frozen paired outputs are under this package's H743/<profile>/CarTest.elf. Use the same speed profile as before.

The OpenMV sources are byte-for-byte identical to the preserved pre-change workspace. No camera update is required for this change. The included OpenMV directory is the paired restoration copy. Actual vehicle calibration files are private and excluded from this public package.

## Verification

Passed: 12 pure-C profile/IMU-sign variants; 3 controller authority variants; entry 149/150 ms boundaries, both sides, timer wrap, early ORBIT event retention, no-new-image expiry; servo 125% mapping and unchanged exit outputs; wireless runtime; ordinary Ackermann 10067 checks and four profile target variants; 563 camera tests (4 existing skips); three delivery validation tests; six actual ARM ELF builds; OpenMV cross-compilation and package hashes.

The legacy MSVC Ackermann runner could not resolve its generated command path on this machine; the same production test sources passed with Zig's C compiler. No device upload, H743 flash/reset or motor start was performed. Hardware/track validation is pending.

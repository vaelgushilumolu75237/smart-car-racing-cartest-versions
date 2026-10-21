# CarTest OpenMV camera

The production source defaults to profile `pd`, build ID `BOTTOM80_POSPD_20261020`. Three complete upload packages are in `../deliverables/OpenMV_Bottom80_Steering_20261021/`. The first camera trial should use `no_pd` to establish the same perception and weighted-position baseline before enabling derivative control. Upload one package at a time: copy its 20 top-level `.py` files together and restart the OpenMV interpreter. Do not mix profiles or upload host `.mpy` files or tests.

| Package | Control behavior | Trial order |
| --- | --- | --- |
| `no_pd` | Existing position EMA output only | First |
| `pd` | Position-only P+D after the EMA | Second |
| `pd_preview` | PD plus bounded confirmed geometry preview | Optional last |

All profiles preserve the same image and route processing. Full-image y=24 divides the 80% near and 20% far groups. The far group must contain at least three supported rows spanning at least two grid rows; otherwise the near rows normalize to the full position. No near rows means an invalid position. The existing EMA is retained. PD uses only that filtered position, a 60 ms low-pass derivative, 30 ms derivative time, D limit 100, maximum capture interval 100 ms, and combined output limit 1000. Invalid frames, long capture gaps, source/identity changes and changes to the actual selected weighted reference row reset the controller. Repeated captures do not advance its history.

Preview remains off in `no_pd` and `pd`. In `pd_preview`, a geometry-qualified candidate of at least 80 magnitude must keep the same sign for three distinct captures over at least 60 ms, with same-sign filtered position of at least 50. Gain rises linearly to 0.35 at position magnitude 250, with applied contribution capped at 100 and slew limited to 500 per second. Existing geometry conflict, invalidity, identity and reference resets remain in force.

The 12-byte observation UART V1 frame is unchanged. The camera diagnostic dump has a separate 32-byte control-extension v2 record with its own sequence, capture/send camera-clock timestamps, actual sent command and CRC-16/CCITT-FALSE. Camera times are relative to the OpenMV clock and must not be subtracted from STM32 arrival times as if they shared an epoch.

`PACKAGE_MANIFEST.json` identifies the default `pd` package. Each upload folder has its own manifest, build ID and file hashes. All 60 profile modules were cross-compiled with bundled mpy-cross 1.25.0 for the ARMv7EM-SP target. The reported board runtime is MicroPython 1.28.0-49; no matching mpy-cross compiler was available in the IDE archive, so 1.28 compiler acceptance remains unverified. The production OpenMV host suite passes 183 tests. The frozen V7 suite ran 79 tests: 78 passed and one board-only case was skipped. These are host and compiler checks only. `test-data/speedpd-20261020-validation/camera/host-performance-comparison.json` reports the full-clear versus touched-row aggregation comparison and controller-only profile timing on synthetic CPython inputs. It does not benchmark the sensor/Viper frame loop or predict camera frame rate. No image-processing resolution reduction or safety-gate removal was made for speed. Production OpenMV 45 fps and vehicle behavior remain unverified; no camera upload or motor run was performed.

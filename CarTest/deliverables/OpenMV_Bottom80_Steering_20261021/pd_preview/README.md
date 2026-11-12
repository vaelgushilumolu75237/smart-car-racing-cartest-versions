# OpenMV Bottom80 steering package

Profile: `pd_preview`  
Build ID: `BOTTOM80_PD_PREVIEW_20261020`

This is an optional comparison profile; begin with no_pd. Confirmed adaptive preview is enabled in this profile.

Upload all 20 top-level `.py` files from this folder together, then restart the OpenMV interpreter. Do not mix profiles, upload tests, or place host-generated `.mpy` validation files on the camera. The 12-byte observation UART protocol is unchanged.

All profiles use full-frame y=24 weighting (near 80% / far 20%). Far support requires at least three rows spanning two grid rows; otherwise the near group is normalized alone. No near rows means invalid position. The existing position EMA remains the PD input.

PD uses a 60 ms derivative low-pass time constant, 30 ms derivative time, +/-100 D component, 100 ms maximum capture gap and +/-1000 combined output. Preview requires three distinct same-sign frames over at least 60 ms, candidate magnitude >=80, same-sign position magnitude >=50, gain up to 0.35 at |position|=250, +/-100 applied limit and 500/s slew. Confirmed adaptive preview is enabled in this profile.

`PACKAGE_MANIFEST.json` records all source and compiled hashes. The bundled compiler is mpy-cross 1.25.0 for ARMv7EM-SP; the reported board runtime is MicroPython 1.28.0-49 and no matching compiler was available in the IDE archive. Cross-compilation does not prove 1.28 acceptance, OpenMV board runtime, vehicle behavior, or 45 fps. No hardware upload was performed.

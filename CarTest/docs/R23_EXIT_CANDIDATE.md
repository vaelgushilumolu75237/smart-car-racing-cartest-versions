# v26-r23 exit candidate (2026-11-01)

Camera identity: TRACK_RING_EXIT_V26_R23_20261101.
Paired H743: unchanged r22 C_RingFast ELF, SHA256
2c4c232eed44f24055cf986a7657a919e682b0b36978ca58d497503d024614ce.

Changes
- Discover a qualified right-ring exit while still INSIDE. Entry transition must
  finish and an inner route must actually have been transmitted. Three adjacent
  eligible frames with an absent/curved outer exit corridor AND a valid current
  inner corridor establish departure from the entry straight. This is not a lap
  counter or proof of a full circle. The subsequent qualified left exit edge
  can take over in its first frame, without the old frozen-turn anchor.
- Latch exit ownership only after a valid complete UART write. Rebuild the target
  from the current left edge and calibrated per-row width. Do not return to the
  island edge during gaps. Invalid frames leave H743 to hold the last transmitted
  exit error under the existing 3000ms deadline and 300ms deceleration; invalid
  frames never refresh that deadline or become fabricated valid routes.
- Preserve stable bilateral recovery and ordinary-route release. Left-ring rules,
  entry detector, steering gain, speed values, PE7 and H743 source/ELFs remain.
- Sampled IDE diagnostics add search, lock, and the actual geometry rejection.
  main/config/track_elements identity checks require the matching full package.

Validation
- 519 camera tests: 515 passed, 4 skipped; 23/23 armv7emsp cross-compiles.
- 72 MCU host cases / 7713 assertions passed. 2000/200 and C 3000/300 hold
  boundary fixtures passed. 13 camera V3 packets passed the real H743 parser
  and extracted production servo mapping, including first INSIDE-exit targets.
- 238 MCU source/config files and all 15 existing actual r22 ELF builds verified
  byte-identical. MCU binaries were reused, not rebuilt or flashed.
- Saved original-image geometry reviewed across five historical hand-push runs.
  Records are sampled; eligible previous state was explicitly assumed for these
  isolated checks. They are NOT a complete temporal replay. One retained exit
  image produced 52 accepted route rows; other ring/gap images remained rejected
  for curvature, missing observed rows, or ordinary-road ownership. The new
  ownership sequence itself is covered by frame/UART regression tests.
- The supplied 5ide.txt has 27 INSIDE records and no TRACK_BUILD header. Its sparse
  output cannot prove every intermediate state or exact video synchronization.
  5.mp4 is compressed visual evidence, not a raw algorithm replay.

Limits and manual acceptance
No camera program was uploaded, no board reset/flashed, and no motor started.
This is a candidate, NOT a road-accepted release. Current left-edge geometry can
still be unusable/outside view at the exit; this change cannot make an unseen
boundary valid. Full right-ring ground exit must still be verified. Actual board
FPS and IDE-preview >=60FPS remain UNVERIFIED; host runtime is not board FPS.

Local delivery
Install/OpenMV contains all 23 Python files, main.py, manifests and the original
camera calibration JSON. CarTest is the complete source, including Src/main.c.
Use the existing r22 C ELF if that exact version is already on the vehicle.
Do not mix Python versions. Camera installation requires explicit authorization.
Keep the old r22 folder as rollback. Private calibration/images/logs are excluded
from the public checkpoint.

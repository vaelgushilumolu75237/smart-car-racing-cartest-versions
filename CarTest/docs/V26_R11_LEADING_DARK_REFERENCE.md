# v26-r11 leading dark reference candidate

The leading uniformly dark threshold block previously required every other block to contain at least 5 percent of both classes. A shrinking white sliver in one otherwise trusted high-contrast block could veto dark classification even when two independent populated reference blocks remained available. That left the leading block unknown and blocked confirmed-ring topology and outer-route continuation.

Require every remaining block to remain trusted and strongly separated, require at least two current-frame populated reference blocks, and retain dark-mean consistency across all peer blocks. Keep the existing dark range/maximum bounds. The resolver still changes only the leading block to known BLACK, never unknown pixels to white road. Another unknown block, weak contrast, inconsistent dark references, insufficient reference population, or a non-dark leading block still fails closed. Ordinary processing does not invoke this confirmed-ring-only resolver. No topology gate is bypassed.

PE7, speed targets/gains, steering mapping, protocol/profile pairing, calibration, native APP, and the H743 2 second loss policy remain unchanged. Three paired H743 ELFs are byte-identical to r10. The r10 first-failure raw capture remains enabled.

Validation: 439 host tests, zero failures/errors, 3 existing skips, one explicitly excluded replay needing missing private RAW input. All 23 production runtime files cross compile for MicroPython 1.28 armv7emsp. New tests cover the 5 percent crossing with two populated peers, insufficient population, weak contrast, inconsistent sparse peer dark means, and ordinary mask isolation.

Three private first-failure original frames also pass the real H750 native mask/topology path in RAM: 52 fully known rows and complete topology on all three. This replay writes no camera files and sends no physical UART. Private host route reconstruction uses exact calibration and anchors reconstructed from the preceding raw image; it restores outer targets but is not a complete temporal replay or road acceptance. Actual vehicle logs, images and calibration remain outside public checkpoints.

This is a candidate for another ground right-ring test. Subsequent route failures, left-ring road behavior and 60+ FPS including actual IDE preview remain unverified. No automatic Python installation, firmware flash, H743 reset or motor start is part of publication.

# PE7 low-RUN/high-STOP with debounce v20 (2026-10-29)

Requested behavior: PE7 is level-controlled, not toggled by repeated presses. After startup/rearm on high stable for 20 ms, low must remain stable for 20 ms before requesting RUN. Low then retains that request. The first high sample clears the request and pending start immediately and invokes the existing motor stop path, without a stop debounce delay. The next start requires stop consumption, high20-ms rearm, then low20-ms qualification. Bounce before a valid start resets low timing; stop bounce cannot directly restart. Held-low startup or fault recovery cannot auto-start.

Only the H743 switch helper/header and one main sampler assignment change. Vision, speed/steering parameters, A/B/C profile pairing, reverse RESET/KEEP, vision-validity and communication protections are retained. Bench and DebugSteering behavior is unchanged. A/B/C target wheel rates remain A=5.6/4.4 rps ring-off; B=4.8/4.0 ring-on; C=5.6/4.4 ring-on. These are target wheel rates, not measured body speed.

Software verification: 73 host cases / 7787 assertions / 0 failures, main STOP/start-handoff checks, four delivery/protocol tests, all15 H743 preset builds with stable production hashes, and all23 unchanged camera modules cross-compiled for armv7emsp. Source review by the lead model; bounded execution/testing by GPT-6 Luna Max.

Status is experimental software-tested, hardware pending. No H743 flash/reset, camera program upload, or motor start was performed for this candidate. This source change does not change firmware already on the vehicle. First verify switch state transitions with motor drive power disconnected, then secured raised-wheel stopping before road tests.

OpenMV sources remain byte-identical to v17; a matching camera installation needs no re-upload. Keep its private camera-specific width calibration. This public package excludes actual vehicle logs, photos, and private calibration. Validation .mpy files are not camera deployment files.

C still requires V3/ACK3/ready1; B/C still require valid width calibration and parked startup checks. Do not bypass pairing or other gates. Ring routes remain experimental. Neither host tests nor UART accepted-frame rates establish board full-pipeline60FPS, USB320x120 new-frame60FPS, mechanical behavior, or road acceptance. See docs/PE7-LEVEL-RUN-STOP-20261029.md in the complete checkpoint.

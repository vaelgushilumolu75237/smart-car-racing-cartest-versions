# v24 PE7 exact reference restoration

The user explicitly requested replacing current switch behavior with the switch module and logic from `D:/T0/10月25_3.51_perfect/CarTest`. This supersedes the later maintained-level FSM and one-second PE7 mask requests. No automatic flashing, reset, camera upload or motor start is authorized by this source replacement.

## Restored behavior and scope

`App/manual_run_button.c` and `Inc/manual_run_button.h` are byte-identical copies of that reference. Its Follow switch is toggle-based: power-on stopped; high stable20ms arms; the next low sample toggles the run request. Holding low does not repeat; release alone does not stop. Rearm high20ms and then another low toggles STOP. A pending stop wins over a later event; hard faults reject starting and existing cancellation clears the request. There is no50ms low qualification, three-state FSM, or1000ms high-ignore window.

In `Src/main.c`, `switch_init`, `manual_button_sample`, `SysTick_Handler` and `EXTI9_5_IRQHandler` match the reference function text exactly. Follow uses rising/falling EXTI and the original three main-loop fallback sample locations. The deferred-start handoff and motor guard use the original request/pending-stop predicates. Removed the obsolete FSM/sample-gap debug variables and helpers; retained actual PWM debug fields, camera diagnostics and strict selected-ELF reader checks. Newer camera communication, session odometry, original gains/speed targets and all unrelated code remain. This restores source behavior; it does not claim the current ELF equals the historical frozen C ELF.

Restoration hashes (SHA256):
- `App/manual_run_button.c`: `9f180df53b5c43172b5c3d6c3de3e4e78fa96e227c679b49b5891554893d7a2d`
- `Inc/manual_run_button.h`: `f66cca80884a5087a5d8a1afae4bb9671acca300413c6b641657d14aa2bbf09f`
- `switch_init`: `72485a9c4cc7566df5027d4b5a31e8f108661e650ea7b8dfec1c8e0a1cc3bf60`
- `manual_button_sample`: `d850c9c46f23bd0bc62f6310b1c09f49c80b4217779e6067f1db6f28ad228782`
- `SysTick_Handler`: `96801ab13ff44331f732cf033bc3dcd373ac7b1bd2d14840869b4565a7824479`
- `EXTI9_5_IRQHandler`: `4127f24bed28402319af9f1b5e8228a2c11dd69853f84d31ca57f286326ad391`

## Validation

- 72 C/C++ host cases,7713 assertions,0 failures, including the restored reference module lifecycle and controller RESET/KEEP/fresh-frame resync tests. Newer unrelated diagnostic/controller tests retained.
- Real production switch sampler, EXTI, SysTick and emergency-stop functions under fake registers:26 checks,0 failures. Covers high20ms rearm, low start/stop toggles, release preserving RUN, held-low non-repeat, missed-edge main fallback, loop-gap PWM stop, hard fault rejection and IRQ preservation.
- Actual motor guard/apply harness9 variants passed, including release preserving RUN, pending-stop revocation, hardware lock, PI/incremental PI/Ackermann and fault gates. Static integration audit passed.
- 36 reader/status/wire/delivery tests passed. All15 H743 presets freshly built. All23 unchanged camera runtime files cross-compiled; no camera suite rerun is claimed for this button-only restoration.

Before editing: source/config/paired ELF/hash preservation under local `logs/diagnostics/pre-pe7-reference-restore-v24-20261029`. Validation evidence under `pe7-reference-v24-tests` and `pe7-reference-v24-builds-20261029`. Public checkpoints exclude these private logs and calibration. Camera already on the completev21/v22/v23 runtime needs no repeat upload. Older camera runtime still requires the paired diagnostic-capable package; ACK, valid observation and hard-fault protections remain.

No hardware action performed forv24. Physical switch/PWM timing, camera/USB60FPS and road acceptance remain pending. The previously observed crossroad route loss and two-second hold behavior were not changed by this switch-only restoration.

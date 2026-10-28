"""Check the production main-loop handoff for a physical Follow start."""

from __future__ import annotations

import argparse
from pathlib import Path


def block_after(source: str, marker: str) -> str:
    start = source.index(marker)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise ValueError(f"unterminated block after {marker!r}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="Src/main.c")
    args = parser.parse_args()
    source = Path(args.source).read_text(encoding="utf-8")

    event_block = block_after(
        source,
        "if (app_config.follow_mode_compiled != 0u &&\n"
        "            manual_run_button.start_pending != 0u)",
    )
    assert "follow_start_event = 1u;" in event_block
    assert "car_manual_button_start_consumed(&manual_run_button);" in event_block

    resync_block = block_after(
        source,
        "if (follow_start_event != 0u &&\n"
        "            app_config.follow_mode_compiled != 0u)",
    )
    disable = resync_block.index("__disable_irq();")
    rebuild = resync_block.index("inputs = make_inputs(")
    run_check = resync_block.index("manual_run_button.run_request != 0u")
    stop_check = resync_block.index("manual_run_button.stop_pending == 0u")
    pending_check = resync_block.index("manual_run_button.start_pending == 0u")
    call = resync_block.index("car_control_begin_manual_follow_start(")
    restore = resync_block.index("__set_PRIMASK(start_event_primask);")
    emergency_stop = resync_block.index("emergency_stop_hardware();")
    assert disable < rebuild < run_check < stop_check < pending_check < call < restore
    assert restore < emergency_stop
    start_guard = block_after(
        resync_block,
        "if (manual_run_button.run_request != 0u &&",
    )
    assert "car_control_begin_manual_follow_start(" in start_guard

    accepted_block = block_after(
        resync_block, "if (manual_start_accepted != 0u)")
    assert "++follow_session_token;" in accepted_block
    assert "if (follow_session_token == 0u)" in accepted_block
    assert "reset_session_odometry();" in accepted_block

    call_at = source.index("car_control_begin_manual_follow_start(")
    assert source.rfind("car_control_tick(", 0, call_at) < call_at
    assert source.rfind("cancel_manual_request_on_fault();", 0, call_at) < call_at
    flush_at = source.index("flush_uart_and_parser();", call_at)
    service_at = source.index("service_reverse_control(HAL_GetTick());", call_at)
    assert call_at < flush_at < service_at

    print("PASS production PE7 start rechecks stop state and advances session identity")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

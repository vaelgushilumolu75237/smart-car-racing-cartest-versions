"""Minimal, never-executed RAM fixture used by host mocks."""

LIVE_SENSOR_UART_SINK = "fake sink required by benchmark preflight"


class _Source:
    pass


class _Sink:
    def write(self, packet):
        return len(packet)


import main

main._main_fast(frame_source=_Source(), uart_sink=_Sink())

"""Host-side wireless framing and explicit command helpers.

The module never opens a serial port by itself.  The optional GUI is an
explicit-connect tool and uses this package for all bytes on the wire.
"""

from .protocol import (
    Frame,
    FrameParser,
    FrameType,
    ProtocolError,
    SeekFree,
    encode_frame,
    pack_command,
)

__all__ = [
    "Frame", "FrameParser", "FrameType", "ProtocolError", "SeekFree",
    "encode_frame", "pack_command",
]

"""Yahboom K230 vision module deluxe, not the generic K230 development board.

Vehicle communication: UART1, TX GPIO9, RX GPIO10 on the factory socket.
Source: https://www.yahboom.net/study/K230/14977
All camera mounting changes require a new private calibration.
"""
PORT_BUILD = "CARTEST_K230_GYRO350_20261106"
UART_ID = 1
UART_TX_PIN = 9
UART_RX_PIN = 10
SENSOR_CSI_ID = None  # Use Yahboom firmware's default CSI (normally CSI2).
SENSOR_INPUT_WIDTH = 1280
SENSOR_INPUT_HEIGHT = 720
SENSOR_FPS = 90  # Requested rate; not a measured processing/display rate.
MIRROR_X = False
FLIP_Y = False
DISPLAY_MODE = "lcd"  # lcd (ST7701 + IDE), ide (VIRT), or off.
DISPLAY_WIDTH = 640
DISPLAY_HEIGHT = 480
DISPLAY_FPS = 60
DISPLAY_UPDATE_PERIOD_MS = 100  # LCD 10 Hz; perception/control are never throttled.
DISPLAY_QUALITY = 50
DATA_DIRECTORY = "/sdcard/cartest-k230"
MOUNT_REVISION = "mount-1"  # Change after camera/lens/height/pitch adjustment.

# The side-band service is present in the wireless package but its H743
# compiler preset is the authority for enabling the physical feature.  The
# active K230 package is the paired wireless build; older source packages keep
# their own k230_settings.py and therefore remain disabled.
WIRELESS_LINK_ENABLED = True
WIRELESS_REQUIRED = True
WIRELESS_STATE_PERIOD_MS = 50       # 20 Hz
WIRELESS_DIAG_PERIOD_MS = 500       # 2 Hz
WIRELESS_BINARY_PERIOD_MS = 500     # 80x60, 2 fps
WIRELESS_BAUDRATE = 115200
WIRELESS_SIDECAR_PATH = DATA_DIRECTORY + "/k230_wireless_settings.bin"


def _crc32(data):
    """Small IEEE CRC32 implementation for the private K230 sidecar."""
    crc = 0xFFFFFFFF
    for value in data:
        crc ^= int(value)
        for _bit in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF


def _put_u16(out, offset, value):
    out[offset] = int(value) & 0xFF
    out[offset + 1] = (int(value) >> 8) & 0xFF


def _put_u32(out, offset, value):
    value = int(value) & 0xFFFFFFFF
    out[offset] = value & 0xFF
    out[offset + 1] = (value >> 8) & 0xFF
    out[offset + 2] = (value >> 16) & 0xFF
    out[offset + 3] = (value >> 24) & 0xFF


def _get_u16(data, offset):
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def _get_u32(data, offset):
    return (int(data[offset]) | (int(data[offset + 1]) << 8) |
            (int(data[offset + 2]) << 16) | (int(data[offset + 3]) << 24))


class WirelessSettingsStore:
    """CRC-checked, versioned private sidecar for camera-only parameters.

    This file is deliberately separate from ``ring_width_calibration.json``:
    a wireless tuning write can never make an otherwise ready geometry map
    appear ready, and a damaged sidecar falls back to the shipped defaults.
    """

    MAGIC = b"KWS1"
    VERSION = 1
    RECORD_SIZE = 24

    def __init__(self, path=None):
        self.path = path or WIRELESS_SIDECAR_PATH

    def _record(self, values):
        out = bytearray(self.RECORD_SIZE)
        out[0:4] = self.MAGIC
        _put_u16(out, 4, self.VERSION)
        _put_u16(out, 6, 8)  # generation + P + D payload
        _put_u32(out, 8, values.get("generation", 0))
        _put_u16(out, 12, values.get("vision_p", 1000))
        _put_u16(out, 14, values.get("vision_d", 1000))
        # 16..19 reserved and kept zero for deterministic records.
        _put_u32(out, 20, _crc32(out[:20]))
        return out

    def load(self):
        try:
            with open(self.path, "rb") as stream:
                raw = stream.read(self.RECORD_SIZE)
            if (len(raw) != self.RECORD_SIZE or raw[0:4] != self.MAGIC or
                    _get_u16(raw, 4) != self.VERSION or
                    _get_u16(raw, 6) != 8 or
                    _get_u32(raw, 20) != _crc32(raw[:20])):
                raise ValueError("invalid K230 wireless sidecar")
            p = _get_u16(raw, 12)
            d = _get_u16(raw, 14)
            if not 500 <= p <= 1500 or not 500 <= d <= 1500:
                raise ValueError("K230 wireless gain outside bounds")
            return {"version": self.VERSION, "generation": _get_u32(raw, 8),
                    "vision_p": p, "vision_d": d,
                    "crc": _get_u32(raw, 20)}
        except Exception:
            # Missing/corrupt records are not calibration success.  Return
            # defaults, while preserving a precise status for diagnostics.
            return {"version": self.VERSION, "generation": 0,
                    "vision_p": 1000, "vision_d": 1000, "crc": 0,
                    "valid": False}

    def save(self, values):
        generation = (int(values.get("generation", 0)) & 0xFFFFFFFF)
        record = self._record({
            "generation": generation,
            "vision_p": int(values.get("vision_p", 1000)),
            "vision_d": int(values.get("vision_d", 1000)),
        })
        parent = self.path.rsplit("/", 1)[0] if "/" in self.path else ""
        if parent:
            try:
                os.mkdir(parent)
            except OSError:
                pass
        temporary = self.path + ".tmp"
        with open(temporary, "wb") as stream:
            stream.write(record)
        try:
            os.remove(self.path)
        except OSError:
            pass
        os.rename(temporary, self.path)
        return {"version": self.VERSION, "generation": generation,
                "vision_p": int(values.get("vision_p", 1000)),
                "vision_d": int(values.get("vision_d", 1000)),
                "crc": _get_u32(record, 20), "valid": True}

    def defaults(self):
        return self.save({"generation": 0, "vision_p": 1000,
                          "vision_d": 1000})


# Full masks/topology remain 80x60; selected path rows keep physical coordinates.
PATH_ROW_STEP = 3
PATH_ROW_PHASE = 1

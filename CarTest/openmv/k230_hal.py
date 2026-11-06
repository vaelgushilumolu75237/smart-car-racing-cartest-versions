"""Lazy CanMV K230 adapters; importing this module never touches hardware.

Keep the production detector's 320x120 center crop, full 4x2 min pooling,
protocol, deadlines and ring decisions. No OpenMV firmware/native binaries
are loaded on this RISC-V platform.
"""
import os
import time
import k230_settings as settings

_uarts = []


def _mkdirs(path):
    current = "/" if path.startswith("/") else ""
    for part in path.split("/"):
        if not part:
            continue
        current = current.rstrip("/") + "/" + part
        try:
            os.mkdir(current)
        except OSError:
            # Do not conceal a read-only filesystem or a file in place of a dir.
            if not (os.stat(current)[0] & 0x4000):
                raise


def configure_paths(config):
    _mkdirs(settings.DATA_DIRECTORY)
    config.RING_WIDTH_CALIBRATION_PATH = settings.DATA_DIRECTORY + "/ring_width_calibration.json"
    config.RING_DIAGNOSTICS_PATH = settings.DATA_DIRECTORY + "/ring_diagnostics.bin"


class _PreviewConnection:
    def isconnected(self):
        # This denotes an enabled CanMV display sink, not a USB cable assertion.
        return settings.DISPLAY_MODE != "off"


class GrayFrame:
    """A cropped, independently owned grayscale image and its byte buffer."""
    def __init__(self, image):
        self.image = image
        if image.width() != 320 or image.height() != 120:
            raise ValueError("K230 crop must be 320x120")
        raw = image.bytearray()
        if len(raw) != 38400:
            raise ValueError("K230 grayscale crop is padded or has wrong size")
        self.raw = raw if isinstance(raw, bytearray) else bytearray(raw)

    def width(self):
        return 320

    def height(self):
        return 120

    def format(self):
        return self.image.format()

    def bytearray(self):
        return self.raw

    def get_pixel(self, xy):
        return self.image.get_pixel(xy)

    def __getattr__(self, name):
        return getattr(self.image, name)


class Camera:
    EXPOSURE_CONTROL_SUPPORTED = False

    def __init__(self):
        self.device = None
        self.display = None
        self.media = None
        self.media_started = False
        self.display_started = False
        self.latest = None
        self.sensor_type = None
        self.binding = None
        self.usb_vcp = _PreviewConnection()

    def configure(self):
        if self.device is not None:
            return self.sensor_type
        from media.sensor import Sensor
        from media.media import MediaManager
        from media.display import Display
        import image
        kwargs = dict(width=settings.SENSOR_INPUT_WIDTH,
                      height=settings.SENSOR_INPUT_HEIGHT, fps=settings.SENSOR_FPS)
        if settings.SENSOR_CSI_ID is not None:
            kwargs["id"] = settings.SENSOR_CSI_ID
        try:
            self.device = Sensor(**kwargs)
            self.device.reset()
            self.sensor_type = self.device.get_type()
            if not isinstance(self.sensor_type, int) or self.sensor_type < 0:
                raise RuntimeError("K230 sensor type unavailable")
            self.device.set_framesize(width=320, height=240)
            self.device.set_pixformat(Sensor.GRAYSCALE)
            self.device.set_hmirror(settings.MIRROR_X)
            self.device.set_vflip(settings.FLIP_Y)
            self.GRAYSCALE = image.GRAYSCALE
            self.display = Display
            self.media = MediaManager
            if settings.DISPLAY_MODE not in ("off", "ide", "lcd"):
                raise ValueError("DISPLAY_MODE must be off, ide or lcd")
            if settings.DISPLAY_MODE != "off":
                mode = Display.ST7701 if settings.DISPLAY_MODE == "lcd" else Display.VIRT
                width = settings.DISPLAY_WIDTH if settings.DISPLAY_MODE == "lcd" else 320
                height = settings.DISPLAY_HEIGHT if settings.DISPLAY_MODE == "lcd" else 240
                # Flag before calls so partial initialization is also cleaned.
                self.display_started = True
                Display.init(mode, width=width, height=height, to_ide=True,
                             fps=settings.DISPLAY_FPS, quality=settings.DISPLAY_QUALITY)
            self.media_started = True
            MediaManager.init()
            self.device.run()
            exitpoint = getattr(os, "exitpoint", None)
            if callable(exitpoint):
                exitpoint(os.EXITPOINT_ENABLE)
            self.binding = self._binding()
            import k230_calibration
            k230_calibration.bind(self.sensor_type, self.binding)
            print("K230_CAMERA type=%d crop=320x240(0,60,320,120) "
                  "requested_fps=%d display=%s isp_exposure=firmware_default "
                  "processing_fps=unverified" % (
                      self.sensor_type, settings.SENSOR_FPS, settings.DISPLAY_MODE))
            return self.sensor_type
        except BaseException:
            self.close()
            raise

    def _binding(self):
        import machine
        uid = "unavailable"
        for name in ("unique_id", "chipid"):
            reader = getattr(machine, name, None)
            if callable(reader):
                uid = repr(reader())
                break
        # Includes capture geometry, orientation and explicit mounting revision.
        return "yahboom-k230|%s|%s|%s|%dx%d@%d|320x240:0,60,320,120|%d,%d|%s" % (
            uid, self.sensor_type, settings.SENSOR_CSI_ID,
            settings.SENSOR_INPUT_WIDTH, settings.SENSOR_INPUT_HEIGHT,
            settings.SENSOR_FPS, settings.MIRROR_X, settings.FLIP_Y,
            settings.MOUNT_REVISION)

    def snapshot(self):
        self.latest = None
        exitpoint = getattr(os, "exitpoint", None)
        if callable(exitpoint):
            exitpoint()
        full = self.device.snapshot()
        if full is None or full.width() != 320 or full.height() != 240:
            raise RuntimeError("K230 capture geometry mismatch; no frame reused")
        if full.format() != self.GRAYSCALE:
            raise RuntimeError("K230 camera did not return grayscale")
        self.latest = GrayFrame(full.copy(roi=(0, 60, 320, 120)))
        return self.latest

    def flush(self):
        if self.latest is not None and settings.DISPLAY_MODE != "off":
            x = (settings.DISPLAY_WIDTH - 320) // 2 if settings.DISPLAY_MODE == "lcd" else 0
            y = (settings.DISPLAY_HEIGHT - 120) // 2 if settings.DISPLAY_MODE == "lcd" else 60
            self.display.show_image(self.latest.image, x=x, y=y)

    def close(self):
        self.latest = None
        errors = []
        for enabled, operation in (
                (self.device is not None, lambda: self.device.stop()),
                (self.display_started, lambda: self.display.deinit()),
                (self.media_started, lambda: self.media.deinit())):
            if enabled:
                try:
                    operation()
                except Exception as error:
                    errors.append(str(error))
        self.device = None
        self.display_started = self.media_started = False
        if errors:
            print("K230_CLEANUP_ERRORS", errors)


class UARTAdapter:
    def __init__(self, uart_id, baudrate, bits=8, parity=None, stop=1,
                 timeout=20, timeout_char=20):
        from machine import UART, FPIOA
        if bits != 8 or parity is not None or stop != 1 or baudrate != 115200:
            raise ValueError("CarTest requires 115200/8N1")
        # The OpenMV config keeps its historical UART3; the factory socket
        # on this particular Yahboom board is UART1 at GPIO9/10.
        fpioa = FPIOA()
        fpioa.set_function(settings.UART_TX_PIN,
                          getattr(FPIOA, "UART%d_TXD" % settings.UART_ID), ie=0, oe=1)
        fpioa.set_function(settings.UART_RX_PIN,
                          getattr(FPIOA, "UART%d_RXD" % settings.UART_ID), ie=1, oe=0)
        # K230 STOPBITS_ONE == 0; passing OpenMV stop=1 selects two stops.
        # Factory CanMV rejects OpenMV timeout/timeout_char keywords.
        self.device = UART(settings.UART_ID, baudrate=baudrate,
                           bits=UART.EIGHTBITS, parity=UART.PARITY_NONE,
                           stop=UART.STOPBITS_ONE)
        self.pending = bytearray()
        _uarts.append(self)

    def any(self):
        any_fn = getattr(self.device, "any", None)
        if callable(any_fn):
            return len(self.pending) + any_fn()
        # Factory UART exposes nonblocking read() but some versions lack any().
        # Read a bounded burst and retain its exact binary bytes for readinto.
        if not self.pending:
            data = self.device.read(256)
            if data:
                if not isinstance(data, (bytes, bytearray)) or len(data) > 256:
                    raise ValueError("K230 UART returned invalid RX data")
                self.pending.extend(data)
        return len(self.pending)

    def readinto(self, buffer):
        if self.pending:
            count = min(len(buffer), len(self.pending))
            buffer[:count] = self.pending[:count]
            del self.pending[:count]
            return count
        return self.device.readinto(buffer)

    def write(self, data):
        # Preserve None/short writes; never claim successful full transmission.
        return self.device.write(data)

    def deinit(self):
        self.device.deinit()


class _PybFacade:
    UART = UARTAdapter

    @staticmethod
    def USB_VCP():
        return sensor.usb_vcp


sensor = Camera()
pyb = _PybFacade()


def run(entrypoint):
    import config
    configure_paths(config)
    try:
        entrypoint()
    finally:
        for uart in _uarts:
            try:
                uart.deinit()
            except Exception as error:
                print("K230_UART_CLEANUP_ERROR", str(error))
        _uarts[:] = []
        sensor.close()


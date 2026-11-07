import struct
import unittest

from tools.wireless.complete_tool import (
    OP_CAMERA_CAL, OP_CAPTURE, OP_DEFAULT, OP_GET_PARAMS, OP_IMU_CAL,
    OP_INFO, OP_MODE, OP_PAGE, OP_SAVE, OP_SET_PARAMS, OP_START, OP_STOP,
    OP_TEST_MOTOR, OP_TEST_SERVO, WirelessClient, decode_camera_diag_payload,
    ImageAssembler, TelemetryEvent, decode_camera_state_payload, decode_config_payload,
    WirelessApp, decode_image_payload, decode_state_payload,
)
from tools.wireless.protocol import Frame, FrameType, ProtocolError, decode_frame, encode_frame


class CompleteToolTests(unittest.TestCase):
    @staticmethod
    def _poll_stub_app(events):
        import queue
        from collections import deque

        app = WirelessApp.__new__(WirelessApp)
        app.queue = queue.Queue()
        for event in events:
            app.queue.put(event)
        app.mcu_state = {}
        app.camera_state = {}
        app.config_state = {}
        app.h743_config = {}
        app.camera_config = {}
        app.last_ack = {}
        app.plot_history = deque(maxlen=180)
        app.status = None
        app._render_states = lambda: None
        app._render_ack = lambda: None
        app._draw_curve = lambda: None
        app.after = lambda *_args: None
        return app

    def test_command_ids_match_firmware_contract(self):
        self.assertEqual(
            [OP_INFO, OP_START, OP_STOP, OP_SET_PARAMS, OP_SAVE, OP_DEFAULT,
             OP_TEST_MOTOR, OP_TEST_SERVO, OP_IMU_CAL, OP_CAMERA_CAL, OP_CAPTURE,
             OP_MODE, OP_PAGE, OP_GET_PARAMS],
            list(range(1, 15)),
        )

    def test_state_signed_fields(self):
        values = [0] * 32
        values[8] = 0xFFFFFFFF
        values[19] = 0xFFFFFFFE
        values[20] = 0x80000000
        values[22] = 7
        decoded = decode_state_payload(struct.pack("<32I", *values))
        self.assertEqual(decoded["vision_error"], -1)
        self.assertEqual(decoded["imu_z_mdps"], -2)
        self.assertEqual(decoded["yaw_mdeg"], -2147483648)

    def test_camera_state_and_diag_are_merged_by_poll(self):
        state = bytearray(36)
        state[0:4] = (123).to_bytes(4, "little")
        state[4:9] = bytes((1, 88, 4, 2, 1))
        diag = struct.pack("<8I", 456, 700, 800, 9, 10, 11, 120000, 3)
        app = self._poll_stub_app((
            TelemetryEvent(Frame(FrameType.STATE, 1, 1, 10, bytes(state)),
                           decode_camera_state_payload(bytes(state))),
            TelemetryEvent(Frame(FrameType.DIAG, 2, 1, 20, diag),
                           decode_camera_diag_payload(diag)),
        ))
        WirelessApp._poll(app)
        self.assertEqual(app.camera_state["camera_frame_id"], 456)
        self.assertEqual(app.camera_state["camera_quality"], 88)
        self.assertEqual(app.camera_state["camera_processing_us"], 700)
        self.assertEqual(app.camera_state["camera_flags"], 3)

    def test_signed_curve_sample_and_selectors(self):
        values = [0] * 32
        values[8] = (-16) & 0xFFFFFFFF
        values[10] = 5280
        values[11] = 5280
        values[12] = 7392
        values[13] = 7392
        values[19] = (-100) & 0xFFFFFFFF
        values[22] = (-200) & 0xFFFFFFFF
        payload = struct.pack("<32I", *values)
        app = self._poll_stub_app((
            TelemetryEvent(Frame(FrameType.STATE, 3, 1, 30, payload),
                           decode_state_payload(payload)),
        ))
        app.h743_config = {"gyro_rate_mdps": 120000}
        WirelessApp._poll(app)
        sample = app.plot_history[-1]
        self.assertLess(sample["gyro_actual"], 0)
        self.assertEqual(sample["gyro_config_plus"], 120000)
        self.assertEqual(sample["gyro_config_minus"], -120000)
        self.assertLess(sample["vision_error"], 0)

        no_config_app = self._poll_stub_app((
            TelemetryEvent(Frame(FrameType.STATE, 4, 1, 40, payload),
                           decode_state_payload(payload)),
        ))
        WirelessApp._poll(no_config_app)
        self.assertEqual(no_config_app.plot_history[-1]["gyro_config_plus"], 0)
        self.assertEqual(no_config_app.plot_history[-1]["gyro_config_minus"], 0)

        class Selector:
            def __init__(self, value):
                self.value = value
            def get(self):
                return self.value

        class Canvas:
            def __init__(self):
                self.lines = []
                self.text = []
            def winfo_width(self):
                return 320
            def winfo_height(self):
                return 120
            def delete(self, *_args):
                self.lines.clear()
                self.text.clear()
            def create_line(self, *args, **kwargs):
                self.lines.append((args, kwargs))
            def create_text(self, *args, **kwargs):
                self.text.append((args, kwargs))

        app.curve_canvas = Canvas()
        app.curve_kind = Selector("gyro")
        app.plot_history.append(dict(sample))
        app.plot_history.append(dict(sample, gyro_actual=100, vision_error=16))
        WirelessApp._draw_curve(app)
        self.assertTrue(any("gyro actual" in item[1].get("text", "")
                            for item in app.curve_canvas.text))
        self.assertTrue(any("config +rate" in item[1].get("text", "")
                            for item in app.curve_canvas.text))
        self.assertTrue(any("config -rate" in item[1].get("text", "")
                            for item in app.curve_canvas.text))
        app.curve_kind.value = "vision"
        WirelessApp._draw_curve(app)
        self.assertTrue(any("vision error" in item[1].get("text", "")
                            for item in app.curve_canvas.text))

    def test_camera_state_and_diag_payloads(self):
        state = bytearray(36)
        state[0:4] = (123).to_bytes(4, "little")
        state[4:9] = bytes((1, 88, 4, 2, 1))
        state[9:11] = (-12).to_bytes(2, "little", signed=True)
        state[11:13] = (34).to_bytes(2, "little", signed=True)
        state[13:17] = (7).to_bytes(4, "little")
        state[17] = 5
        state[18:20] = (900).to_bytes(2, "little")
        state[20:22] = bytes((3, 1))
        state[22:26] = (0xAABBCCDD).to_bytes(4, "little")
        state[26:28] = (9).to_bytes(2, "little")
        decoded = decode_camera_state_payload(bytes(state))
        self.assertEqual(decoded["camera_frame_id"], 123)
        self.assertEqual(decoded["camera_lateral_error"], -12)
        self.assertEqual(decoded["camera_session_token"], 0xAABBCCDD)

        diag = decode_camera_diag_payload(struct.pack("<8I", *range(8)))
        self.assertEqual(diag["camera_processing_us"], 1)
        self.assertEqual(diag["camera_flags"], 7)

    def test_image_dimensions_are_checked(self):
        header = struct.pack("<IIIHHBB", 7, 0, 2, 8, 1, 1, 0)
        with self.assertRaises(ProtocolError):
            decode_image_payload(header + b"\xA5\x00")

    def test_h743_timestamp_and_hello_nonce(self):
        class FakeSerial:
            def __init__(self):
                self.writes = []
            def write(self, value):
                self.writes.append(bytes(value))

        import queue
        client = WirelessClient(queue.Queue())
        client.serial = FakeSerial()
        client.hello_nonce = 0x12345678
        now = int(__import__("time").monotonic() * 1000) & 0xFFFFFFFF
        reply = Frame(FrameType.HELLO_REPLY, 4, 0xCAFEBABE,
                      (now - 250) & 0xFFFFFFFF,
                      struct.pack("<III", client.hello_nonce, 9, 0x3F))
        client._handle_frame(reply)
        self.assertEqual(client.session, 0xCAFEBABE)
        self.assertIsNotNone(client.h743_clock_offset_ms)
        client._send(FrameType.COMMAND, b"x")
        sent = decode_frame(client.serial.writes[-1])
        self.assertLess(abs(((now - 250) & 0xFFFFFFFF) - sent.timestamp_ms), 100)

        client.hello_nonce = 0x87654321
        old_session = client.session
        stale = Frame(FrameType.HELLO_REPLY, 5, 0x11111111, now,
                      struct.pack("<III", 0x12345678, 9, 0x3F))
        client._handle_frame(stale)
        self.assertEqual(client.session, old_session)
    def test_config_records_and_camera_sidecar(self):
        import zlib
        record = bytearray(128)
        struct.pack_into("<IIII", record, 0, 0x31534757, 1, 7, 50)
        struct.pack_into("<H", record, 20, 1000)
        struct.pack_into("<I", record, 22, 120000)
        struct.pack_into("<H", record, 26, 1000)
        struct.pack_into("<H", record, 58, 950)
        struct.pack_into("<I", record, 70, zlib.crc32(record[20:70]) & 0xFFFFFFFF)
        struct.pack_into("<I", record, 74, zlib.crc32(record[:74]) & 0xFFFFFFFF)
        decoded = decode_config_payload(bytes(record))
        self.assertEqual(decoded["config_target"], "h743")
        self.assertEqual(decoded["settings_generation"], 7)
        self.assertEqual(decoded["vision_d_gain_permille"], 950)

        camera = bytearray(16)
        struct.pack_into("<IIHH", camera, 0, 1, 4, 1100, 900)
        struct.pack_into("<I", camera, 12, zlib.crc32(camera[:12]) & 0xFFFFFFFF)
        decoded = decode_config_payload(bytes(camera))
        self.assertEqual(decoded["config_target"], "camera")
        self.assertEqual(decoded["vision_gain_permille"], 1100)

    def test_session_filter_queue_bound_and_image_ttl(self):
        import queue
        client = WirelessClient(queue.Queue(maxsize=1))
        client.session = 5
        frame = Frame(FrameType.STATE, 1, 6, 1, bytes(128))
        client._handle_frame(frame)
        self.assertEqual(client.events.qsize(), 1)
        client.requested_mode = 1
        stale_ack = bytes([0, OP_MODE, 0, 0]) + struct.pack("<III", 1, 0, 1)
        client._handle_frame(Frame(FrameType.ACK, 2, 6, 1, stale_ack))
        self.assertFalse(client.mode_complete)
        stale_image = struct.pack("<IIIHHBB", 1, 0, 600, 80, 60, 1, 1) + bytes(10)
        client._handle_frame(Frame(FrameType.IMAGE, 3, 6, 1, stale_image))
        self.assertIsNone(client.images.frame_id)
        client._emit(TelemetryEvent(frame, {"drop": 1}))
        self.assertLessEqual(client.events.qsize(), 1)

        assembler = ImageAssembler()
        first = {"frame_id": 1, "width": 16, "height": 1,
                 "format": 1, "flags": 1, "total": 2, "offset": 0}
        second = dict(first, flags=2, offset=1)
        assembler.add(first, b"\x00", 100)
        with self.assertRaises(ProtocolError):
            assembler.add(second, b"\x00", 60101)
    def test_image_metadata_bounds(self):
        header = struct.pack("<IIIHHBB", 7, 0, 1, 8, 1, 1, 0)
        self.assertEqual(decode_image_payload(header + b"\xA5")[1], b"\xA5")
        bad = bytearray(header)
        bad[4] = 0xFF
        with self.assertRaises(ProtocolError):
            decode_image_payload(bytes(bad) + b"\xA5")


if __name__ == "__main__":
    unittest.main()

"""Collect an explicitly authorized RAM entry probe; never install camera files."""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import sys
import time
import zlib

ROOT = Path(__file__).resolve().parents[1]


class EntryCollector:
    def __init__(self, directory: Path, stationary_check=False):
        self.directory = directory
        self.frames = []
        self.images = {}
        self.completed_images = []
        self.errors = []
        self.ready = False
        self.done = False
        self.done_summary = None
        self.pending = ""
        self.last_sequence = None
        self.stationary_check = stationary_check
        self.buffer_progress = None
        self.pull_ready = False
        self.frame_file = (directory / "frames.jsonl").open("w", encoding="utf8")

    def feed(self, text):
        self.pending += text
        while "\n" in self.pending:
            line, self.pending = self.pending.split("\n", 1)
            self.line(line.rstrip("\r"))

    def line(self, line):
        if line.startswith("ENTRY_READY_TO_PUSH=1"):
            if not self.ready:
                print("STATIONARY_GATE_PASSED" if self.stationary_check else
                      "READY_TO_PUSH: parked calibration passed; begin slow ground push now", flush=True)
            self.ready = True
        elif line.startswith("ENTRY_CONFIG "):
            config = json.loads(line[len("ENTRY_CONFIG "):])
            (self.directory / "entry-config.json").write_text(json.dumps(config, indent=2))
        elif line.startswith("ENTRY_BUFFERED "):
            self.buffer_progress = line
        elif line.startswith("ENTRY_PULL_READY "):
            self.pull_ready = True
        elif line.startswith("ENTRY_FRAME "):
            row = json.loads(line[len("ENTRY_FRAME "):])
            sequence = row["sequence"]
            if not isinstance(sequence, int) or not 0 <= sequence < 65536:
                raise ValueError("invalid entry sequence")
            if self.last_sequence is not None:
                delta = (sequence - self.last_sequence) & 65535
                if not 0 < delta < 32768:
                    raise ValueError("duplicate or backward entry sequence")
            for key, length in (("row_status", 120), ("row_edge2", 120),
                                ("baseline_valid", 60), ("baseline_left2", 60),
                                ("baseline_right2", 60), ("outer_shift_x2", 2)):
                if not isinstance(row.get(key), list) or len(row[key]) != length:
                    raise ValueError("entry array layout mismatch: " + key)
            self.frame_file.write(json.dumps(row, separators=(",", ":")) + "\n")
            self.frame_file.flush()
            self.frames.append(row)
            self.last_sequence = sequence
        elif line.startswith("ENTRY_IMAGE_META "):
            meta = json.loads(line[len("ENTRY_IMAGE_META "):])
            image_id = meta["image_id"]
            if (not isinstance(image_id, int) or not 0 <= image_id < 8 or
                    image_id in self.images or meta["width"] != 320 or
                    meta["height"] != 120 or meta["byte_count"] != 38400):
                raise ValueError("image metadata layout mismatch")
            self.images[image_id] = {"meta": meta, "data": bytearray()}
        elif line.startswith("ENTRY_IMAGE_CHUNK "):
            _, identity, offset, hex_data = line.split()
            image_id, offset = int(identity), int(offset)
            item = self.images[image_id]
            raw = bytes.fromhex(hex_data)
            if offset != len(item["data"]) or not 0 < len(raw) <= 384:
                raise ValueError("missing, duplicate or oversized image chunk")
            if offset + len(raw) > 38400:
                raise ValueError("image exceeds fixed size")
            item["data"].extend(raw)
        elif line.startswith("ENTRY_IMAGE_END "):
            image_id = int(line.split()[1])
            item = self.images[image_id]
            meta, raw = item["meta"], item["data"]
            if image_id in self.completed_images or len(raw) != 38400:
                raise ValueError("incomplete or repeated image")
            if "%08x" % (zlib.crc32(raw) & 0xffffffff) != str(meta["crc32"]).lower():
                raise ValueError("image CRC mismatch")
            if not any(x["sequence"] == meta["sequence"] for x in self.frames):
                raise ValueError("image has no corresponding boundary record")
            name = "image-%d-seq-%05d" % (image_id, meta["sequence"])
            (self.directory / (name + ".pgm")).write_bytes(b"P5\n320 120\n255\n" + raw)
            (self.directory / (name + ".json")).write_text(json.dumps(meta, indent=2))
            self.completed_images.append(image_id)
        elif line.startswith("ENTRY_CAPTURE_ERROR"):
            self.errors.append(line)
            print(line, flush=True)
        elif line.startswith("ENTRY_CAPTURE_DONE"):
            self.done_summary = {k: int(v) for k, v in
                                 (item.split("=", 1) for item in line.split()[1:])}
            if self.done_summary.get("error") != 0:
                self.errors.append("board reports capture error")
            self.done = True

    def close(self):
        self.frame_file.close()
        if self.pending.strip() and (self.pending.lstrip().startswith("ENTRY_") or not self.done):
            self.errors.append("unparsed terminal tail: " + self.pending[:200])
        missing = sorted(set(self.images) - set(self.completed_images))
        if self.done_summary is not None:
            if self.done_summary.get("frame_count") != len(self.frames):
                self.errors.append("board/host frame counts differ: stdout may have lost records")
            if self.done_summary.get("image_count") != len(self.completed_images):
                self.errors.append("board/host image counts differ")
        return {
            "ready": self.ready, "terminal_seen": self.done,
            "frames": len(self.frames), "images_verified": len(self.completed_images),
            "incomplete_image_ids": missing, "errors": self.errors,
            "first_sequence": self.frames[0]["sequence"] if self.frames else None,
            "last_sequence": self.last_sequence,
            "board_reported_counts": self.done_summary,
            "not_fps_acceptance": True, "not_vehicle_pairing_acceptance": True,
        }


def receive_pull_capture(call, collector, meta_channel, data_channel):
    """Pull immutable RAM objects by offset; require exact length and CRC."""
    size = call("_channel_size", meta_channel)
    if not isinstance(size, int) or not 0 < size <= 131072:
        raise ValueError("invalid RAM metadata size")

    def read_exact(channel, offset, length):
        if not isinstance(offset, int) or not isinstance(length, int) or offset < 0 or not 0 < length <= 2097152:
            raise ValueError("invalid RAM object range")
        result = bytearray()
        for start in range(0, length, 1024):
            wanted = min(1024, length-start)
            part = call("_channel_read", channel, offset+start, wanted)
            if not isinstance(part, (bytes, bytearray)) or len(part) != wanted:
                raise ValueError("short RAM channel read")
            result.extend(part)
        return bytes(result)

    meta = json.loads(read_exact(meta_channel, 0, size))
    if meta.get("schema") != "entry-pull-v1":
        raise ValueError("unsupported RAM capture schema")
    frames, images = meta["frames"], meta["images"]
    if len(frames) > 129 or len(images) > 8:
        raise ValueError("RAM capture exceeds fixed object bounds")
    (collector.directory / "pull-meta.json").write_text(json.dumps(meta, indent=2))
    total = call("_channel_size", data_channel)
    if not isinstance(total, int) or not 0 <= total <= 2097152:
        raise ValueError("RAM payload exceeds fixed bound")

    def object_bytes(item):
        if item["offset"]+item["size"] > total:
            raise ValueError("RAM object exceeds channel")
        raw = read_exact(data_channel, item["offset"], item["size"])
        if "%08x" % (zlib.crc32(raw)&0xffffffff) != str(item["crc32"]).lower():
            raise ValueError("RAM object CRC mismatch")
        return raw

    if meta.get("config") is not None:
        collector.line("ENTRY_CONFIG " + object_bytes(meta["config"]).decode("utf8"))
    for item in frames:
        text = object_bytes(item).decode("utf8")
        if json.loads(text)["sequence"] != item["sequence"]:
            raise ValueError("RAM frame sequence mismatch")
        collector.line("ENTRY_FRAME " + text)
    for item in images:
        raw = object_bytes(item)
        collector.line("ENTRY_IMAGE_META " + json.dumps(item))
        for offset in range(0, len(raw), 384):
            collector.line("ENTRY_IMAGE_CHUNK %d %d %s" % (
                item["image_id"], offset, raw[offset:offset+384].hex()))
        collector.line("ENTRY_IMAGE_END %d" % item["image_id"])
    collector.line("ENTRY_CAPTURE_DONE frame_count=%d processed_frame_count=%d image_count=%d error=%d" % (
        meta["frame_count"], meta["processed_frame_count"], meta["image_count"], meta["error"]))
    return meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--port", default="COM9")
    parser.add_argument("--stationary-check", action="store_true",
                        help="transport canary: operator must keep the camera still")
    args = parser.parse_args()
    source = args.source.resolve()
    manifest = json.loads(source.with_suffix(".manifest.json").read_text())
    raw = source.read_bytes()
    if hashlib.sha256(raw).hexdigest() != manifest["source_sha256"]:
        raise ValueError("RAM probe manifest mismatch")
    if manifest.get("physical_uart") is not False or manifest.get("flash_write") is not False:
        raise ValueError("only a no-physical-UART / no-Flash RAM probe is permitted")
    output = ROOT / "logs/diagnostics/ring-entry-probe" / time.strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True)
    sys.path[:0] = [str(ROOT), str(ROOT / "logs/diagnostics/perf-host-deps")]
    from tools.benchmark_openmv_preview import load_camera_factory, _bounded_camera_call
    camera = load_camera_factory(ROOT / "logs/diagnostics/openmv-sdk/src")(
        args.port, timeout=1, max_retry=2)
    calls = defaultdict(list)
    collector = EntryCollector(output, stationary_check=args.stationary_check)
    error = None

    def call(name, *values, deadline=5):
        return _bounded_camera_call(camera, name, values, deadline, min(deadline, 1), calls)

    try:
        call("connect", deadline=12)
        call("stop")
        call("streaming", False)
        for _ in range(12):
            if not call("read_status").get("stdout"):
                break
            call("read_stdout")
        call("exec", raw.decode("utf8"), deadline=45)
        print("RAM_ENTRY_CAPTURE_STARTED output=" + str(output), flush=True)
        deadline = time.monotonic() + int(manifest.get("run_limit_s", 60)) + 185
        progress = time.monotonic()
        channel_check = time.monotonic()
        with (output / "stdout.log").open("w", encoding="utf8") as log:
            while time.monotonic() < deadline:
                status = call("read_status")
                if status.get("stdout"):
                    chunk = call("read_stdout") or ""
                    log.write(chunk)
                    log.flush()
                    collector.feed(chunk)
                if collector.pull_ready or time.monotonic()-channel_check >= 2:
                    call("update_channels")
                    meta_channel = call("get_channel", "entry_meta")
                    data_channel = call("get_channel", "entry_data")
                    if meta_channel is not None and data_channel is not None:
                        receive_pull_capture(call, collector, meta_channel, data_channel)
                    channel_check = time.monotonic()
                if collector.done:
                    break
                if time.monotonic() - progress >= 5:
                    last = collector.frames[-1] if collector.frames else {}
                    print(json.dumps({"frames": len(collector.frames), "state": last.get("state"),
                                      "reason": last.get("reason"), "arc_rejection": last.get("arc_rejection"),
                                      "board_buffer": collector.buffer_progress}), flush=True)
                    progress = time.monotonic()
                time.sleep(.002)
    except BaseException as exc:
        error = repr(exc)
        print("CAPTURE_HOST_ERROR", error, flush=True)
    finally:
        if error is None and camera.is_connected():
            try:
                call("stop")
            except BaseException:
                pass
        camera.disconnect()
        report = collector.close()
        report.update(source_sha256=hashlib.sha256(raw).hexdigest(), host_error=error,
                      calls_ms=dict(calls), physical_uart=False, flash_write=False,
                      ram_left_for_recovery_on_host_error=error is not None)
        (output / "report.json").write_text(json.dumps(report, indent=2))
        print("REPORT", output, flush=True)
    if error or not report["terminal_seen"] or not report["ready"] or report["errors"] or report["incomplete_image_ids"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

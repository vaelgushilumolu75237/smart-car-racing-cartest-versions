"""Exact whole-frame red count; one R per continuous >4% event."""
WIDTH = 320
HEIGHT = 120
RED_PIXEL_LIMIT = 1536


def red_regions(frame, threshold):
    if frame.width() != WIDTH or frame.height() != HEIGHT:
        raise ValueError("red detector requires 320x120")
    # Unit stride visits all pixels; retain every component, including 1 pixel.
    # pixels() counts classified pixels, never bounding-box area.
    blobs = frame.find_blobs(
        [threshold], x_stride=1, y_stride=1, pixels_threshold=1,
        area_threshold=1, merge=False)
    return blobs


def count_red_pixels(frame, threshold):
    return sum(blob.pixels() for blob in red_regions(frame, threshold))


def draw_red_boxes(preview, blobs, pixels, scale, x, y):
    color = (0, 255, 0) if pixels > RED_PIXEL_LIMIT else (0, 0, 255)
    for blob in blobs:
        bx, by, bw, bh = blob.rect()
        preview.draw_rectangle(int(x + bx * scale), int(y + by * scale),
                               max(1, int(bw * scale)), max(1, int(bh * scale)),
                               color=color, thickness=2)


class RedEvent:
    def __init__(self):
        self.sent = False

    def update(self, pixels, uart):
        if pixels <= RED_PIXEL_LIMIT:
            self.sent = False
            return False
        if not self.sent and uart.write(b"R") == 1:
            self.sent = True
            return True
        return False

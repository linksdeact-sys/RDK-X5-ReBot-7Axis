import time
import unittest

from panel.camera_preview import BOUNDARY, CameraPreview, JpegParts, MAX_JPEG


def part(frame):
    return BOUNDARY + b"Content-type: image/jpeg\r\nContent-length: " + str(len(frame)).encode() + b"\r\n\r\n" + frame


class CameraTests(unittest.TestCase):
    def test_split_headers_and_two_frames(self):
        frames = [b"\xff\xd8abc\xff\xd9", b"\xff\xd8def\xff\xd9"]
        stream = b"\r\n".join(part(frame) for frame in frames)
        parser = JpegParts()
        result = []
        for i in range(0, len(stream), 7):
            result.extend(parser.feed(stream[i:i+7]))
        self.assertEqual(result, frames)

    def test_rejects_oversized_frame(self):
        data = BOUNDARY + f"Content-length: {MAX_JPEG+1}\r\n\r\n".encode()
        with self.assertRaises(ValueError):
            list(JpegParts().feed(data))

    def test_no_preview_before_manual_start(self):
        preview = CameraPreview()
        self.assertIsNone(preview.worker)
        with self.assertRaises(RuntimeError):
            preview.frame(None)

    def test_wrong_lease_cannot_view_or_stop(self):
        preview = CameraPreview()
        preview.lease = "owner"
        preview.last_request = preview.frame_at = time.monotonic()
        preview.latest = b"latest JPEG"
        with self.assertRaises(RuntimeError):
            preview.frame("different")
        preview.stop("different")
        self.assertEqual(preview.frame("owner"), b"latest JPEG")
        preview.stop("owner")
        self.assertIsNone(preview.latest)

    def test_stale_frame_not_served_and_expired_lease_rejected(self):
        preview = CameraPreview()
        preview.lease = "owner"
        preview.last_request = time.monotonic()
        preview.latest = b"old data"
        self.assertIsNone(preview.frame("owner"))
        preview.last_request -= 10
        with self.assertRaises(RuntimeError):
            preview.frame("owner")

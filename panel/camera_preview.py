"""Single-viewer, in-memory RGB preview; no motor integration."""

import os
from pathlib import Path
import re
import secrets
import select
import stat
import subprocess
import threading
import time


BOUNDARY = b"--rebotframe\r\n"
MAX_JPEG = 512 * 1024
LEASE_SECONDS = 4
FRAME_MAX_AGE = 0.8


def d435i_color_node(video_root=Path("/sys/class/video4linux"), dev_root=Path("/dev")):
    devices = []
    for entry in video_root.glob("video*"):
        if not re.fullmatch(r"video\d+", entry.name):
            continue
        try:
            if (entry / "index").read_text().strip() != "0":
                continue
            ancestors = entry.resolve(strict=True).parents
            interface = next(p for p in ancestors if (p / "bInterfaceNumber").is_file())
            usb = interface.parent
            if ((usb / "idVendor").read_text().strip().lower(),
                (usb / "idProduct").read_text().strip().lower(),
                (interface / "bInterfaceNumber").read_text().strip()) != ("8086", "0b3a", "03"):
                continue
            device = dev_root / entry.name
            if stat.S_ISCHR(device.stat().st_mode):
                devices.append(device)
        except (OSError, StopIteration):
            continue
    if len(devices) != 1:
        raise RuntimeError("未找到唯一的 D435i 彩色设备，请检查 USB 连接")
    return devices[0]


def capture_command(simulated):
    command = ["nice", "-n", "10", "ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error",
               "-filter_threads", "1"]
    if simulated:
        command += ["-re", "-f", "lavfi", "-i", "testsrc2=size=640x480:rate=6"]
    else:
        command += ["-f", "v4l2", "-input_format", "yuyv422", "-video_size", "640x480",
                    "-framerate", "6", "-i", str(d435i_color_node())]
    return command + ["-an", "-threads", "1", "-c:v", "mjpeg", "-q:v", "7", "-f", "mpjpeg",
                      "-boundary_tag", "rebotframe", "-flush_packets", "1", "pipe:1"]


class JpegParts:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while self.buffer:
            # FFmpeg separates each payload from the next boundary with CRLF.
            if self.buffer.startswith(b"\r\n"):
                del self.buffer[:2]
            header_end = self.buffer.find(b"\r\n\r\n")
            if header_end < 0:
                if len(self.buffer) > 512:
                    raise ValueError("Invalid JPEG part header")
                return
            header = self.buffer[:header_end]
            match = re.search(rb"\r\nContent-length: ([0-9]{1,6})$", header, re.I)
            if header_end > 512 or not header.startswith(BOUNDARY) or not match:
                raise ValueError("Invalid JPEG part")
            size = int(match[1])
            if not 0 < size <= MAX_JPEG:
                raise ValueError("Invalid JPEG part length")
            offset = header_end + 4
            if len(self.buffer) < offset + size:
                return
            jpeg = bytes(self.buffer[offset:offset + size])
            del self.buffer[:offset + size]
            if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
                raise ValueError("Invalid JPEG frame")
            yield jpeg


class CameraPreview:
    def __init__(self, simulated=False):
        self.simulated = simulated
        self.lock = threading.Lock()
        self.lease = None
        self.last_request = 0.0
        self.latest = None
        self.frame_at = 0.0
        self.error = None
        self.worker = None
        self.closing = False

    def start(self):
        with self.lock:
            if self.closing or self.worker is not None:
                raise RuntimeError("预览正在使用或结束中，请稍后重试")
            self.lease = secrets.token_urlsafe(24)
            self.last_request = time.monotonic()
            self.latest = None
            self.frame_at = 0.
            self.error = None
            self.worker = threading.Thread(target=self._capture, args=(self.lease,),
                                           daemon=True, name="d435i-preview")
            self.worker.start()
            return {"lease": self.lease, "simulated": self.simulated}

    def frame(self, lease):
        with self.lock:
            now = time.monotonic()
            if not lease or self.lease != lease or now - self.last_request > LEASE_SECONDS:
                raise RuntimeError(self.error or "预览已结束，请重新开启")
            self.last_request = now
            return self.latest if now - self.frame_at <= FRAME_MAX_AGE else None

    def stop(self, lease):
        with self.lock:
            if lease and self.lease == lease:
                self.lease = None
                self.latest = None
        return {"ok": True}

    def close(self):
        with self.lock:
            self.closing = True
            self.lease = None
            self.latest = None
            worker = self.worker
        if worker is not None:
            worker.join(timeout=3)

    def _capture(self, lease):
        process = None
        error = None
        try:
            process = subprocess.Popen(capture_command(self.simulated), stdout=subprocess.PIPE,
                                       stderr=subprocess.DEVNULL)
            decoder = JpegParts()
            last_frame = time.monotonic()
            while True:
                now = time.monotonic()
                with self.lock:
                    if self.lease != lease or now - self.last_request > LEASE_SECONDS:
                        break
                if now - last_frame > 5:
                    raise RuntimeError("D435i 画面超时，请检查 USB 后重试")
                if not select.select([process.stdout], [], [], .2)[0]:
                    continue
                data = os.read(process.stdout.fileno(), 65536)
                if not data:
                    raise RuntimeError("D435i 采集已断开或设备被占用")
                for jpeg in decoder.feed(data):
                    last_frame = time.monotonic()
                    with self.lock:
                        if self.lease == lease:
                            self.latest, self.frame_at = jpeg, last_frame
        except RuntimeError as exc:
            error = str(exc)
        except (OSError, ValueError):
            error = "无法采集彩色画面，请检查 D435i 和 FFmpeg"
        finally:
            try:
                if process is not None:
                    try:
                        process.terminate()
                        process.wait(timeout=.6)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=.6)
                    finally:
                        process.stdout.close()
            except (OSError, subprocess.TimeoutExpired):
                error = "D435i 采集进程未正常退出"
            finally:
                with self.lock:
                    self.lease = None
                    self.latest = None
                    self.error = error
                    self.worker = None

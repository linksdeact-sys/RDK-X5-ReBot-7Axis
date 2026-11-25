"""Read only the KP20 joystick and its periodic HID reports. No CAN access."""

import array
import copy
import fcntl
import glob
import math
import os
from pathlib import Path
import select
import struct
import threading
import time


TRIGGER_DEADZONE = 0.10
# ABS_BRAKE=10 is LT, ABS_GAS=9 is RT on the BFM receiver.
_TRIGGER_CODES = {"lt": 10, "rt": 9}


def _valid_triggers(triggers):
    return isinstance(triggers, dict) and all(
        type(triggers.get(name)) in (int, float) and 0.0 <= triggers[name] <= 1.0
        for name in _TRIGGER_CODES)


def triggers_neutral(triggers):
    """Require actual released readings, not just a zero net intent."""
    return _valid_triggers(triggers) and all(
        triggers[name] < TRIGGER_DEADZONE for name in _TRIGGER_CODES)


def valid_triggers(triggers):
    return _valid_triggers(triggers)


def trigger_intent(triggers):
    """Positive closes (RT); negative opens (LT); dual press is inactive."""
    if not _valid_triggers(triggers):
        return 0.0
    lt, rt = triggers["lt"], triggers["rt"]
    if (lt >= TRIGGER_DEADZONE) == (rt >= TRIGGER_DEADZONE):
        return 0.0
    return rt - lt


def _trigger_levels(axes):
    if not all(code in axes for code in _TRIGGER_CODES.values()):
        return None
    return {name: max(0.0, min(1.0, (axes[code] / 32767.0 + 1.0) / 2.0))
            for name, code in _TRIGGER_CODES.items()}


def _map_intent(axes, buttons, triggers):
    """BFM Linux codes -> joint directions verified on this particular arm."""
    def axis(code):
        return max(-1.0, min(1.0, axes.get(code, 0) / 32767.0))

    return [-axis(0), axis(1), axis(5), axis(17), axis(2),
            float(buttons.get(307, 0)) - float(buttons.get(308, 0)),
            trigger_intent(triggers)]


def map_intent(axes, buttons):
    return _map_intent(axes, buttons, _trigger_levels(axes))


class SimGamepad:
    def __init__(self):
        self.lock = threading.Lock()
        self.value = {"connected": True, "name": "KP20 simulated", "deadman": False,
                      "stop": False, "intent": [0.0]*7, "raw_axes": {}, "buttons": [],
                      "triggers": {"lt": 0.0, "rt": 0.0}}

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy(self.value)
        if not value["connected"]:
            value.update(deadman=False, intent=[0.0]*7, triggers=None)
        return {**value, "report_age_ms": 0.0, "error": None}

    def inject(self, body):
        intent = body.get("intent", [0.0]*7)
        if not isinstance(intent, list) or len(intent) != 7 or any(
                type(v) not in (int, float) or not -1 <= v <= 1 for v in intent):
            raise ValueError("Invalid simulated axes")
        triggers = body.get("triggers", {"lt": 0.0, "rt": 0.0})
        if not _valid_triggers(triggers):
            raise ValueError("Invalid simulated triggers; provide lt and rt in [0, 1]")
        triggers = {name: float(triggers[name]) for name in _TRIGGER_CODES}
        contact = body.get("gripper_object_rad", self.value.get("gripper_object_rad"))
        if contact is not None and (type(contact) not in (int, float) or not .85 <= contact <= 4.22):
            raise ValueError("Invalid simulated gripper object position")
        velocity=body.get("gripper_velocity_rad_s")
        teaching_q=body.get("teaching_q",self.value.get("teaching_q"))
        if teaching_q is not None:
            from .teaching import pose
            teaching_q=pose(teaching_q)
        if velocity is not None and (type(velocity) not in (int,float) or not math.isfinite(velocity) or abs(velocity)>2.5):
            raise ValueError("Invalid simulated gripper velocity")
        with self.lock:
            self.value.update({"intent": intent[:6] + [trigger_intent(triggers)], "triggers": triggers,
                               "deadman": body.get("deadman") is True,
                               "stop": body.get("stop") is True,
                               "rx_fault_axis": body.get("rx_fault_axis"),
                               "gripper_object_rad": contact,
                               "gripper_velocity_rad_s":velocity,
                               "teaching_q":teaching_q,
                               "connected": body.get("connected", True) is True})

    def close(self):
        pass


class GamepadReader:
    def __init__(self):
        self.lock = threading.Lock()
        self.closing = threading.Event()
        self.value = {"connected": False, "name": "未找到鲲鹏20", "deadman": False,
                      "stop": False, "intent": [0.0]*7, "raw_axes": {}, "buttons": [],
                      "triggers": None, "report_at": 0.0, "error": "接收机尚未就绪"}
        self.thread = threading.Thread(target=self.run, name="kp20-input", daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            value = copy.deepcopy(self.value)
        age = time.monotonic() - value.pop("report_at")
        value["report_age_ms"] = round(age*1000, 1) if age < 3600 else None
        value["connected"] = value["connected"] and age < 0.15
        if not value["connected"]:
            value.update(deadman=False, intent=[0.0]*7, triggers=None)
        return value

    def run(self):
        while not self.closing.is_set():
            handles = []
            try:
                for path in sorted(glob.glob("/dev/input/js*")):
                    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                    handles.append(fd)
                    name = bytearray(128)
                    fcntl.ioctl(fd, 0x80806A13, name)
                    name = name.decode(errors="replace").strip("\x00")
                    if "BTP-KP20" in name and "BFM" in name:
                        break
                    os.close(handles.pop())
                else:
                    raise RuntimeError("未找到鲲鹏20 BFM 模式接收机")
                axis_map = array.array("B", [0]*64)
                button_map = array.array("H", [0]*512)
                fcntl.ioctl(fd, 0x80406A32, axis_map)
                fcntl.ioctl(fd, 0x84006A34, button_map)
                has_trigger_axes = all(code in axis_map for code in _TRIGGER_CODES.values())
                device = Path("/sys/class/input") / Path(path).name / "device"
                hid = next(parent for parent in device.resolve().parents if "20BC:5128." in parent.name.upper())
                raw_name = next((hid / "hidraw").glob("hidraw*"))
                raw_fd = os.open("/dev/" + raw_name.name, os.O_RDONLY | os.O_NONBLOCK)
                handles.append(raw_fd)
                # Wait for actual joystick initial events. A reconnect with
                # one held trigger must not look like both were released.
                axes = {}
                buttons = {}
                report_at = 0.0
                while not self.closing.is_set():
                    ready, _, _ = select.select(handles, [], [], 0.02)
                    if raw_fd in ready:
                        while True:
                            try:
                                report = os.read(raw_fd, 256)
                            except BlockingIOError:
                                break
                            if not report:
                                raise OSError("手柄 HID 已断开")
                            report_at = time.monotonic()
                    if fd in ready:
                        while True:
                            try:
                                events = os.read(fd, 4096)
                            except BlockingIOError:
                                break
                            if not events:
                                raise OSError("手柄摇杆已断开")
                            for offset in range(0, len(events), 8):
                                _, value, kind, number = struct.unpack("IhBB", events[offset:offset+8])
                                if kind & 2:
                                    axes[axis_map[number]] = value
                                elif kind & 1:
                                    buttons[button_map[number]] = value
                    fresh = time.monotonic()-report_at < 0.15
                    triggers = _trigger_levels(axes) if fresh and has_trigger_axes else None
                    with self.lock:
                        self.value = {"connected": fresh, "name": name,
                                      "deadman": bool(buttons.get(310)),
                                      "stop": bool(buttons.get(305) or buttons.get(314)),
                                      "intent": _map_intent(axes, buttons, triggers), "raw_axes": dict(axes),
                                      "triggers": triggers,
                                      "buttons": [code for code, value in buttons.items() if value],
                                      "report_at": report_at,
                                      "error": None if fresh else "手柄报告超过150ms未更新"}
            except Exception as exc:
                with self.lock:
                    self.value.update(connected=False, deadman=False, intent=[0.0]*7,
                                      triggers=None, error=str(exc))
                self.closing.wait(0.5)
            finally:
                for fd in handles:
                    os.close(fd)

    def close(self):
        self.closing.set()
        self.thread.join(timeout=2)

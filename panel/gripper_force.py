"""Manual single-opening force verification. No SDK, no sensor-force claims."""
import hashlib
import json
import math
import threading
import time

from .gripper_hold import HOLD_TORQUE_MAX, HOLD_TRAVEL


def identity(points):
    data = {"model": "4310", "id": 7, "feedback_id": 23, "points": points}
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def profile_torque(profile, points):
    if (not isinstance(profile, dict) or type(profile.get("version")) is not int or profile.get("version") != 1
            or profile.get("identity") != identity(points)):
        return None
    tau, readings = profile.get("hold_torque_nm"), profile.get("readings_n")
    position = profile.get("position_rad")
    if (type(tau) not in (int, float) or not math.isfinite(tau) or not .02 <= tau <= HOLD_TORQUE_MAX
            or profile.get("target_force_n") != 3. or not isinstance(readings, list) or len(readings) != 3
            or any(type(v) not in (int, float) or not math.isfinite(v) or not 2.5 <= v <= 3.5 for v in readings)
            or type(position) not in (int, float) or not math.isfinite(position)
            or not points.get("open", math.inf) < position < points.get("closed", -math.inf)
            or profile.get("source") != "operator_gauge_single_opening"):
        return None
    return float(tau)


class ForceCalibration:
    def __init__(self, points):
        self.lock = threading.RLock()
        self.points = dict(points)
        self.ready = False
        self.position = None
        self.last_tick = 0.
        self.pulse = None
        self.last_pulse = None
        self.next_id = 1
        self.records = []
        self.commit_requested = False
        self.committed = False
        self.aborted = False

    def tick(self, *, ready, armed, safe, dt, position, command_tau=0.):
        with self.lock:
            self.ready = ready and safe and not self.aborted
            self.position = position if self.ready else None
            self.last_tick = time.monotonic()
            if self.pulse:
                if (not self.ready or not armed or dt > .04 or
                        abs(position-self.pulse["position_rad"]) > HOLD_TRAVEL):
                    self.last_pulse = {**self.pulse, "complete": False, "error": "测力中断，未记录"}
                    self.pulse = None
                else:
                    self.pulse["elapsed_s"] += dt
                    if abs(command_tau-self.pulse["torque_nm"]) <= 1e-4:
                        self.pulse["steady_s"] += dt
                    if self.pulse["elapsed_s"] >= 2.:
                        self.last_pulse = {**self.pulse, "complete": self.pulse["steady_s"] >= .75}
                        self.pulse = None
            return self.pulse["torque_nm"] if self.pulse else 0.

    def request_pulse(self, tau):
        if (type(tau) not in (int, float) or not math.isfinite(tau) or
                not .02 <= tau <= HOLD_TORQUE_MAX or abs(tau*100-round(tau*100)) > 1e-7):
            raise ValueError("测力力矩需为0.02～0.20Nm，步长0.01Nm；不是牛顿输入")
        with self.lock:
            if not self.ready or time.monotonic()-self.last_tick > .15 or self.aborted or self.commit_requested:
                raise RuntimeError("请先夹住测力工具并松开RT确认接触")
            if self.pulse:
                raise RuntimeError("当前2秒测力脉冲尚未结束")
            if len(self.records) >= 3:
                raise RuntimeError("已完成三次读数，请停止失能并保存")
            if self.records and abs(self.records[0]["torque_nm"]-tau) > 1e-9:
                self.records = []
            self.pulse = {"id": self.next_id, "torque_nm": float(tau), "position_rad": self.position,
                          "elapsed_s": 0., "steady_s": 0.}
            self.next_id += 1
            return self.pulse["id"]

    def record(self, pulse_id, force):
        if type(pulse_id) is not int or pulse_id <= 0:
            raise ValueError("测力脉冲编号无效")
        if type(force) not in (int, float) or not math.isfinite(force) or not 2.5 <= force <= 3.5:
            raise ValueError("仅接受实测2.5～3.5N；超过范围请调整力矩重新试夹，不可冒充已达到3N")
        with self.lock:
            pulse = self.last_pulse
            if (self.pulse or self.aborted or self.commit_requested or not pulse or
                    not pulse.get("complete") or pulse["id"] != pulse_id
                    or any(r["pulse_id"] == pulse_id for r in self.records)):
                raise RuntimeError("需记录刚完成且尚未记录的完整测力脉冲")
            if (not self.ready or time.monotonic()-self.last_tick > .15 or self.position is None
                    or abs(self.position-pulse["position_rad"]) > HOLD_TRAVEL):
                raise RuntimeError("接触已变化，请重新确认并测量")
            if self.records and abs(self.records[0]["position_rad"]-pulse["position_rad"]) > HOLD_TRAVEL:
                raise RuntimeError("三次测量需在同一开口附近进行")
            if len(self.records) >= 3:
                raise RuntimeError("已记录三次，请停止失能并保存")
            self.records.append({"pulse_id": pulse_id, "torque_nm": pulse["torque_nm"],
                                 "position_rad": pulse["position_rad"], "force_n": float(force)})

    def payload(self):
        with self.lock:
            if len(self.records) != 3 or self.pulse or self.aborted:
                raise RuntimeError("需要同一力矩、同一开口的三次完整测力记录")
            if len({r["torque_nm"] for r in self.records}) != 1:
                raise RuntimeError("三次测量的保持力矩必须一致")
            return {"version": 1, "identity": identity(self.points), "target_force_n": 3.,
                    "hold_torque_nm": self.records[0]["torque_nm"],
                    "readings_n": [r["force_n"] for r in self.records],
                    "recorded_pulse_ids": [r["pulse_id"] for r in self.records],
                    "position_rad": self.records[0]["position_rad"], "verified_at": time.time(),
                    "source": "operator_gauge_single_opening"}

    def abort(self):
        with self.lock:
            self.aborted, self.pulse, self.ready = True, None, False
            self.commit_requested = False

    def snapshot(self):
        with self.lock:
            return {"contact_ready": self.ready, "position_rad": self.position,
                    "pulse": dict(self.pulse) if self.pulse else None,
                    "last_pulse": dict(self.last_pulse) if self.last_pulse else None,
                    "readings_n": [r["force_n"] for r in self.records],
                    "recorded_pulse_ids": [r["pulse_id"] for r in self.records],
                    "commit_requested": self.commit_requested, "committed": self.committed,
                    "aborted": self.aborted, "target_force_n": 3.}

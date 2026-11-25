"""Run one bounded J1 round trip using the official MotorBridge SDK."""

import json
import math
import signal
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import yaml
from motorbridge import Controller, Mode


def interrupt(signum, frame):
    raise InterruptedError(f"Received signal {signum}")


def read_state(motor, controller):
    motor.request_feedback()
    time.sleep(0.008)
    controller.poll_feedback_once()
    return motor.get_state()


def main():
    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    config = yaml.safe_load(Path("config/rebotarm_dm.yaml").read_text())
    joint = next(joint for joint in config["joints"] if joint["name"] == "joint1")
    controller = Controller.from_dm_device(config["dm_device_type"], str(config["dm_channel"]))
    motor = None
    rows = []
    timestamp = datetime.now(ZoneInfo("Asia/Tokyo"))
    directory = Path("calibration_snapshots")
    directory.mkdir(exist_ok=True)
    report = directory / ("j1_test_" + timestamp.strftime("%Y%m%dT%H%M%S") + ".json")
    try:
        motor = controller.add_damiao_motor(joint["motor_id"], joint["feedback_id"], joint["model"])
        time.sleep(0.3)
        state = None
        for _ in range(100):
            state = read_state(motor, controller)
            if state is not None:
                break
        if state is None or state.status_code != 0:
            raise RuntimeError("J1 must respond and be disabled before this test")
        origin = state.pos
        motor.ensure_mode(Mode.MIT, 1000)
        delta = math.radians(3.0)
        print(f"J1 origin={math.degrees(origin):+.3f}deg delta=+3deg kp=40 kd=2", flush=True)
        motor.enable()
        for _ in range(100):
            state = read_state(motor, controller)
            if state is not None and state.status_code == 1:
                break
        if state is None or state.status_code != 1:
            raise RuntimeError(f"J1 enable was not confirmed: {state}")
        start = time.monotonic()
        next_tick = start
        last_log = -1.0
        actual_peak = origin
        last_state = state
        while True:
            elapsed = time.monotonic() - start
            if elapsed >= 8.0:
                break
            if elapsed < 1.0:
                phase = "engage"
                blend = elapsed
                fraction = 0.0
            elif elapsed < 4.0:
                phase = "out"
                blend = 1.0
                u = (elapsed - 1.0) / 3.0
                fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
            elif elapsed < 7.0:
                phase = "return"
                blend = 1.0
                u = (elapsed - 4.0) / 3.0
                fraction = 1.0 - (10 * u**3 - 15 * u**4 + 6 * u**5)
            else:
                phase = "release"
                blend = 8.0 - elapsed
                fraction = 0.0
            target = origin + delta * fraction
            motor.send_mit(target, 0.0, 40.0 * blend, 2.0, 0.0)
            last_state = read_state(motor, controller)
            if last_state is None or last_state.status_code != 1:
                raise RuntimeError(f"Unexpected J1 feedback: {last_state}")
            if abs(last_state.pos - origin) > math.radians(6) or abs(last_state.vel) > 0.3:
                raise RuntimeError("J1 exceeded the bounds of the small-angle test")
            actual_peak = max(actual_peak, last_state.pos)
            if elapsed - last_log >= 0.2:
                row = {"t": elapsed, "phase": phase, "target_rad": target, **asdict(last_state)}
                rows.append(row)
                print(
                    f"{elapsed:4.1f}s {phase:7s} "
                    f"target={math.degrees(target):+.2f}deg "
                    f"actual={math.degrees(last_state.pos):+.2f}deg "
                    f"v={last_state.vel:+.3f} torque={last_state.torq:+.3f}",
                    flush=True,
                )
                last_log = elapsed
            next_tick += 0.01
            time.sleep(max(0.0, next_tick - time.monotonic()))
        print(
            f"RESULT peak_delta={math.degrees(actual_peak-origin):+.3f}deg "
            f"return_error={math.degrees(last_state.pos-origin):+.3f}deg",
            flush=True,
        )
    finally:
        try:
            if motor is not None:
                motor.disable()
                print("J1 disable sent", flush=True)
        finally:
            controller.close()
            report.write_text(
                json.dumps({"timestamp": timestamp.isoformat(), "samples": rows}, indent=2),
                encoding="utf-8",
            )
            print(f"Report: {report}", flush=True)


if __name__ == "__main__":
    main()

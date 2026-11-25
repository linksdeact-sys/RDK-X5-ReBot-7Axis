"""Set the supported physical pose as zero using the official RebotArm API."""

import json
import math
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

from reBotArm_control_py.actuator import RebotArm


def snapshot(arm):
    for _ in range(3):
        for motor in arm._motor_map.values():
            motor.request_feedback()
            for controller in arm._ctrl_map.values():
                controller.poll_feedback_once()
            time.sleep(0.02)
    return {name: motor.get_state() for name, motor in arm._motor_map.items()}


def main():
    arm = RebotArm()
    arm.connect()
    try:
        before = snapshot(arm)
        if any(state is None or state.status_code != 0 for state in before.values()):
            raise RuntimeError("Zero setting requires feedback from all seven disabled motors")
        directory = Path(__file__).resolve().parent / "calibration_snapshots"
        directory.mkdir(exist_ok=True)
        timestamp = datetime.now(ZoneInfo("Asia/Tokyo"))
        backup = directory / ("zero_" + timestamp.strftime("%Y%m%dT%H%M%S") + ".json")
        record = {
            "timestamp": timestamp.isoformat(),
            "pose_reference": "Official DM URDF neutral pose, manually aligned by user",
            "before": {name: asdict(state) for name, state in before.items()},
        }
        backup.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Pre-zero readings saved to {backup}", flush=True)
        arm.set_zero()
        after = snapshot(arm)
        record["after"] = {
            name: asdict(state) if state is not None else None
            for name, state in after.items()
        }
        backup.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
        verified = True
        for name, state in after.items():
            if state is None:
                print(f"{name} NO_FEEDBACK", flush=True)
                verified = False
                continue
            angle = math.degrees(state.pos)
            print(f"{name} angle={angle:+.3f}deg status={state.status_code}", flush=True)
            verified = verified and state.status_code == 0 and abs(angle) < 1.0
        print("ZERO_VERIFIED" if verified else "ZERO_CHECK_REQUIRED", flush=True)
        return 0 if verified else 2
    finally:
        for controller in arm._ctrl_map.values():
            controller.close()


if __name__ == "__main__":
    raise SystemExit(main())

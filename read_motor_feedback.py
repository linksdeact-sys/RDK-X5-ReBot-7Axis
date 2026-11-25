"""Query seven Damiao motors through the official MotorBridge SDK."""

import argparse
import math
import time
from pathlib import Path

import yaml
from motorbridge import Controller


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/rebotarm_dm.yaml"),
    )
    parser.add_argument("--transport", choices=["dm-serial", "dm-device"])
    parser.add_argument("--device-type")
    parser.add_argument("--dm-channel")
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    transport = args.transport or config.get("transport", "dm-serial")
    device_type = args.device_type or config.get("dm_device_type", "usb2canfd")
    dm_channel = args.dm_channel or str(config.get("dm_channel", "0"))
    channel = config["channel"]
    if transport == "dm-device":
        print(f"Opening {device_type}, channel {dm_channel}", flush=True)
        controller = Controller.from_dm_device(device_type, dm_channel)
    else:
        print(f"Opening {channel}, serial baud 921600", flush=True)
        controller = Controller.from_dm_serial(channel, 921600)
    try:
        motors = [
            (
                joint,
                controller.add_damiao_motor(
                    joint["motor_id"], joint["feedback_id"], joint["model"]
                ),
            )
            for joint in config["joints"]
        ]
        for _ in range(5):
            for _, motor in motors:
                motor.request_feedback()
                controller.poll_feedback_once()
                time.sleep(0.01)
                controller.poll_feedback_once()

        received = 0
        for joint, motor in motors:
            state = motor.get_state()
            label = (
                f"{joint['name']} model={joint['model']} "
                f"id=0x{joint['motor_id']:02X} "
                f"feedback=0x{joint['feedback_id']:02X}"
            )
            if state is None:
                print(f"{label} NO_FEEDBACK", flush=True)
                continue
            received += 1
            print(
                f"{label} status=0x{state.status_code:X} "
                f"pos={math.degrees(state.pos):+.2f}deg "
                f"vel={state.vel:+.4f}rad/s torque={state.torq:+.3f}Nm "
                f"temp={state.t_mos:.0f}/{state.t_rotor:.0f}C "
                f"rx=0x{state.arbitration_id:X}",
                flush=True,
            )
        print(f"Feedback received from {received}/{len(motors)} motors", flush=True)
        return 0 if received == len(motors) else 2
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())

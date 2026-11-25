"""Operator-experience soft gates; hardware validity limits stay separate."""
GRIP_CONTROL_PAUSE_S=.100
GRIP_SOFT_SPEED_RAD_S=.9
ARM_SOFT_SPEED_RAD_S=.8
ARM_IMMEDIATE_SPEED_RAD_S=1.5
ARM_SPEED_CONFIRM_S=.05
ARM_SPEED_CONFIRM_FRAMES=2

def snapshot():
    return {"gripper_control_pause_ms":GRIP_CONTROL_PAUSE_S*1000,
            "gripper_soft_speed_rad_s":GRIP_SOFT_SPEED_RAD_S,
            "arm_soft_speed_rad_s":ARM_SOFT_SPEED_RAD_S,
            "arm_immediate_speed_rad_s":ARM_IMMEDIATE_SPEED_RAD_S,
            "arm_speed_confirm_ms":ARM_SPEED_CONFIRM_S*1000,
            "arm_speed_confirm_frames":ARM_SPEED_CONFIRM_FRAMES}

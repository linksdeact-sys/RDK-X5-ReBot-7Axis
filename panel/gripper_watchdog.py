"""J7 startup-only RAM configuration. Never enable a drive or save to Flash."""
from .gripper_teleop import firmware_timeout_ms

TIMEOUT_REGISTER = 9
TARGET_RAW = 4000  # 50 microseconds/unit -> 200ms.


def ensure_gripper_watchdog(motor, require_idle):
    """Keep a valid existing setting; repair an out-of-range value while idle.

    require_idle must verify all seven drives have fresh, disabled, stationary
    feedback. Reads failing or malformed values never authorize a blind write.
    A successful send is not sufficient: a separate read must match exactly.
    """
    require_idle()
    before = motor.damiao_get_param_u32(TIMEOUT_REGISTER, 100)
    if type(before) is not int or not 0 <= before <= 0xFFFFFFFF:
        raise RuntimeError("J7 断联超时参数读取无效，未修改参数、未使能电机")
    try:
        milliseconds = firmware_timeout_ms(before)
    except RuntimeError:
        require_idle()
        motor.damiao_write_param_u32(TIMEOUT_REGISTER, TARGET_RAW)
        require_idle()
        after = motor.damiao_get_param_u32(TIMEOUT_REGISTER, 100)
        if type(after) is not int or after != TARGET_RAW:
            raise RuntimeError(f"J7 断联超时自动配置未确认：期望原始值 {TARGET_RAW}，读回 {after!r}；未使能")
        require_idle()
        return {"before_raw": before, "after_raw": after,
                "timeout_ms": firmware_timeout_ms(after), "configured_ram": True}
    require_idle()
    return {"before_raw": before, "after_raw": before,
            "timeout_ms": milliseconds, "configured_ram": False}

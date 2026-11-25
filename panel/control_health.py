"""Control-completion policy, separate from CAN freshness and motor faults."""
import math

SOFT_PAUSE_MS = 100.0
HARD_TIMEOUT_MS = 250.0

def watchdog_action(age_ms):
    if not math.isfinite(age_ms) or age_ms < 0 or age_ms > HARD_TIMEOUT_MS:
        return "FAULT"
    if age_ms > SOFT_PAUSE_MS:
        return "PAUSE"
    return "OK"

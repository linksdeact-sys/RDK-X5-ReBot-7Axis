"""Shared freshness rules for CAN health, UI feedback and motion preflight."""

import math

MAX_RX_AGE_MS = 150
MOTOR_FIELDS = ("pos", "vel", "torq", "status_code", "t_mos", "t_rotor")


def times_from_rx(metadata, now):
    return [now - item["age_ms"] / 1000
            if item["sequence"] > 0 and 0 <= item["age_ms"] <= MAX_RX_AGE_MS else None
            for item in metadata]


def snapshot_motor(state):
    if state is None:
        return None
    snapshot = {name: getattr(state, name) for name in MOTOR_FIELDS}
    return snapshot if all(math.isfinite(value) for value in snapshot.values()) else None


def is_fresh(state, feedback_at, now):
    return (state is not None and feedback_at is not None
            and 0 <= now - feedback_at <= MAX_RX_AGE_MS / 1000)

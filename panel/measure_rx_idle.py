"""Read-only HTTP measurement. Abort if any axis becomes enabled or a job starts."""
import argparse
import json
import math
from pathlib import Path
import time
from urllib.request import ProxyHandler, build_opener


def percentile(values, fraction):
    return sorted(values)[round((len(values)-1)*fraction)]


def measure(url, seconds):
    ages = [[] for _ in range(7)]
    opener = build_opener(ProxyHandler({}))  # Local hardware must not traverse an HTTP proxy.
    start = time.monotonic()
    samples = 0
    while time.monotonic()-start < seconds:
        with opener.open(url.rstrip("/")+"/api/state", timeout=2.) as response:
            state = json.load(response)
        if state.get("active") or not state.get("connection", {}).get("ready"):
            raise RuntimeError("Measurement requires an idle connected panel")
        joints = state.get("joints", [])
        if len(joints) != 7 or any(j.get("status") != 0 for j in joints):
            raise RuntimeError("Measurement requires all seven axes disabled")
        for i, joint in enumerate(joints):
            age = joint.get("feedback_age_ms")
            if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
                raise RuntimeError(f"Invalid J{i+1} feedback age")
            ages[i].append(age)
        samples += 1
        time.sleep(.01)
    if samples < 20:
        raise RuntimeError(f"Insufficient measurement coverage: {samples} samples")
    return {"seconds": time.monotonic()-start, "samples": samples,
            "measurement": "API feedback age, including idle monitor scheduling; not wire latency",
            "all_disabled": True, "joints": [
                {"joint": i+1, "min_ms": min(a), "p50_ms": percentile(a, .50),
                 "p95_ms": percentile(a, .95), "max_ms": max(a),
                 "over_150ms": sum(v > 150. for v in a)} for i, a in enumerate(ages)]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://192.168.127.10:8765")
    parser.add_argument("--seconds", type=float, default=8.)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 2. <= args.seconds <= 30.:
        parser.error("--seconds must be between 2 and 30")
    result = measure(args.url, args.seconds)
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text+"\n", encoding="utf-8")
    print(text)

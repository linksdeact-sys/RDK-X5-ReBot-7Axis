"""Run continuous official gravity compensation; Ctrl+C requests smooth disable."""

import argparse
import json
import sys
import time
import urllib.request


sys.stdout.reconfigure(encoding="utf-8")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--check", action="store_true", help="Read status without enabling motors")
parser.add_argument("--base-url", default="http://192.168.127.10:8765")
parser.add_argument("--profile", choices=["official", "measured"] +
                    [f"j4_{p}" for p in range(30, 100, 5) if p != 50], default="j4_75")
args = parser.parse_args()
base = args.base_url.rstrip("/")
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def status():
    return json.load(opener.open(base + "/api/state", timeout=3))


def post(path, body):
    token = status()["csrf_token"]
    request = urllib.request.Request(
        base + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json", "Origin": base, "X-Arm-Token": token},
    )
    return json.load(opener.open(request, timeout=3))


initial = status()
print(json.dumps({key: initial[key] for key in ("active", "joints", "error")}, ensure_ascii=False), flush=True)
if args.check:
    raise SystemExit(0)
if initial["active"]:
    raise RuntimeError("Another job is already active")
job_id = f"gravity-continuous-{time.time_ns()}"
submitted = False
finished = False
try:
    input(f"扶稳机械臂，使用 {args.profile} 倍率持续运行，按 Enter 启动。Ctrl+C 停止并失能。")
    submitted = True
    post("/api/test", {"joint": 3, "degrees": 1, "kind": "gravity", "owner": job_id,
                       "request_id": job_id, "gravity_profile": args.profile})
    while True:
        state = status()
        if state["result"] and state["result"].get("id") == job_id and state["active"] is None:
            print("RESULT " + json.dumps(state["result"], ensure_ascii=False), flush=True)
            finished = True
            break
        active = state["active"] or {}
        print(json.dumps({"phase": active.get("phase"), "elapsed": active.get("elapsed"),
                          "command": active.get("torques")}, ensure_ascii=False), flush=True)
        time.sleep(1)
except KeyboardInterrupt:
    print("正在请求平滑停止，请继续扶稳机械臂。", flush=True)
finally:
    if submitted and not finished:
        try:
            post("/api/stop", {})
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                state = status()
                if state["active"] is None and all(joint["status"] == 0 for joint in state["joints"]):
                    print("七轴已失能。", flush=True)
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError("Disable confirmation timed out")
        except Exception as exc:
            print(f"停止未确认，请现场确认电机状态并准备断电。{exc}", file=sys.stderr, flush=True)
            raise

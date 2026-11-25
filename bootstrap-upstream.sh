#!/usr/bin/env bash
# Fetch source and model assets; never connect USB or start a controller.
set -euo pipefail
cd -- "$(dirname -- "$0")"
pin=3524ebc5316985d6d74fe30269ae771fea715507
upstream=.upstream-cache/rebot
mkdir -p .upstream-cache
if [ ! -e "$upstream" ]; then
    git clone --filter=blob:none --no-checkout https://github.com/Seeed-Projects/reBotArm_control_py.git "$upstream"
fi
if [ -e reBotArm_control_py ] || [ -e urdf ]; then
    echo "Runtime directories already exist; bootstrap is only for a fresh source checkout." >&2
    exit 1
fi
git -C "$upstream" checkout --detach "$pin"
test "$(git -C "$upstream" rev-parse HEAD)" = "$pin"
test "$(git hash-object "$upstream/reBotArm_control_py/actuator/rebotarm.py")" = 3c6be3edda9be9d7e728874c236da0c9f85b84c4
cp -a "$upstream/reBotArm_control_py" "$upstream/urdf" .
for f in rebotarm.yaml rebotarm_rs.yaml; do
    if [ ! -e "config/$f" ]; then cp "$upstream/config/$f" "config/$f"; fi
done
git apply --check rebotarm-runtime.patch
git apply rebotarm-runtime.patch
echo "Upstream source and URDF assets prepared. No hardware opened."

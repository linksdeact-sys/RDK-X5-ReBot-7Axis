#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$project_dir"
export LD_LIBRARY_PATH="/home/sunrise/rebot-usb-runtime/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -f "$project_dir/native/librebot_dm_observer.so" ]; then
    export REBOT_DM_REAL_LIB="/home/sunrise/.cache/motorbridge/dm_device/v1.1.0/linux/arm64/libdm_device.so"
    if [ -f "$project_dir/native/libdm_device_rx1ms.so" ]; then
        # A separately prepared, checksum-locked compatibility copy. Never patch
        # the installed vendor SDK or modify a library in a running process.
        "$project_dir/.venv/bin/python" "$project_dir/native/prepare_dm_rx.py" \
            --verify "$project_dir/native/libdm_device_rx1ms.so"
        export REBOT_DM_REAL_LIB="$project_dir/native/libdm_device_rx1ms.so"
    fi
    export MOTOR_DM_DEVICE_LIB="$project_dir/native/librebot_dm_observer.so"
fi

if [ "$#" -eq 0 ]; then
    set -- read_motor_feedback.py
fi

exec "$project_dir/.venv/bin/python" "$@"

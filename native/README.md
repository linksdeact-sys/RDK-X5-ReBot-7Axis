# Real feedback observation for local-mit-v2

The installed MotorBridge 0.5.0 Python state API exposes cached values without RX counters or timestamps. Runtime synchronous register reads were interfering with control timing. This Linux-only forwarding library records actual receive-callback times and sequences while continuing to call the original SDK callback.

It opens no additional USB handle, modifies no firmware/EEPROM, and leaves the installed MotorBridge library and vendor SDK unchanged. `run_rebot.sh` selects it for this project via `MOTOR_DM_DEVICE_LIB`, and points `REBOT_DM_REAL_LIB` at the existing SDK. Missing observation support prevents the new hardware panel from starting.

Forwarded API signatures use the upstream SDK header from [MotorBridge v0.5.0](https://github.com/motorbridge/motorbridge/tree/082cbb8c36dfdc8fbf9baa4cc02c5992ab0dc2ad), `third_party/dm_device/v1.1.0/dmcan.h`. On RDK that header is copied alongside these sources as `native/dmcan.h`. The local reference checkout is ignored by Git and is not a system dependency.

Build from the RDK project directory:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -fPIC -shared -pthread native/dm_observer.cpp -ldl -o native/librebot_dm_observer.so
g++ -std=c++17 -Wall -Wextra -Werror -pthread native/test_dm_observer.cpp -ldl -o native/test_dm_observer
native/test_dm_observer
```

The offline test verifies callback forwarding and rejection of wrong channel, DLC or motor ID. It opens no USB device. Real disabled-only verification has also confirmed counters advance for all seven motor feedback IDs 0x11–0x17. This verifies receipt of data, not stability of physical motion.

The fixed hardware configuration is one USB2CANFD channel0 with motor IDs1–7. The observer deliberately counts only valid eight-byte standard feedback on this channel, with matching low-nibble motor ID. Reconfiguration of adapter/channel/IDs requires updating and retesting this filter.

Do not rebuild the .so while the hardware panel is running. Stop control, support the arm, confirm seven disabled motors and terminate the panel before replacing it. A rollback of the observer alone is insufficient: new software requires timestamps and will remain blocked without them. The previous pure-torque hand-control implementation is known unstable and must not be used as a fallback.

## Project-local DM 1.1.0 receive timing compatibility fix

The official AArch64 1.1.0 binary (SHA256
`4a73c8ec7d64342f0d0a1156d5102f519c5f176fb23696740f254fe6492c3f2a`)
has a hard-coded 100ms idle sleep in `can_bus_box::notification_thread_func()`.
Frames queued during that sleep cannot wake it. Disabled-only measurements showed
roughly 100ms callback batches; this is delivery latency, not proof of CAN wire
delay. The browser's 200ms refresh hides these gaps.

`prepare_dm_rx.py` creates a separate, **locally modified, not vendor-released**
copy. It changes only the idle timespec at ELF file offset `0x123b0` from
`{0,100000000ns}` to `{0,1000000ns}`. Disassembly of the exact reviewed binary
shows this constant loaded at `0xd1c8` only by the notification idle branch.
USB event waiting, heartbeat, CAN packets, callbacks, motor gains, speed limits,
the 150ms feedback fault rule, and firmware are unchanged. Full input/output
SHA256 checks reject any other SDK or partial output; no generic search/replace,
instruction patch, global sleep interception or in-process memory modification.

Prepare while the panel is stopped, after seven fresh disabled replies:

```sh
.venv/bin/python native/prepare_dm_rx.py \
  --source /home/sunrise/.cache/motorbridge/dm_device/v1.1.0/linux/arm64/libdm_device.so \
  --output native/libdm_device_rx1ms.so
.venv/bin/python -m unittest native.test_prepare_dm_rx
```

`run_rebot.sh` verifies the derived checksum on every start, then selects this
copy for this project only. A mismatch aborts startup rather than silently
loading it. The installed vendor library is untouched. Existing different
output is never overwritten. An SDK update requires a new review, not bypassing
the checks. To roll back, first stop and confirm disabled, terminate the panel,
then move the derived copy aside and restart; the original high-latency SDK is
**not** a verified usable seven-axis teleop fallback. Do not auto-start motion.

Offline artifact tests and disabled-only latency measurements do not establish
physical motion safety. Combined arm/gripper acceptance still needs a supported,
user-operated test with the existing watchdog and stop path.

2026-11-21 verification: 80 panel unit tests passed; five artifact tests passed
locally against the official reference binary (four passed on RDK, reference-only
test skipped there). Fake-CAN integration confirmed J1-J6 joystick references
and J7 LT velocity commands active in the **same session**, plus RT/LT reversal,
local J7 soft braking, hard faults and shutdown. The real browser check blocks
all POST requests and confirmed one J1-J7 Start entry, seven online/disabled
axes, and no JavaScript errors; it never clicked Start.

Disabled-only HTTP measurements, same idle monitor, no movement:

| SDK | Samples / duration | Worst age across seven axes | Over 150ms |
| --- | --- | --- | --- |
| Original 100ms idle | 277 / 5s | 143.9ms | 0 |
| Local 1ms idle | 438 / 8s | 62.0ms | 0 |
| Local 1ms follow-up | 847 / 15s | 61.6ms | 0 |

These ages include the panel's approximately 50ms idle refresh scheduling; they
are neither measured CAN wire latency nor a claim of completed physical teleop
acceptance. Evidence is `native/rx-baseline-disabled.json`,
`native/rx-after-disabled.json` and `native/rx-soak-disabled.json`. RDK rollback
backup: `/home/sunrise/rebot-rx-release.JJZLYDTt`. The idle service loads the
separate reviewed library, retains the original SDK checksum, and does not
auto-start motion after restart/reconnect.

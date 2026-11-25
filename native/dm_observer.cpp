// Transparent DM SDK forwarding shim. Count only real channel-0 motor replies.
// Original receive callback is always forwarded; no second USB handle is opened.
#include "dmcan.h"
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <dlfcn.h>
#include <mutex>
#include <unordered_map>

namespace {
struct Received {
    uint64_t count = 0;
    std::chrono::steady_clock::time_point at{};
};
std::mutex rx_mutex;
std::array<Received, 7> received;
std::unordered_map<dmcan_device_handle*, dev_recv_callback> callbacks;

void* real_library() {
    static void* lib = [] {
        const char* path = std::getenv("REBOT_DM_REAL_LIB");
        return path ? dlopen(path, RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND) : nullptr;
    }();
    return lib;
}
template<class T> T symbol(const char* name) {
    void* lib = real_library();
    return lib ? reinterpret_cast<T>(dlsym(lib, name)) : nullptr;
}
void observer(dmcan_device_handle* dev, usb_rx_frame_t* frame) {
    dev_recv_callback next = nullptr;
    {
        std::lock_guard<std::mutex> lock(rx_mutex);
        const auto found = callbacks.find(dev);
        if (found != callbacks.end()) next = found->second;
        if (frame && !frame->head.ext && !frame->head.rtr && frame->head.channel == 0 &&
            frame->head.dlc == 8 && frame->head.can_id >= 0x11 && frame->head.can_id <= 0x17 &&
            (frame->payload[0] & 15) == frame->head.can_id - 0x10) {
            auto& state = received[frame->head.can_id-0x11];
            ++state.count;
            state.at = std::chrono::steady_clock::now();
        }
    }
    if (next) next(dev, frame);
}
}

extern "C" int rebot_rx_meta(uint32_t can_id, double* age_s, uint64_t* sequence) {
    if (can_id < 0x11 || can_id > 0x17 || !age_s || !sequence) return -1;
    std::lock_guard<std::mutex> lock(rx_mutex);
    const auto& state = received[can_id-0x11];
    *sequence = state.count;
    *age_s = state.count ? std::chrono::duration<double>(std::chrono::steady_clock::now()-state.at).count() : 1e9;
    return 0;
}
extern "C" void dmcan_context_create(dmcan_context** ctx) {
    auto fn = symbol<decltype(&dmcan_context_create)>("dmcan_context_create");
    if (ctx) *ctx = nullptr;
    if (fn) fn(ctx);
}
extern "C" int dmcan_find_devices_with_type(dmcan_context* ctx, dmcan_device_type_t type) {
    auto fn = symbol<decltype(&dmcan_find_devices_with_type)>("dmcan_find_devices_with_type");
    return fn ? fn(ctx, type) : 0;
}
extern "C" bool dmcan_device_get(dmcan_context* ctx, dmcan_device_handle** dev, int index) {
    auto fn = symbol<decltype(&dmcan_device_get)>("dmcan_device_get");
    return fn && fn(ctx, dev, index);
}
extern "C" bool dmcan_device_open(dmcan_device_handle* dev) {
    auto fn = symbol<decltype(&dmcan_device_open)>("dmcan_device_open");
    return fn && fn(dev);
}
extern "C" bool dmcan_device_enable_channel(dmcan_device_handle* dev, uint8_t channel) {
    auto fn = symbol<decltype(&dmcan_device_enable_channel)>("dmcan_device_enable_channel");
    return fn && fn(dev, channel);
}
extern "C" bool dmcan_device_get_channel_baudrate(dmcan_device_handle* dev, uint8_t channel, dmcan_channel_can_info_t* info) {
    auto fn = symbol<decltype(&dmcan_device_get_channel_baudrate)>("dmcan_device_get_channel_baudrate");
    return fn && fn(dev, channel, info);
}
extern "C" bool dmcan_device_set_channel_baudrate(dmcan_device_handle* dev, uint8_t channel, dmcan_channel_can_info_t info) {
    auto fn = symbol<decltype(&dmcan_device_set_channel_baudrate)>("dmcan_device_set_channel_baudrate");
    return fn && fn(dev, channel, info);
}
extern "C" void dmcan_device_hook_recv_callback(dmcan_device_handle* dev, dev_recv_callback callback) {
    {
        std::lock_guard<std::mutex> lock(rx_mutex);
        callbacks[dev] = callback;
        received = {};
    }
    auto fn = symbol<decltype(&dmcan_device_hook_recv_callback)>("dmcan_device_hook_recv_callback");
    if (fn) fn(dev, callback ? observer : nullptr);
}
extern "C" bool dmcan_device_send_can(dmcan_device_handle* dev, uint8_t ch, uint32_t id,
    bool canfd, bool ext, bool rtr, bool brs, uint8_t len, uint8_t* data) {
    auto fn = symbol<decltype(&dmcan_device_send_can)>("dmcan_device_send_can");
    return fn && fn(dev, ch, id, canfd, ext, rtr, brs, len, data);
}
extern "C" int dmcan_utils_get_dlc_from_len(int len) {
    auto fn = symbol<decltype(&dmcan_utils_get_dlc_from_len)>("dmcan_utils_get_dlc_from_len");
    return fn ? fn(len) : -1;
}

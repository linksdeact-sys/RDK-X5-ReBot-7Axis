// Offline: no SDK opened, no CAN commands sent.
#include "dm_observer.cpp"
#include <cassert>
#include <thread>
#include <iostream>

int main() {
    auto* dev = reinterpret_cast<dmcan_device_handle*>(1);
    int forwarded = 0;
    static int* count;
    count = &forwarded;
    callbacks[dev] = [](dmcan_device_handle*, usb_rx_frame_t*) { ++*count; };
    double age = 0;
    uint64_t seq = 0;
    assert(rebot_rx_meta(0x11, &age, &seq) == 0 && seq == 0 && age > 1000);
    usb_rx_frame_t frame{};
    frame.head.can_id = 0x11; frame.head.dlc = 8; frame.payload[0] = 0x11;
    observer(dev, &frame);
    assert(rebot_rx_meta(0x11, &age, &seq) == 0 && seq == 1 && age < 0.01);
    frame.head.channel = 1; observer(dev, &frame);
    frame.head.channel = 0; frame.payload[0] = 2; observer(dev, &frame);
    frame.payload[0] = 1; frame.head.dlc = 7; observer(dev, &frame);
    assert(rebot_rx_meta(0x11, &age, &seq) == 0 && seq == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    assert(rebot_rx_meta(0x11, &age, &seq) == 0 && age >= 0.02);
    assert(forwarded == 4);
    std::cout << "PASS RX timestamps, counters, channel/ID/DLC filters and callback forwarding; no USB opened\n";
}

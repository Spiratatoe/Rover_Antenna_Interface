
#ifndef ALL_LAYERS_APPLICATION_LAYER_H
#define ALL_LAYERS_APPLICATION_LAYER_H


#include <cstdint>
#include <cstdlib>
#include <cstdio>

enum class LinkStatus : uint8_t {
    LINK_DOWN = 0,
    LINK_UP = 1,
    LINK_DEGRADED = 2
};

struct LocalPacket {
    LinkStatus status;  // --- byte 0
    int8_t rssi;       // dBm (-128 to +127)   ---  byte 1
    int8_t ping;       // in 10ms steps (0-255) (max 2550ms ?? - probs meant 255) --- byte 2
    uint8_t bandwidth; // Mbps (0-255)  --- byte 3

    // fake data before we have real stuff
    void mockData(bool linkActive) {
        if (linkActive) {
            status = LinkStatus::LINK_UP;
            rssi = -50 + (rand() % 20); // Simulate RSSI (-50 dBm ± fluctuation)
            ping = rand() % 50;         // Simulate ping (0-500ms)
            bandwidth = 10 + (rand() % 100); // Simulate bandwidth (10-110 Mbps)
        } else {
            status = LinkStatus::LINK_DOWN;
            rssi = -128;
            ping = 255;
            bandwidth = 0;
        }
    }

    void sendLocalPacket(bool linkActive) {
        LocalPacket packet;
        packet.mockData(linkActive);

        // Simulate sending (replace with actual communication code)
        printf("Sending Local Packet - Status: %d, RSSI: %d dBm, Ping: %d ms, Bandwidth: %d Mbps\n",
               static_cast<int>(packet.status), packet.rssi, packet.ping * 10, packet.bandwidth);
    }



};

#endif //ALL_LAYERS_APPLICATION_LAYER_H

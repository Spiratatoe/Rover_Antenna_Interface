
#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

// Constants for packet structure
#define PACKET_MAX_PAYLOAD_SIZE 8
#define PACKET_HEADER_SIZE 1
#define PACKET_MAX_SIZE (PACKET_HEADER_SIZE + PACKET_MAX_PAYLOAD_SIZE)

// Bit masks for header byte
#define PACKET_TYPE_MASK    0x80    // Bit 7
#define PACKET_RESERVED_MASK 0x70   // Bits 4-6
#define PACKET_PID_MASK     0x0F    // Bits 0-3

// Packet types
#define PACKET_TYPE_LOCAL       1
#define PACKET_TYPE_BROADCAST   0

// Reserved PIDs
#define PID_BROADCAST_RESERVED  0

// Application PIDs
typedef enum {
    // System PIDs (0-3)
    PID_SYS_STATUS = 1,
    PID_SYS_ERROR = 2,
    PID_SYS_HEARTBEAT = 3,

    // Rover PIDs (4-7)
    PID_ROVER_POSITION = 4,
    PID_ROVER_SPEED = 5,
    PID_ROVER_BATTERY = 6,
    PID_ROVER_TILT = 7,

    // RAC PIDs (8-11)
    PID_RAC_GPS = 8,
    PID_RAC_COMMAND = 9,
    PID_RAC_STATUS = 10,

    // BAC PIDs (12-15)
    PID_BAC_TEMP = 12,
    PID_BAC_RADIO = 13,
    PID_BAC_STATUS = 14
} PacketID;

// Packet structure
typedef struct {
    uint8_t header;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
    uint8_t payload_length;
} Packet;

// Function declarations
bool packet_create(Packet* packet, bool is_local, PacketID pid,
                   const uint8_t* payload, uint8_t payload_length);
bool packet_is_local(const Packet* packet);
bool packet_is_broadcast(const Packet* packet);
PacketID packet_get_pid(const Packet* packet);
uint8_t packet_get_payload_length(const Packet* packet);
const uint8_t* packet_get_payload(const Packet* packet);





bool packet_create(Packet* packet, bool is_local, PacketID pid,
                   const uint8_t* payload, uint8_t payload_length) {
    // Validate input parameters
    if (!packet || payload_length > PACKET_MAX_PAYLOAD_SIZE ||
        (payload_length > 0 && !payload) ||
        (pid > (PACKET_PID_MASK & 0xFF))) {
        return false;
    }

    // Check for reserved broadcast PID
    if (!is_local && pid == PID_BROADCAST_RESERVED) {
        return false;
    }

    // Construct header byte
    packet->header = (is_local ? PACKET_TYPE_LOCAL : PACKET_TYPE_BROADCAST) << 7;
    packet->header |= (pid & PACKET_PID_MASK);

    // Copy payload if present
    if (payload_length > 0) {
        memcpy(packet->payload, payload, payload_length);
    }
    packet->payload_length = payload_length;

    return true;
}

bool packet_is_local(const Packet* packet) {
    if (!packet) return false;
    return (packet->header & PACKET_TYPE_MASK) == (PACKET_TYPE_LOCAL << 7);
}

bool packet_is_broadcast(const Packet* packet) {
    if (!packet) return false;
    return (packet->header & PACKET_TYPE_MASK) == (PACKET_TYPE_BROADCAST << 7);
}

PacketID packet_get_pid(const Packet* packet) {
    if (!packet) return static_cast<PacketID>(PID_BROADCAST_RESERVED);
    return static_cast<PacketID>(packet->header & PACKET_PID_MASK);
}

uint8_t packet_get_payload_length(const Packet* packet) {
    if (!packet) return 0;
    return packet->payload_length;
}

const uint8_t* packet_get_payload(const Packet* packet) {
    if (!packet) return NULL;
    return packet->payload;
}



void print_packet(const Packet* packet) {
    printf("Packet:\n");
    printf("  Type: %s\n", packet_is_local(packet) ? "Local" : "Broadcast");
    printf("  PID: %d\n", packet_get_pid(packet));
    printf("  Payload Length: %d\n", packet_get_payload_length(packet));
    printf("  Payload: ");
    const uint8_t* payload = packet_get_payload(packet);
    for (int i = 0; i < packet_get_payload_length(packet); i++) {
        printf("%02X ", payload[i]);
    }
    printf("\n");
}



#endif PACKET_PROTOCOL_H

//
// Created by jack spiratos on 2025-03-27.
//

#ifndef COMPUTER_LAYERS_NETWORK_LAYER_H
#define COMPUTER_LAYERS_NETWORK_LAYER_H

#include <vector>
#include <cstdint>

// ----------------------------------------------------------------------
// Network Layer Specification
//
// A network packet is structured as follows:
//
// [Header Byte] [Payload Bytes...]
//
// Header Byte format:
// - Bit 7: Local flag (1 for local packet, 0 for broadcast)
// - Bits 4-6: Reserved (set to 0 when sending)
// - Bits 0-3: Packet ID (PID) (0-15)
//
// The payload is a byte array (typically up to 8 bytes for our design).
// ----------------------------------------------------------------------

// Encodes a network packet given a PID, payload and local flag.
// Parameters:
//   pid      - The Packet ID (0-15) identifying the type of packet.
//   payload  - The payload data as a vector of bytes.
//   isLocal  - True if the packet is local (bit7 = 1); false for broadcast.
// Returns:
//   A vector of bytes representing the complete network packet.
inline std::vector<uint8_t> encodeNetworkPacket(uint8_t pid, const std::vector<uint8_t>& payload, bool isLocal = true) {
    std::vector<uint8_t> packet;
    // Create the header:
    // Bit 7: isLocal flag
    // Bits 4-6: Reserved (0)
    // Bits 0-3: Packet ID (PID)
    uint8_t header = (isLocal ? 0x80 : 0x00) | (pid & 0x0F);
    packet.push_back(header);
    // Append the payload bytes
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

// Decodes a network packet.
// Parameters:
//   packet   - The complete network packet as a vector of bytes.
//   pid      - Output parameter to hold the extracted Packet ID.
//   payload  - Output parameter to hold the payload bytes.
//   isLocal  - Output parameter to indicate if the packet is local.
// Returns:
//   True if decoding succeeded (i.e. packet is non-empty), false otherwise.
inline bool decodeNetworkPacket(const std::vector<uint8_t>& packet, uint8_t &pid, std::vector<uint8_t>& payload, bool &isLocal) {
    if (packet.empty()) {
        return false;
    }
    uint8_t header = packet[0];
    isLocal = (header & 0x80) != 0;
    pid = header & 0x0F;
    // The rest of the packet is the payload.
    payload.assign(packet.begin() + 1, packet.end());
    return true;
}


#endif //COMPUTER_LAYERS_NETWORK_LAYER_H

//
// Created by jack spiratos on 2025-03-27.
//

#ifndef COMPUTER_LAYERS_DATA_LINK_LAYER_H
#define COMPUTER_LAYERS_DATA_LINK_LAYER_H
#include <vector>
#include <cstdint>

// ----------------------------------------------------------------------
// Data Link Layer Specification for RAI PIF
//
// A frame is constructed as follows:
//
// [Leading Flag] [Length] [Data] [CRC8] [Closing Flag]
//
// - Leading Flag: Fixed byte 0x55 (start of frame)
// - Length: 1-byte field indicating the number of data bytes (the network packet size)
// - Data: The network packet bytes (already built in the Network Layer)
// - CRC8: 1-byte checksum computed over the Length and Data bytes using
//         polynomial 0x1D, initial value 0xFF, no final XOR.
// - Closing Flag: Fixed byte 0x55 (end of frame)
//
// Byte Stuffing (applied to Length, Data, and CRC fields):
// - Replace any occurrence of 0x55 with the two-byte sequence: 0xAA 0x05
// - Replace any occurrence of 0xAA with the two-byte sequence: 0xAA 0x0A
//
// The leading and closing flag bytes (0x55) are not stuffed.
// ----------------------------------------------------------------------

// Computes CRC8 over the given data vector using polynomial 0x1D,
// initial value 0xFF, and no final XOR.
inline uint8_t computeCRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0xFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x1D;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Performs byte stuffing on the input vector.
// Replaces any occurrence of 0x55 with 0xAA 0x05 and 0xAA with 0xAA 0x0A.
inline std::vector<uint8_t> byteStuff(const std::vector<uint8_t>& input) {
    std::vector<uint8_t> output;
    for (uint8_t byte : input) {
        if (byte == 0x55) {
            output.push_back(0xAA);
            output.push_back(0x05);
        } else if (byte == 0xAA) {
            output.push_back(0xAA);
            output.push_back(0x0A);
        } else {
            output.push_back(byte);
        }
    }
    return output;
}

// ----------------------------------------------------------------------
// New: Byte Unstuffing function (inverse of byteStuff)
// Scans the input vector and reverses the escape sequences.
// ----------------------------------------------------------------------
inline std::vector<uint8_t> byteUnstuff(const std::vector<uint8_t>& input) {
    std::vector<uint8_t> output;
    for (size_t i = 0; i < input.size(); ) {
        if (input[i] == 0xAA) {
            if (i + 1 >= input.size()) {
                // Incomplete escape sequence: break or handle error as needed.
                break;
            }
            uint8_t nextByte = input[i + 1];
            if (nextByte == 0x05) {
                output.push_back(0x55);
            } else if (nextByte == 0x0A) {
                output.push_back(0xAA);
            } else {
                // Unexpected escape sequence: output the escape byte and continue.
                output.push_back(input[i]);
                i++;
                continue;
            }
            i += 2;
        } else {
            output.push_back(input[i]);
            i++;
        }
    }
    return output;
}

// ----------------------------------------------------------------------
// Frames a network packet into a complete data link layer frame.
// The networkPacket parameter is a vector of bytes produced by the Network Layer.
// The resulting frame has the structure:
// [0x55] [Length] [Byte-stuffed (Length + networkPacket + CRC)] [0x55]
// ----------------------------------------------------------------------
inline std::vector<uint8_t> framePacket(const std::vector<uint8_t>& networkPacket) {
    std::vector<uint8_t> frame;

    // Append Leading Flag (0x55)
    frame.push_back(0x55);

    // Length: size of the network packet.
    uint8_t length = static_cast<uint8_t>(networkPacket.size());

    // Build the raw field that will be stuffed:
    // Start with the Length byte, then the network packet data.
    std::vector<uint8_t> rawField;
    rawField.push_back(length);
    rawField.insert(rawField.end(), networkPacket.begin(), networkPacket.end());

    // Compute the CRC8 over the raw field (Length + Data)
    uint8_t crc = computeCRC8(rawField);

    // Append the CRC to the raw field.
    rawField.push_back(crc);

    // Apply byte stuffing to the raw field.
    std::vector<uint8_t> stuffedField = byteStuff(rawField);

    // Append the stuffed field to the frame.
    frame.insert(frame.end(), stuffedField.begin(), stuffedField.end());

    // Append the Closing Flag (0x55)
    frame.push_back(0x55);

    return frame;
}

// ----------------------------------------------------------------------
// Deserializes (unframes) a data link layer frame back into the original
// network packet. It verifies the leading and closing flags, performs
// byte unstuffing, checks the length and CRC, and outputs the network packet.
// Returns true if successful, false otherwise.
// ----------------------------------------------------------------------
inline bool unframePacket(const std::vector<uint8_t>& frame, std::vector<uint8_t>& networkPacket) {
    // Check that frame is long enough: must include at least 4 bytes:
    // Leading flag, Length, CRC, and Closing flag.
    if (frame.size() < 4) return false;
    // Check leading and closing flags.
    if (frame.front() != 0x55 || frame.back() != 0x55) return false;
    // Extract the stuffed field (everything between the flags).
    std::vector<uint8_t> stuffedField(frame.begin() + 1, frame.end() - 1);
    // Perform byte unstuffing.
    std::vector<uint8_t> rawField = byteUnstuff(stuffedField);
    // rawField must have at least 2 bytes: one for length and one for CRC.
    if (rawField.size() < 2) return false;
    // The first byte is the length.
    uint8_t length = rawField[0];
    // The raw field should be exactly length + 2 bytes (length byte, data bytes, and CRC).
    if (rawField.size() != static_cast<size_t>(length + 2)) return false;
    // Separate out the CRC: it's the last byte.
    uint8_t receivedCrc = rawField.back();
    // Compute the CRC over the raw field excluding the CRC byte.
    std::vector<uint8_t> crcData(rawField.begin(), rawField.end() - 1);
    uint8_t computedCrc = computeCRC8(crcData);
    if (computedCrc != receivedCrc) return false;
    // Extract the network packet: bytes following the length byte.
    networkPacket.assign(rawField.begin() + 1, rawField.begin() + 1 + length);
    return true;
}




#endif //COMPUTER_LAYERS_DATA_LINK_LAYER_H

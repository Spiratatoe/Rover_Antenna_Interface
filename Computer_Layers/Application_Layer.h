//
// Created by jack spiratos on 2025-03-27.
//
//

#ifndef COMPUTER_LAYERS_APPLICATION_LAYER_H
#define COMPUTER_LAYERS_APPLICATION_LAYER_H

#include <vector>
#include <cstdint>
#include <cmath>

// Sensor data structures
struct LIS3DHData { // sensor name that the accel goes by
    float ax;         // Acceleration in g's (X)
    float ay;         // Acceleration in g's (Y)
    float az;         // Acceleration in g's (Z)
    float temperature; // Temperature in °C -- probs don't need this, but might as well
};

struct LIS2MDLData { // sensor name that the magnotometer goes by
    float magX;       // Magnetic field in G (X)
    float magY;       // Magnetic field in G (Y)
    float magZ;       // Magnetic field in G (Z)
    float temperature; // Temperature in °C -- probs don't need this, but might as well
};

// when gps ready -- add it's struct here ...

// Scaling factors for converting float sensor values to integer representation
constexpr float ACCEL_SCALE = 1000.0f;  // For acceleration (e.g., 0.012g becomes 12)
constexpr float TEMP_SCALE   = 1.0f;      // For temperature (1:1 conversion)
constexpr float MAG_SCALE    = 1000.0f;   // For magnetometer data (if needed)

// The application layer is responsible for taking sensor data and encoding it into a payload.
// For this example, we pack only the LIS3DH sensor data into an 8-byte payload as follows:
//  - 16 bits for X acceleration (scaled)
//  - 16 bits for Y acceleration (scaled)
//  - 16 bits for Z acceleration (scaled)
//  - 16 bits for temperature (no scaling or as desired)
// All values are stored in little-endian order so if 0x0034 we pack 0x34 then 0x00
inline std::vector<uint8_t> encodeApplicationData(const LIS3DHData &lis3dh, const LIS2MDLData & /*unused*/) {
    std::vector<uint8_t> payload;

    // Convert and scale the LIS3DH values to 16-bit integers.
    int16_t ax_int = static_cast<int16_t>(std::round(lis3dh.ax * ACCEL_SCALE));
    int16_t ay_int = static_cast<int16_t>(std::round(lis3dh.ay * ACCEL_SCALE));
    int16_t az_int = static_cast<int16_t>(std::round(lis3dh.az * ACCEL_SCALE));
    int16_t temp_int = static_cast<int16_t>(std::round(lis3dh.temperature * TEMP_SCALE));

    // Pack each 16-bit value into the payload in little-endian order.
    payload.push_back(static_cast<uint8_t>(ax_int & 0xFF));
    payload.push_back(static_cast<uint8_t>((ax_int >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>(ay_int & 0xFF));
    payload.push_back(static_cast<uint8_t>((ay_int >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>(az_int & 0xFF));
    payload.push_back(static_cast<uint8_t>((az_int >> 8) & 0xFF));
    payload.push_back(static_cast<uint8_t>(temp_int & 0xFF));
    payload.push_back(static_cast<uint8_t>((temp_int >> 8) & 0xFF));

    return payload;
}


#endif //COMPUTER_LAYERS_APPLICATION_LAYER_H

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <string>
#include <regex>
#include <cstdlib>
#include <vector>
#include <cstdio>

#include "Data_Link_Layer.h"   // provides framePacket()
#include "Network_Layer.h"     // provides encodeNetworkPacket()
#include "Application_Layer.h" // provides encodeApplicationData() and defines LIS3DHData, LIS2MDLData

int main() {
    // Adjust this to match your serial device (e.g., "/dev/tty.usbmodem21401")
    const char* portName = "/dev/tty.usbmodem21401";

    // Open the serial port.
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error opening serial port " << portName
                  << ": " << strerror(errno) << std::endl;
        return 1;
    }
    // Clear non-blocking flag.
    fcntl(fd, F_SETFL, 0);

    // Configure the serial port.
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);  // enable receiver, local mode
    options.c_cflag &= ~PARENB;           // no parity
    options.c_cflag &= ~CSTOPB;           // one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;               // 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input mode
    options.c_oflag &= ~OPOST;            // raw output mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable flow control
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;             // timeout in deciseconds
    tcsetattr(fd, TCSANOW, &options);

    std::cout << "Serial port " << portName << " opened. Reading data..." << std::endl;

    // Buffer to accumulate data from the serial port.
    const int bufferSize = 256;
    char buffer[bufferSize];
    std::string lineBuffer;

    // Setup regex patterns to parse the sensor output.
    std::regex accelRegex(R"(Acceleration:\s*X:\s*([-+]?[0-9]*\.?[0-9]+)g,\s*Y:\s*([-+]?[0-9]*\.?[0-9]+)g,\s*Z:\s*([-+]?[0-9]*\.?[0-9]+)g)");
    std::regex lis3dhTempRegex(R"(LIS3DH Temperature:\s*([-+]?[0-9]*\.?[0-9]+))");
    std::regex magRegex(R"(Magnetic Field:\s*X:\s*([-+]?[0-9]*\.?[0-9]+)\s*G,\s*Y:\s*([-+]?[0-9]*\.?[0-9]+)\s*G,\s*Z:\s*([-+]?[0-9]*\.?[0-9]+)\s*G)");
    std::regex lis2mdlTempRegex(R"(LIS2MDL Temperature:\s*([-+]?[0-9]*\.?[0-9]+))");

    // Data containers for sensor readings.
    // (These are defined in Application_Layer.h as well.)
    LIS3DHData lis3dh = {0, 0, 0, 0};
    LIS2MDLData lis2mdl = {0, 0, 0, 0};

    // Main loop: read from the chip and process complete lines.
    while (true) {
        int n = read(fd, buffer, bufferSize - 1);
        if (n < 0) {
            std::cerr << "Error reading: " << strerror(errno) << std::endl;
            break;
        } else if (n > 0) {
            buffer[n] = '\0';
            lineBuffer.append(buffer);
            // Check for newline to determine complete lines.
            size_t pos = 0;
            while ((pos = lineBuffer.find('\n')) != std::string::npos) {
                std::string completeLine = lineBuffer.substr(0, pos);
                if (!completeLine.empty() && completeLine.back() == '\r') {
                    completeLine.pop_back();
                }
                std::cout << "Received: " << completeLine << std::endl;
                std::smatch match;
                if (std::regex_search(completeLine, match, accelRegex)) {
                    lis3dh.ax = std::stof(match[1].str());
                    lis3dh.ay = std::stof(match[2].str());
                    lis3dh.az = std::stof(match[3].str());
                    std::cout << "Parsed Acceleration: X=" << lis3dh.ax << "g, "
                              << "Y=" << lis3dh.ay << "g, "
                              << "Z=" << lis3dh.az << "g" << std::endl;
                } else if (std::regex_search(completeLine, match, lis3dhTempRegex)) {
                    lis3dh.temperature = std::stof(match[1].str());
                    std::cout << "Parsed LIS3DH Temperature: "
                              << lis3dh.temperature << "°C" << std::endl;
                } else if (std::regex_search(completeLine, match, magRegex)) {
                    lis2mdl.magX = std::stof(match[1].str());
                    lis2mdl.magY = std::stof(match[2].str());
                    lis2mdl.magZ = std::stof(match[3].str());
                    std::cout << "Parsed Magnetic Field: X=" << lis2mdl.magX << " G, "
                              << "Y=" << lis2mdl.magY << " G, "
                              << "Z=" << lis2mdl.magZ << " G" << std::endl;
                } else if (std::regex_search(completeLine, match, lis2mdlTempRegex)) {
                    lis2mdl.temperature = std::stof(match[1].str());
                    std::cout << "Parsed LIS2MDL Temperature: "
                              << lis2mdl.temperature << "°C" << std::endl;
                } else {
                    std::cout << "Ignored: " << completeLine << std::endl;
                }
                // Remove the processed line.
                lineBuffer.erase(0, pos + 1);

                // For demonstration, once we have a complete set of readings,
                // build and print the final packet.
                // Here, we assume a complete reading is indicated by nonzero acceleration or temperature.
                if ( (lis3dh.ax != 0 || lis3dh.ay != 0 || lis3dh.az != 0 || lis3dh.temperature != 0) ) {
                    // --- Application Layer: Encode sensor data into payload.
                    std::vector<uint8_t> appPayload = encodeApplicationData(lis3dh, lis2mdl);

                    // --- Network Layer: Prepend a header.
                    // For example, we use PID = 1 for sensor data and set the local flag.
                    std::vector<uint8_t> networkPacket = encodeNetworkPacket(1, appPayload, true);

                    // --- Data Link Layer: Frame the network packet.
                    std::vector<uint8_t> framedPacket = framePacket(networkPacket);

                    // Print the final framed packet in hexadecimal.
                    std::cout << "Final Framed Packet: ";
                    for (uint8_t byte : framedPacket) {
                        printf("%02X ", byte);
                    }
                    std::cout << std::endl;
                }
            }
        }
    }

    close(fd);


//start tests here

//    // Create an example network packet with the data bytes:
//    // [0x55, 0x66, 0x77, 0x88, 0x99, 0xAA]
//    std::vector<uint8_t> networkPacket = {0x55, 0x66, 0x77, 0x88, 0x99, 0xAA};
//
//    // Use the data link layer to frame the network packet.
//    std::vector<uint8_t> framedPacket = framePacket(networkPacket);
//
//    // Print the framed packet in hexadecimal format.
//    std::cout << "Framed Packet: ";
//    for (uint8_t byte : framedPacket) {
//        printf("%02X ", byte);
//    }
//    std::cout << std::endl;
//
//    // Expected output (in hex):
//    // 55 06 AA 05 66 77 88 99 AA 0A C5 55
//
//    // Test the unframing function: convert the framed packet back into the original network packet.
//    std::vector<uint8_t> unframedPacket;
//    bool success = unframePacket(framedPacket, unframedPacket);
//    if (success) {
//        std::cout << "Unframed Packet: ";
//        for (uint8_t byte : unframedPacket) {
//            printf("%02X ", byte);
//        }
//        std::cout << std::endl;
//    } else {
//        std::cout << "Unframing failed!" << std::endl;
//    }

    return 0;
}
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

// For UDP sockets on macOS/Linux
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

// Include your layered protocol headers
#include "Data_Link_Layer.h"   // framePacket(), unframePacket()
#include "Network_Layer.h"     // encodeNetworkPacket(), decodeNetworkPacket()
#include "Application_Layer.h" // encodeApplicationData(), LIS3DHData, LIS2MDLData

int main() {
    // 1. OPEN & CONFIGURE SERIAL PORT
    const char* portName = "/dev/tty.usbmodem21401";  // <-- Adjust to your actual device
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error opening serial port " << portName
                  << ": " << strerror(errno) << std::endl;
        return 1;
    }
    // Make read() blocking
    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 data bits
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;   // 1-second total read timeout
    tcsetattr(fd, TCSANOW, &options);

    std::cout << "Serial port " << portName << " opened. Reading data..." << std::endl;

    // 2. SETUP PARSING (REGEX) FOR SENSOR DATA
    const int bufferSize = 256;
    char buffer[bufferSize];
    std::string lineBuffer;

    std::regex accelRegex(R"(Acceleration:\s*X:\s*([-+]?[0-9]*\.?[0-9]+)g,\s*Y:\s*([-+]?[0-9]*\.?[0-9]+)g,\s*Z:\s*([-+]?[0-9]*\.?[0-9]+)g)");
    std::regex lis3dhTempRegex(R"(LIS3DH Temperature:\s*([-+]?[0-9]*\.?[0-9]+))");
    std::regex magRegex(R"(Magnetic Field:\s*X:\s*([-+]?[0-9]*\.?[0-9]+)\s*G,\s*Y:\s*([-+]?[0-9]*\.?[0-9]+)\s*G,\s*Z:\s*([-+]?[0-9]*\.?[0-9]+)\s*G)");
    std::regex lis2mdlTempRegex(R"(LIS2MDL Temperature:\s*([-+]?[0-9]*\.?[0-9]+))");

    // Data containers for sensor readings
    LIS3DHData lis3dh = {0, 0, 0, 0};
    LIS2MDLData lis2mdl = {0, 0, 0, 0};

    // 3. MAIN LOOP: READ SERIAL, PARSE, BUILD & SEND PACKETS
    while (true) {
        int n = read(fd, buffer, bufferSize - 1);
        if (n < 0) {
            std::cerr << "Error reading from serial: " << strerror(errno) << std::endl;
            break;
        } else if (n > 0) {
            buffer[n] = '\0';
            lineBuffer.append(buffer);

            // Process complete lines
            size_t pos = 0;
            while ((pos = lineBuffer.find('\n')) != std::string::npos) {
                std::string completeLine = lineBuffer.substr(0, pos);
                if (!completeLine.empty() && completeLine.back() == '\r') {
                    completeLine.pop_back();
                }
                std::cout << "Received: " << completeLine << std::endl;

                // 3a. PARSE SENSOR DATA
                std::smatch match;
                if (std::regex_search(completeLine, match, accelRegex)) {
                    lis3dh.ax = std::stof(match[1].str());
                    lis3dh.ay = std::stof(match[2].str());
                    lis3dh.az = std::stof(match[3].str());
                    std::cout << "Parsed Acceleration: X=" << lis3dh.ax
                              << "g, Y=" << lis3dh.ay << "g, Z=" << lis3dh.az << "g\n";
                } else if (std::regex_search(completeLine, match, lis3dhTempRegex)) {
                    lis3dh.temperature = std::stof(match[1].str());
                    std::cout << "Parsed LIS3DH Temperature: "
                              << lis3dh.temperature << "°C\n";
                } else if (std::regex_search(completeLine, match, magRegex)) {
                    lis2mdl.magX = std::stof(match[1].str());
                    lis2mdl.magY = std::stof(match[2].str());
                    lis2mdl.magZ = std::stof(match[3].str());
                    std::cout << "Parsed Magnetic Field: X=" << lis2mdl.magX
                              << " G, Y=" << lis2mdl.magY << " G, Z=" << lis2mdl.magZ << " G\n";
                } else if (std::regex_search(completeLine, match, lis2mdlTempRegex)) {
                    lis2mdl.temperature = std::stof(match[1].str());
                    std::cout << "Parsed LIS2MDL Temperature: "
                              << lis2mdl.temperature << "°C\n";
                } else {
                    std::cout << "Ignored: " << completeLine << std::endl;
                }
                // Erase processed line from buffer
                lineBuffer.erase(0, pos + 1);

                // 3b. BUILD & SEND PACKET if we have valid readings
                // (Simplified check: any non-zero reading from LIS3DH)
                if ( (lis3dh.ax != 0.0f) || (lis3dh.ay != 0.0f) ||
                     (lis3dh.az != 0.0f) || (lis3dh.temperature != 0.0f) )
                {
                    // i. Application Layer
                    std::vector<uint8_t> appPayload = encodeApplicationData(lis3dh, lis2mdl);

                    // ii. Network Layer (PID=1, local=true)
                    std::vector<uint8_t> networkPacket = encodeNetworkPacket(1, appPayload, true);

                    // iii. Data Link Layer
                    std::vector<uint8_t> framedPacket = framePacket(networkPacket);

                    // Print final framed packet
                    std::cout << "Final Framed Packet: ";
                    for (uint8_t byte : framedPacket) {
                        printf("%02X ", byte);
                    }
                    std::cout << std::endl;

                    // 3c. SEND OVER UDP
                    //    Open a UDP socket, send to remote IP:port, then close.
                    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
                    if (sockfd < 0) {
                        perror("socket creation failed");
                        continue;
                    }

                    struct sockaddr_in destAddr;
                    memset(&destAddr, 0, sizeof(destAddr));
                    destAddr.sin_family = AF_INET;
                    destAddr.sin_port = htons(5005);  // <-- set your desired port
                    // e.g., remote side at 10.240.0.50
                    if (inet_pton(AF_INET, "10.240.0.50", &destAddr.sin_addr) <= 0) {
                        std::cerr << "Invalid remote IP address\n";
                        close(sockfd);
                        continue;
                    }

                    ssize_t sentBytes = sendto(sockfd,
                                               framedPacket.data(),
                                               framedPacket.size(),
                                               0,
                                               (struct sockaddr*)&destAddr,
                                               sizeof(destAddr));
                    if (sentBytes < 0) {
                        perror("sendto failed");
                    } else {
                        std::cout << "Sent " << sentBytes
                                  << " bytes to 10.240.0.50:5005\n";
                    }
                    close(sockfd);
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
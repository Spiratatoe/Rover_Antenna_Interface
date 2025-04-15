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

// Sockets for a mac or linux machine, change if windows
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

// other layers
#include "Data_Link_Layer.h"   // framePacket(), unframePacket()
#include "Network_Layer.h"     // encodeNetworkPacket(), decodeNetworkPacket()
#include "Application_Layer.h" // encodeApplicationData(), encodeGPSData(), and sensor/GPS data structs

// parse GPS data
bool parseGNGGA(const std::string &line, GPSData &gpsOut) {
    if (line.rfind("$GNGGA", 0) != 0) {
        return false;
    }
    std::vector<std::string> tokens;
    size_t start = 0;
    while (true) {
        size_t pos = line.find(',', start);
        if (pos == std::string::npos) {
            tokens.push_back(line.substr(start));
            break;
        }
        tokens.push_back(line.substr(start, pos - start));
        start = pos + 1;
    }
    if (tokens.size() < 10) {
        return false;
    }
    float latRaw = 0.0f;
    try {
        latRaw = std::stof(tokens[2]);
    } catch (...) {
        return false;
    }
    float latDeg = std::floor(latRaw / 100.0f);
    float latMin = latRaw - (latDeg * 100.0f);
    float latitude = latDeg + (latMin / 60.0f);
    if (tokens[3] == "S") {
        latitude = -latitude;
    }
    float lonRaw = 0.0f;
    try {
        lonRaw = std::stof(tokens[4]);
    } catch (...) {
        return false;
    }
    float lonDeg = std::floor(lonRaw / 100.0f);
    float lonMin = lonRaw - (lonDeg * 100.0f);
    float longitude = lonDeg + (lonMin / 60.0f);
    if (tokens[5] == "W") {
        longitude = -longitude;
    }
    float altitude = 0.0f;
    try {
        altitude = std::stof(tokens[9]);
    } catch (...) {
        altitude = 0.0f;
    }
    gpsOut.lat = latitude;
    gpsOut.lon = longitude;
    gpsOut.alt = altitude;
    return true;
}

int main() {
    const char* portName = "/dev/tty.usbmodem2101";  // MAKE SURE TO CHANGE TO PORT YOU'RE USING!!!
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Error opening serial port " << portName
                  << ": " << strerror(errno) << std::endl;
        return 1;
    }

    fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;   // 1-second timeout
    tcsetattr(fd, TCSANOW, &options);

    std::cout << "Serial port " << portName << " opened. Reading data..." << std::endl;

    //Parse Sensor Data
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
    GPSData gps = {0.0f, 0.0f, 0.0f};

    // LOOP: read serial, parse, build & send packets
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
                std::smatch match;

                // Parse sensor data from the line, if applicable
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
                } else if (completeLine.rfind("$GNGGA", 0) == 0) {
                    if (parseGNGGA(completeLine, gps)) {
                        std::cout << "Parsed GPS: lat=" << gps.lat
                                  << ", lon=" << gps.lon
                                  << ", alt=" << gps.alt << std::endl;
                    }
                } else {
                    std::cout << "Ignored: " << completeLine << std::endl;
                }
                // Remove the processed line
                lineBuffer.erase(0, pos + 1);

                // Changed to send data anyways for demo, since gps wouldn't be able to catch at the location
                // therefore build & send packet ALWAYS (regardless of sensor validity)
                // For this example, we choose to send GPS data (PID = 2) if available,
                // otherwise send sensor data (PID = 1). But even if GPS values are zero, we send them.
                std::vector<uint8_t> appPayload;
                uint8_t pid = 1;  // default: sensor data
                // Always send the GPS packet regardless of whether it has nonzero values.
                if (completeLine.rfind("$GNGGA", 0) == 0) {
                    appPayload = encodeGPSData(gps);
                    pid = 2;
                } else {
                    appPayload = encodeApplicationData(lis3dh, lis2mdl);
                    pid = 1;
                }

                // Build network packet with chosen PID.
                std::vector<uint8_t> networkPacket = encodeNetworkPacket(pid, appPayload, true);

                // Frame the network packet.
                std::vector<uint8_t> framedPacket = framePacket(networkPacket);

                // Print the final framed packet.
                std::cout << "Final Framed Packet: ";
                for (uint8_t byte : framedPacket) {
                    printf("%02X ", byte);
                }
                std::cout << std::endl;

                // Send the packet over UDP.
                int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
                if (sockfd < 0) {
                    perror("socket creation failed");
                    continue;
                }

                struct sockaddr_in destAddr;
                memset(&destAddr, 0, sizeof(destAddr));
                destAddr.sin_family = AF_INET;
                destAddr.sin_port = htons(5005);  // Destination port
                // Set destination IP address (adjust as needed for your network)
                if (inet_pton(AF_INET, "172.20.10.12", &destAddr.sin_addr) <= 0) {
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
                    std::cout << "Sent " << sentBytes << " bytes to 172.20.10.12:5005\n";
                }
                close(sockfd);
            }
        }
    }

    close(fd);
    return 0;
}
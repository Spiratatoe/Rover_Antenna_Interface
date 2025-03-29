#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <cstdio>

#pragma comment(lib, "Ws2_32.lib")

int main() {
    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if(iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return 1;
    }

    // Create a UDP socket
    SOCKET ListenSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (ListenSocket == INVALID_SOCKET) {
        std::cerr << "Socket creation failed: " << WSAGetLastError() << std::endl;
        WSACleanup();
        return 1;
    }

    // Set up the local address structure to bind to port 5005
    sockaddr_in service;
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    service.sin_port = htons(5005);

    // Bind the socket
    if (bind(ListenSocket, (SOCKADDR*)&service, sizeof(service)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    std::cout << "Listening for UDP packets on port 5005..." << std::endl;

    // Buffer to store incoming data
    const int recvbuflen = 1024;
    char recvbuf[recvbuflen];

    sockaddr_in senderAddr;
    int senderAddrSize = sizeof(senderAddr);

    while (true) {
        // Receive data
        int bytesReceived = recvfrom(ListenSocket, recvbuf, recvbuflen, 0,
                                     (SOCKADDR*)&senderAddr, &senderAddrSize);
        if (bytesReceived == SOCKET_ERROR) {
            std::cerr << "recvfrom failed: " << WSAGetLastError() << std::endl;
            break;
        }

        // Print sender information
        char senderIP[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(senderAddr.sin_addr), senderIP, INET_ADDRSTRLEN);
        std::cout << "Received " << bytesReceived << " bytes from "
                  << senderIP << ":" << ntohs(senderAddr.sin_port) << std::endl;

        // Print received data in hexadecimal
        std::cout << "Data: ";
        for (int i = 0; i < bytesReceived; ++i) {
            printf("%02X ", (unsigned char)recvbuf[i]);
        }
        std::cout << std::endl;
    }

    // Cleanup
    closesocket(ListenSocket);
    WSACleanup();
    return 0;
}

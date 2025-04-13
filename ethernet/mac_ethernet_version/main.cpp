#include <arpa/inet.h>   // for inet_ntop, htons, etc.
#include <sys/socket.h>  // for socket, bind, recvfrom
#include <netinet/in.h>  // for sockaddr_in
#include <unistd.h>      // for close
#include <errno.h>       // for errno
#include <string.h>      // for memset
#include <iostream>
#include <cstdio>

int main() {
    // Create a UDP socket (AF_INET = IPv4, SOCK_DGRAM = UDP)
    int listenSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (listenSocket < 0) {
        perror("socket creation failed");
        return 1;
    }

    // Set up the local address structure
    sockaddr_in service;
    memset(&service, 0, sizeof(service));
    service.sin_family = AF_INET;
    service.sin_addr.s_addr = INADDR_ANY;  // Listen on all interfaces
    service.sin_port = htons(5005);        // <-- Choose your port (e.g., 5005)

    // Bind the socket
    if (bind(listenSocket, (struct sockaddr*)&service, sizeof(service)) < 0) {
        perror("bind failed");
        close(listenSocket);
        return 1;
    }

    std::cout << "Listening for UDP packets on port 5005..." << std::endl;

    // Buffer to store incoming data
    const int recvbuflen = 1024;
    char recvbuf[recvbuflen];

    sockaddr_in senderAddr;
    socklen_t senderAddrSize = sizeof(senderAddr);

    while (true) {
        // Receive data
        int bytesReceived = recvfrom(listenSocket, recvbuf, recvbuflen, 0,
                                     (struct sockaddr*)&senderAddr, &senderAddrSize);
        if (bytesReceived < 0) {
            perror("recvfrom failed");
            break;
        }

        // Convert sender IP to readable form
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
    close(listenSocket);
    return 0;
}
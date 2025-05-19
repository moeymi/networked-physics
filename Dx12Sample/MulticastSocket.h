#pragma once
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <stdexcept>

class MulticastSocket {
public:
    MulticastSocket(const std::string& groupIp, unsigned short port, const std::string& localInterfaceIp);
    ~MulticastSocket();

    void send(const char* data, int size);
    int receive(char* buffer, int bufferSize, sockaddr_in* sender = nullptr);

    SOCKET getSocket() const { return m_socket; }

private:
    SOCKET m_socket;
    sockaddr_in m_groupAddr;
};

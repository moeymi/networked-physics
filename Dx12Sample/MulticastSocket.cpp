#include "MulticastSocket.h"
#include "MulticastSocket.h"
#include <iostream>

MulticastSocket::MulticastSocket(const std::string& groupIp, unsigned short port, const std::string& localInterfaceIp)
    : m_socket(INVALID_SOCKET)
{
    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (m_socket == INVALID_SOCKET)
        throw std::runtime_error("Failed to create socket");

    // Allow multiple sockets to bind to the same port
    int reuse = 1;
    if (setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse)) < 0)
        throw std::runtime_error("setsockopt(SO_REUSEADDR) failed");

    // Bind to local address
    sockaddr_in localAddr{};
    localAddr.sin_family = AF_INET;
    localAddr.sin_port = htons(port);
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(m_socket, (sockaddr*)&localAddr, sizeof(localAddr)) < 0)
        throw std::runtime_error("bind() failed");

    // Join multicast group
    ip_mreq mreq{};

    inet_pton(AF_INET, groupIp.c_str(), &mreq.imr_multiaddr);
    inet_pton(AF_INET, localInterfaceIp.c_str(), &mreq.imr_interface);

    if (setsockopt(m_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0)
        throw std::runtime_error("setsockopt(IP_ADD_MEMBERSHIP) failed");

    // Set default group address for sending
    m_groupAddr.sin_family = AF_INET;
    m_groupAddr.sin_port = htons(port);
    inet_pton(AF_INET, groupIp.c_str(), &m_groupAddr.sin_addr);

    // Set multicast TTL
    int ttl = 1;
    setsockopt(m_socket, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl));
}

MulticastSocket::~MulticastSocket() {
    if (m_socket != INVALID_SOCKET) {
        closesocket(m_socket);
    }
}

void MulticastSocket::send(const char* data, int size) {
    int result = sendto(m_socket, data, size, 0, (sockaddr*)&m_groupAddr, sizeof(m_groupAddr));
    if (result == SOCKET_ERROR) {
        int err = WSAGetLastError();
        std::cerr << "sendto failed: " << err << std::endl;
    }
}

int MulticastSocket::receive(char* buffer, int bufferSize, sockaddr_in* sender) {
    int senderLen = sizeof(sockaddr_in);
    sockaddr_in senderAddr{};
    int bytes = recvfrom(m_socket, buffer, bufferSize, 0, (sockaddr*)&senderAddr, &senderLen);
    if (bytes == SOCKET_ERROR) return -1;

    if (sender) *sender = senderAddr;
    return bytes;
}

// MulticastSocket.h
#pragma once
#include "Socket.h"
#include "IPAddress.h"
#include <iostream>

class MulticastSocket : public Socket
{
public:
    /*  group  – e.g. 239.255.42.42:5000 */
    MulticastSocket(const IPAddress& group,
        const IPAddress& iface);

    ~MulticastSocket() override = default;

    void send(const char* data, int size);
    int  receive(char* buffer, int bufSize, sockaddr_in* sender = nullptr);

private:
    sockaddr_in m_groupSa{};
};

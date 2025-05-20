#pragma once
#include <winsock2.h>
#include <stdexcept>
#include "IPAddress.h"

class Socket
{
public:
    Socket() = default;
    explicit Socket(SOCKET s) : sock_(s) {}
    virtual ~Socket() { close(); }

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
    Socket& operator=(Socket&& other) noexcept
    {
        if (this != &other)
        {
            close();
            sock_ = other.sock_;
            other.sock_ = INVALID_SOCKET;
        }
        return *this;
    }

    bool    isValid() const { return sock_ != INVALID_SOCKET; }
    SOCKET  native()  const { return sock_; }

    void close() noexcept
    {
        if (isValid())
        {
            ::closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
    }

    // helpers that accept IPAddress ------------------------------

    void bind(const IPAddress& addr)
    {
        sockaddr_in sa = addr.toSockAddr();
        if (::bind(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0)
            throw std::runtime_error("bind() failed");
    }

    void connect(const IPAddress& addr)
    {
        sockaddr_in sa = addr.toSockAddr();
        if (::connect(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0)
            throw std::runtime_error("connect() failed");
    }

protected:
    SOCKET sock_ = INVALID_SOCKET;
};

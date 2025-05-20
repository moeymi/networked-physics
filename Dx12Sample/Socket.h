// Socket.hpp
#pragma once
#include <winsock2.h>
#include <system_error>

class Socket {
public:
    Socket() = default;
    explicit Socket(SOCKET s) : sock_(s) {}

    virtual ~Socket() noexcept { close(); }

    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;

    Socket(Socket&& other) noexcept
        : sock_(other.sock_) {
        other.sock_ = INVALID_SOCKET;
    }

    Socket& operator=(Socket&& other) noexcept {
        if (this != &other) {
            close();
            sock_ = other.sock_;
            other.sock_ = INVALID_SOCKET;
        }
        return *this;
    }

    bool    isValid() const { return sock_ != INVALID_SOCKET; }
    SOCKET  native()    const { return sock_; }

    void close() noexcept {
        if (isValid()) {
            ::closesocket(sock_);
            sock_ = INVALID_SOCKET;
        }
    }

    void setBlocking(bool blocking) {
        u_long mode = blocking ? 0 : 1;
        int rc = ioctlsocket(sock_, FIONBIO, &mode);
        if (rc != NO_ERROR) {
            throw std::system_error{
                static_cast<int>(WSAGetLastError()),
                std::system_category(),
                "ioctlsocket(FIONBIO) failed"
            };
        }
    }

protected:
    SOCKET sock_ = INVALID_SOCKET;
};

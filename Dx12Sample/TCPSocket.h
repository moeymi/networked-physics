#pragma once
#include "Socket.h"
#include "IPAddress.h"
#include <vector>
#include <string_view>

class TCPSocket : public Socket
{
public:
    enum class Mode { Active, Passive };

    /* Active (client-side) constructor
       - creates a blocking socket, then connects           */
    explicit TCPSocket(const IPAddress& remote,
        bool nonBlocking = false);

    /* Passive (server-side) constructor
       - creates a listening socket bound to localAddr      */
    TCPSocket(const IPAddress& localAddr,
        int backlog = SOMAXCONN,
        bool nonBlocking = false);

    /* Adopt an accepted socket (used by accept())          */
    explicit TCPSocket(SOCKET accepted) : Socket(accepted) {}

    /* --- server only ------------------------------------ */
    TCPSocket accept(IPAddress* peerAddr = nullptr);

    /* --- client + server -------------------------------- */
    size_t send(const void* data, size_t len, int flags = 0);
    size_t recv(void* data, size_t len, int flags = 0);

    /* utility helpers                                      */
    void    setNonBlocking(bool on);
    int     lastError() const { return ::WSAGetLastError(); }

private:
    explicit TCPSocket(Mode m) : mode_(m) {}
    Mode mode_;
};

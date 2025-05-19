#include "TCPSocket.h"
#include <stdexcept>

static SOCKET makeSocket(bool nonBlocking)
{
    SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == INVALID_SOCKET)
        throw std::runtime_error("socket() failed");

    if (nonBlocking)
    {
        u_long nb = 1;
        ::ioctlsocket(s, FIONBIO, &nb);
    }
    return s;
}

/* ---------- Active -------------------------------------------------- */
TCPSocket::TCPSocket(const IPAddress& remote, bool nonBlocking)
    : Socket(makeSocket(nonBlocking)), mode_(Mode::Active)
{
    sockaddr_in sa = remote.toSockAddr();

    if (::connect(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0)
    {
        int e = ::WSAGetLastError();
        if (!(nonBlocking && e == WSAEWOULDBLOCK))
            throw std::runtime_error("connect() failed (" + std::to_string(e) + ")");
    }
}

/* ---------- Passive ------------------------------------------------- */
TCPSocket::TCPSocket(const IPAddress& localAddr, int backlog, bool nonBlocking)
    : Socket(makeSocket(nonBlocking)), mode_(Mode::Passive)
{
    sockaddr_in sa = localAddr.toSockAddr();
    if (::bind(sock_, reinterpret_cast<sockaddr*>(&sa), sizeof(sa)) != 0)
        throw std::runtime_error("bind() failed");

    if (::listen(sock_, backlog) != 0)
        throw std::runtime_error("listen() failed");
}

/* ---------- Accept -------------------------------------------------- */
TCPSocket TCPSocket::accept(IPAddress* peerAddr)
{
    sockaddr_in sa{};
    int         len = sizeof(sa);

    SOCKET s = ::accept(sock_, reinterpret_cast<sockaddr*>(&sa), &len);
    if (s == INVALID_SOCKET)
        throw std::runtime_error("accept() failed");

    if (peerAddr) *peerAddr = IPAddress(sa);
    return TCPSocket(s);              // move-constructs
}

/* ---------- Send / Recv -------------------------------------------- */
size_t TCPSocket::send(const void* data, size_t len, int flags)
{
    int n = ::send(sock_,
        reinterpret_cast<const char*>(data),
        static_cast<int>(len), flags);
    if (n == SOCKET_ERROR)
        throw std::runtime_error("send() failed (" +
            std::to_string(::WSAGetLastError()) + ")");
    return static_cast<size_t>(n);
}

size_t TCPSocket::recv(void* data, size_t len, int flags)
{
    int n = ::recv(sock_,
        reinterpret_cast<char*>(data),
        static_cast<int>(len), flags);
    if (n == SOCKET_ERROR)
        throw std::runtime_error("recv() failed (" +
            std::to_string(::WSAGetLastError()) + ")");
    return static_cast<size_t>(n);
}

/* ---------- Misc ---------------------------------------------------- */
void TCPSocket::setNonBlocking(bool on)
{
    u_long nb = on ? 1 : 0;
    ::ioctlsocket(sock_, FIONBIO, &nb);
}

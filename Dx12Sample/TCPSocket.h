// TCPSocket.hpp
#pragma once
#include "Socket.h"
#include "IPAddress.h"
#include <winsock2.h>
#include <ws2tcpip.h>
#include <system_error>
#include <vector>

class TCPSocket : public Socket {
private:
    void bind(const IPAddress& addr) {
        auto saOpt = addr.toSockAddr();
        if (!saOpt)
            throw std::runtime_error("Invalid address");

        // Extract the actual sockaddr_in out of the optional
        const sockaddr_in& sin = *saOpt;

        // Cast to sockaddr* and use sizeof(sockaddr_in)
        if (::bind(sock_,
            reinterpret_cast<const sockaddr*>(&sin),
            static_cast<int>(sizeof(sin))) != 0)
        {
            auto ec = WSAGetLastError();
            throw std::system_error{ ec,
                                     std::system_category(),
                                     "bind() failed" };
        }
    }
public:
    explicit TCPSocket(bool blocking = true) {
        SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (s == INVALID_SOCKET) {
            throw std::system_error{
                static_cast<int>(WSAGetLastError()),
                std::system_category(),
                "socket(AF_INET, SOCK_STREAM) failed"
            };
        }
        sock_ = s;
        setBlocking(blocking);
    }

    TCPSocket(SOCKET s, bool blocking = true)
        : Socket(s)
    {
        setBlocking(blocking);
    }

    /// Server: bind + listen
    void listen(const IPAddress& addr, int backlog = SOMAXCONN) {
        bind(addr);
        if (::listen(sock_, backlog) != 0) {
            throw std::system_error{
                static_cast<int>(WSAGetLastError()),
                std::system_category(),
                "listen() failed"
            };
        }
    }

    TCPSocket accept() {
        SOCKET client = ::accept(sock_, nullptr, nullptr);
        if (client == INVALID_SOCKET) {
            int ec = WSAGetLastError();
            if (ec == WSAEWOULDBLOCK) {
                return TCPSocket(INVALID_SOCKET, isBlocking());
            }
			std::string err = "accept() failed: " + std::to_string(ec);
			OutputDebugStringA(err.c_str());
        }
        return TCPSocket(client, isBlocking());
    }

    int connect(const IPAddress& addr) {
        auto saOpt = addr.toSockAddr();
        if (!saOpt) throw std::runtime_error("Invalid address");
        const sockaddr_in& sa = *saOpt;
        int rc = ::connect(sock_, reinterpret_cast<const sockaddr*>(&sa), sizeof(sa));
        if (rc != 0) {
            int ec = WSAGetLastError();
            if (!(ec == WSAEWOULDBLOCK || ec == WSAEINPROGRESS)) {
				std::string err = "connect() failed: " + std::to_string(ec);
				OutputDebugStringA(err.c_str());
            }
        }
        return rc;
    }

    int sendAll(const void* data, size_t len) {
        const char* ptr = static_cast<const char*>(data);
        int sentAll = 0;
        while (len > 0) {
            int sent = ::send(sock_, ptr, static_cast<int>(len), 0);
            if (sent == SOCKET_ERROR) {
                int ec = WSAGetLastError();
                if (ec == WSAEWOULDBLOCK) {
                    // caller must wait for writability
                    break;
                }
				std::string err = "send() failed: " + std::to_string(ec);
				OutputDebugStringA(err.c_str());
            }
			sentAll += sent;
            ptr += sent;
            len -= sent;
        }
        return sentAll;
    }

    int recv(void* buffer, size_t maxlen) {
        int recvd = ::recv(sock_, static_cast<char*>(buffer), static_cast<int>(maxlen), 0);
        if (recvd == SOCKET_ERROR) {
            int ec = WSAGetLastError();
            if (ec != WSAEWOULDBLOCK) {
                std::string err = "recv() failed: " + std::to_string(ec);
                OutputDebugStringA(err.c_str());
            }
        }
        return recvd;
    }

	std::optional<sockaddr_in> getSockAddr() const {
		sockaddr_in addr;
		int addrLen = sizeof(addr);
		if (getsockname(sock_, reinterpret_cast<sockaddr*>(&addr), &addrLen) == SOCKET_ERROR) {
			int ec = WSAGetLastError();
			if (ec != WSAEWOULDBLOCK) {
				auto err = std::string("getsockname() failed: ") + std::to_string(ec);
				OutputDebugStringA(err.c_str());
				return std::nullopt;
			}
		}
		return addr;
	}

    bool isBlocking() const {
        return true;
    }
};

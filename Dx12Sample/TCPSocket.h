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
    int bind(const IPAddress& addr) {
        auto saOpt = addr.toSockAddr();
        if (!saOpt)
            throw std::runtime_error("Invalid address");

        // Extract the actual sockaddr_in out of the optional
        const sockaddr_in& sin = *saOpt;

		int rc = ::bind(sock_, reinterpret_cast<const sockaddr*>(&sin), sizeof(sin));
        if (rc != 0)
        {
            auto ec = WSAGetLastError();
            auto err = std::string("bind() failed: ") + std::to_string(ec);
            OutputDebugStringA(err.c_str());
        }
        return rc;
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

    int listen(const IPAddress& addr, int backlog = SOMAXCONN) {
        int rc = bind(addr);
		if (rc != 0) {
			return rc;
		}
		rc = ::listen(sock_, backlog);
        if (rc != 0) {
            int ec = WSAGetLastError();
            auto err = std::string("listen() failed: ") + std::to_string(ec);
            OutputDebugStringA(err.c_str());
        }
		return rc;
    }

    TCPSocket accept() {
        SOCKET client = ::accept(sock_, nullptr, nullptr);
        if (client == INVALID_SOCKET) {
            int ec = WSAGetLastError();
            if (ec == WSAEWOULDBLOCK) {
                auto err = std::string("accept() failed: ") + std::to_string(ec);
                OutputDebugStringA(err.c_str());
            }
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
                auto err = std::string("connect() failed: ") + std::to_string(ec);
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
                auto err = std::string("send() failed: ") + std::to_string(ec);
                OutputDebugStringA(err.c_str());
				return sentAll;
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
            if (ec != WSAEWOULDBLOCK)
            {
				auto err = std::string("recv() failed: ") + std::to_string(ec);
				OutputDebugStringA(err.c_str());
			}
        }
        return recvd;
    }

    bool isBlocking() const {
        return true;
    }
};

#include "IPAddress.h"
#include <iphlpapi.h>
#include <ws2tcpip.h>
#include <vector>
#include <string>
#include <Windows.h>

#pragma comment(lib, "iphlpapi.lib") // Link against the IP Helper API library to resolve any linker issues.

static std::string getFirstLocalAddress() {
    // 1) Get the local host name
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) != 0) {
        throw std::system_error(WSAGetLastError(), std::system_category(),
            "gethostname failed");
    }

    // 2) Ask getaddrinfo for IPv4 addresses only
    addrinfo hints = {};
    hints.ai_family = AF_INET;     // <-- only IPv4 now
    hints.ai_socktype = SOCK_STREAM; // doesn't really matter
    hints.ai_flags = AI_CANONNAME;

    addrinfo* result = nullptr;
    int rc = getaddrinfo(hostname, nullptr, &hints, &result);
    if (rc != 0) {
        throw std::runtime_error(std::string("getaddrinfo failed: ")
            + gai_strerrorA(rc));
    }

    // 3) Walk the list, skip loopback
    std::string found;
    for (addrinfo* p = result; p; p = p->ai_next) {
        auto* sin = reinterpret_cast<sockaddr_in*>(p->ai_addr);
        char addrbuf[INET_ADDRSTRLEN] = { 0 };
        inet_ntop(AF_INET, &sin->sin_addr, addrbuf, sizeof(addrbuf));
        if (std::strcmp(addrbuf, "127.0.0.1") == 0)
            continue;
        found = addrbuf;
        break;
    }

    freeaddrinfo(result);
    if (found.empty())
        found = "127.0.0.1";
    return found;
}

IPAddress::IPAddress() : m_host("0.0.0.0"), m_port(0) {}

IPAddress::IPAddress(const std::string& host, uint16_t port)
	: m_host(host), m_port(port) {}

IPAddress::IPAddress(const sockaddr_in& sa) {
	char str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(sa.sin_addr), str, INET_ADDRSTRLEN);
	m_host = str;
	m_port = ntohs(sa.sin_port);
}

IPAddress IPAddress::initializeLocal(uint16_t port) {
    std::string host = getFirstLocalAddress();
    return IPAddress(host, port);
}

const std::string& IPAddress::host() const {
    return m_host;
}
std::optional<sockaddr_in> IPAddress::toSockAddr() const {
    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(m_port);
	if (inet_pton(AF_INET, m_host.c_str(), &addr.sin_addr) <= 0) {
		return std::nullopt; // Invalid address
	}
    return addr;
}
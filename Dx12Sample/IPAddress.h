#pragma once
#include "pch.h"
#include <string>

class IPAddress {
public:
	IPAddress();
	explicit IPAddress(const std::string& host, uint16_t port); 
    IPAddress(const sockaddr_in& sa);
    static IPAddress initializeLocal(uint16_t);

    const std::string& host() const;

    std::optional<sockaddr_in> toSockAddr() const;

    uint16_t port() const { return m_port; }
    std::string toString() const { return m_host + ":" + std::to_string(m_port); }

private:
    std::string m_host;
	uint16_t m_port = 0;
};

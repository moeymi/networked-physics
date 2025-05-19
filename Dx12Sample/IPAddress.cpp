#include "IPAddress.h"
#include <iphlpapi.h>
#include <ws2tcpip.h>
#include <vector>
#include <string>
#include <Windows.h>

#pragma comment(lib, "iphlpapi.lib") // Link against the IP Helper API library to resolve any linker issues.

void IPAddress::initialize(const std::string& address) {
	m_Address = address;
	if (m_Address.empty()) {
		initializeLocal();
	}
	else {
		// Validate the IP address format
		struct sockaddr_in sa;
		int result = inet_pton(AF_INET, m_Address.c_str(), &(sa.sin_addr));
		if (result <= 0) {
			OutputDebugStringA("Invalid IP address format.\n");
			initializeLocal();
		}
	}
}

void IPAddress::initializeLocal() {
    ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG family = AF_INET;
    ULONG bufLen = 0;
    if (GetAdaptersAddresses(family, flags, nullptr, nullptr, &bufLen) != ERROR_BUFFER_OVERFLOW) {
        m_Address = "127.0.0.1";
        return;
    }

    std::vector<char> buffer(bufLen);
    IP_ADAPTER_ADDRESSES* adapters = reinterpret_cast<IP_ADAPTER_ADDRESSES*>(buffer.data());
    if (GetAdaptersAddresses(family, flags, nullptr, adapters, &bufLen) == NO_ERROR) {
        for (auto* adapter = adapters; adapter; adapter = adapter->Next) {
            if (adapter->OperStatus != IfOperStatusUp)
                continue;
            if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK)
                continue;
            std::wstring friendlyName(adapter->FriendlyName);
            if (friendlyName.find(L"vEthernet") != std::wstring::npos)
                continue;

            for (auto* unicast = adapter->FirstUnicastAddress; unicast; unicast = unicast->Next) {
                SOCKADDR_IN* sa_in = reinterpret_cast<SOCKADDR_IN*>(unicast->Address.lpSockaddr);
                char str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(sa_in->sin_addr), str, INET_ADDRSTRLEN);
                std::string ip = str;
                if (ip != "127.0.0.1") {
                    m_Address = ip;
                    OutputDebugStringA(("Local address: " + m_Address + "\n").c_str());
                    return;
                }
            }
        }
    }

    m_Address = "127.0.0.1";
}

const std::string IPAddress::get() {
    return m_Address;
}
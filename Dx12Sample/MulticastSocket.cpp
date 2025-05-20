// MulticastSocket.cpp
#include "MulticastSocket.h"

MulticastSocket::MulticastSocket(const IPAddress& group,
    const IPAddress& iface)
{
    sock_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock_ == INVALID_SOCKET)
        throw std::runtime_error("socket() failed");

    /* allow several sockets to share the same port */
    int reuse = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR,
        reinterpret_cast<char*>(&reuse), sizeof(reuse));

    /* bind 0.0.0.0:<group.port()> so we can *receive* */
    sockaddr_in any{};
    any.sin_family = AF_INET;
    any.sin_port = htons(group.port());
    any.sin_addr.s_addr = htonl(INADDR_ANY);

    if (::bind(sock_, reinterpret_cast<sockaddr*>(&any), sizeof(any)) != 0)
        throw std::runtime_error("bind() failed");

    ip_mreq mreq{};

	auto groupAddr = group.toSockAddr();
	if (!groupAddr)
		throw std::runtime_error("Invalid group address");

	mreq.imr_multiaddr = groupAddr->sin_addr;           // 239.x.x.x

	auto ifaceAddr = iface.toSockAddr();
	if (!ifaceAddr)
		throw std::runtime_error("Invalid interface address");

	mreq.imr_interface = ifaceAddr->sin_addr;

    if (::setsockopt(sock_, IPPROTO_IP, IP_ADD_MEMBERSHIP,
        reinterpret_cast<char*>(&mreq), sizeof(mreq)) != 0)
        throw std::runtime_error("IP_ADD_MEMBERSHIP failed");

    /* never leaves the host.                                             */
    if (::setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_IF,
        reinterpret_cast<char*>(&mreq.imr_interface),
        sizeof(mreq.imr_interface)) != 0)
        std::cerr << "IP_MULTICAST_IF failed: " << WSAGetLastError() << '\n';
    /* ------------------------------------------------------------------ */

	auto groupAdd = group.toSockAddr();
	if (!groupAdd)
		throw std::runtime_error("Invalid group address");

    m_groupSa = groupAdd.value();

    int ttl = 1;
    ::setsockopt(sock_, IPPROTO_IP, IP_MULTICAST_TTL,
        reinterpret_cast<char*>(&ttl), sizeof(ttl));
}


void MulticastSocket::send(const char* data, int size)
{
    int n = ::sendto(sock_, data, size, 0,
        reinterpret_cast<const sockaddr*>(&m_groupSa),
        sizeof(m_groupSa));
    if (n == SOCKET_ERROR)
        std::cerr << "sendto failed: " << WSAGetLastError() << '\n';
}

int MulticastSocket::receive(char* buf, int bufSize, sockaddr_in* sender)
{
    sockaddr_in from{};
    int len = sizeof(from);

    int n = ::recvfrom(sock_, buf, bufSize, 0,
        reinterpret_cast<sockaddr*>(&from), &len);
    if (n == SOCKET_ERROR)
        return -1;

    if (sender) *sender = from;
    return n;
}

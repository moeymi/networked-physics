#include "NetworkEngine.h"
#include <chrono>
#include <iostream>
#include "SphereCollider.h"
#include "CapsuleCollider.h"
#include "BoxCollider.h"

#include "Application.h"
#include "CommandQueue.h"
#include "CommandList.h"
#include "GlobalData.h"
#include "PhysicsEngine.h"
#include "IPAddress.h"

NetworkEngine::NetworkEngine()
	: m_listenSocket(INVALID_SOCKET),
	m_sharedData(std::make_unique<SharedData>())
{
    m_materialMap[0] = Material::White;
	m_materialMap[1] = Material::Red;
	m_materialMap[2] = Material::Green;
	m_materialMap[3] = Material::Blue;
}

NetworkEngine::~NetworkEngine() {
    closesocket(m_listenSocket);
    WSACleanup();
}

std::vector<PeerInfo> NetworkEngine::getPeersInfo() const {
    std::vector<PeerInfo> peers(0);
    for (const auto& pair : m_peerInfoMap) {
        peers.push_back(pair.second);
    }
    return peers;
}

void NetworkEngine::setScnearioListener(std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&, const float&)> listener) {
	m_createScenario = listener;
}

void NetworkEngine::setStartSimulationListener(std::function<void(double)> listener) {
	m_startSimulation = listener;
}

void NetworkEngine::scheduleSimulationStart(float time) {
    flatbuffers::FlatBufferBuilder builder;
    auto msgType = builder.CreateString("StartSimulation");
    auto startSim = NetSim::CreateStartSimulation(builder, time);

    NetSim::NetworkMessageBuilder msg(builder);
    msg.add_msg_type(msgType);
    msg.add_data_type(NetSim::MessageUnion_StartSimulation);
    msg.add_data(startSim.Union());
    builder.Finish(msg.Finish());

    for (SOCKET s : m_peerSockets) {
        sendMessage(s, builder);
    }
}

void NetworkEngine::changeGravity(const float& gravity) {
	flatbuffers::FlatBufferBuilder builder;

	auto msgType = builder.CreateString("ChangeGravity");

	auto changeGravity = NetSim::CreateGravityChange(builder, gravity);
	NetSim::NetworkMessageBuilder msg(builder);

	msg.add_msg_type(msgType);
	msg.add_data_type(NetSim::MessageUnion_GravityChange);
	msg.add_data(changeGravity.Union());
	builder.Finish(msg.Finish());

	for (SOCKET s : m_peerSockets) {
		sendMessage(s, builder);
	}
}

void NetworkEngine::initializeSockets(unsigned short listenPort) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        throw std::runtime_error("WSAStartup failed");
    }

    m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_listenSocket == INVALID_SOCKET) {
        WSACleanup();
        throw std::runtime_error("Failed to create listen socket");
    }

    u_long nonBlocking = 1;
    ioctlsocket(m_listenSocket, FIONBIO, &nonBlocking);

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(listenPort);

    if (bind(m_listenSocket, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(m_listenSocket);
        WSACleanup();
        throw std::runtime_error("Bind failed");
    }

    if (listen(m_listenSocket, SOMAXCONN) == SOCKET_ERROR) {
        closesocket(m_listenSocket);
        WSACleanup();
        throw std::runtime_error("Listen failed");
    }

    std::thread(&NetworkEngine::listenForDiscovery, this, GlobalData::g_broadcastPort).detach();

    // Broadcast discovery message
    broadcastDiscovery(GlobalData::g_broadcastPort);
}

void NetworkEngine::connectToPeer(const std::string& ip, unsigned short port) {
    SOCKET peer = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (peer == INVALID_SOCKET) return;

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    inet_pton(AF_INET, ip.c_str(), &addr.sin_addr);
    addr.sin_port = htons(port);

    if (connect(peer, (sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR) {
        closesocket(peer);
        return;
    }

    u_long nonBlocking = 1;
    ioctlsocket(peer, FIONBIO, &nonBlocking);
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        m_peerSockets.push_back(peer);
    }
    sendRecognize(peer);
}

void NetworkEngine::removePeer(SOCKET peerSocket) {
    std::lock_guard<std::mutex> lock(m_peerMutex);
    closesocket(peerSocket);
    m_peerSockets.erase(std::remove(m_peerSockets.begin(), m_peerSockets.end(), peerSocket), m_peerSockets.end());
	m_peerInfoMap.erase(peerSocket);
}

void NetworkEngine::broadcastDiscovery(unsigned short discoveryPort) {
    SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpSocket == INVALID_SOCKET) {
        return;
    }

    // Set TTL for multicast (e.g., 1 limits it to local network)
    int ttl = 1;
    if (setsockopt(udpSocket, IPPROTO_IP, IP_MULTICAST_TTL, (char*)&ttl, sizeof(ttl)) < 0) {
        closesocket(udpSocket);
        return;
    }

    // Set up the multicast address
    sockaddr_in multicastAddr{};
    multicastAddr.sin_family = AF_INET;
    multicastAddr.sin_port = htons(discoveryPort);
    inet_pton(AF_INET, "239.255.0.1", &multicastAddr.sin_addr);

    std::string discoveryMessage = constructDiscoveryMessage();

    sendto(udpSocket, discoveryMessage.c_str(), static_cast<int>(discoveryMessage.size()), 0,
        reinterpret_cast<sockaddr*>(&multicastAddr), sizeof(multicastAddr));

    closesocket(udpSocket);
}

void NetworkEngine::listenForDiscovery(unsigned short discoveryPort) {
    SOCKET udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (udpSocket == INVALID_SOCKET) {
        return;
    }

    // Allow multiple sockets to bind to the same address/port
    int reuse = 1;
    setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&reuse, sizeof(reuse));

    sockaddr_in recvAddr{};
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(discoveryPort);
    recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udpSocket, reinterpret_cast<sockaddr*>(&recvAddr), sizeof(recvAddr)) == SOCKET_ERROR) {
        closesocket(udpSocket);
        return;
    }

    // Join multicast group
    ip_mreq mreq{};
    inet_pton(AF_INET, "239.255.0.1", &mreq.imr_multiaddr);
    IPAddress localAddress;// do this early in your engine startup
	localAddress.initializeLocal();
    inet_pton(AF_INET, localAddress.get().c_str(), &mreq.imr_interface.s_addr);

    if (setsockopt(udpSocket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0) {
        closesocket(udpSocket);
        return;
    }

    char buffer[1024];
    sockaddr_in senderAddr{};
    int senderAddrSize = sizeof(senderAddr);

    while (true) {
        int bytesReceived = recvfrom(udpSocket, buffer, sizeof(buffer), 0,
            reinterpret_cast<sockaddr*>(&senderAddr), &senderAddrSize);
        if (bytesReceived > 0) {
            auto message = NetSim::GetNetworkMessage(buffer);
            if (message->data_type() == NetSim::MessageUnion_DiscoveryBroadcast) {
                const NetSim::DiscoveryBroadcast* discovery = message->data_as_DiscoveryBroadcast();
                if (discovery->peer_id() == GlobalData::g_clientId) {
                    continue;
                }
                if (discovery->protocol_version() != PROTOCOL_VERSION) {
                    continue;
                }

                char ipStr[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &senderAddr.sin_addr, ipStr, sizeof(ipStr));
                std::string senderIP = ipStr;
                unsigned short senderPort = discovery->tcp_port();

                connectToPeer(senderIP, senderPort);
            }
        }
    }

    closesocket(udpSocket);
}

std::string NetworkEngine::constructDiscoveryMessage() {
    flatbuffers::FlatBufferBuilder builder;

    // Create fields
    auto name = builder.CreateString(GlobalData::g_clientName);
    NetSim::Vec3 color{ GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z };

    // Create the DiscoveryBroadcast payload
    auto discovery = NetSim::CreateDiscoveryBroadcast(
        builder,
        PROTOCOL_VERSION,
        GlobalData::g_clientId,
        name,
        GlobalData::g_listenPort,
        &color
    );

    // Wrap in a NetworkMessage
    auto msgType = builder.CreateString("DiscoveryBroadcast");
    NetSim::NetworkMessageBuilder msgBuilder(builder);
    msgBuilder.add_msg_type(msgType);
    msgBuilder.add_data_type(NetSim::MessageUnion_DiscoveryBroadcast);
    msgBuilder.add_data(discovery.Union());

    auto networkMessage = msgBuilder.Finish();
    builder.Finish(networkMessage);

    // Return serialized buffer as string
    return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

void NetworkEngine::sendPing(SOCKET peerSocket) {
    flatbuffers::FlatBufferBuilder builder;
    double now = GlobalData::getTimestamp();
    m_sentPingTimestamps[now] = now;
    auto ping = NetSim::CreatePing(builder, now);

    auto msgType = builder.CreateString("Ping");
    NetSim::NetworkMessageBuilder msg(builder);
    msg.add_msg_type(msgType);
    msg.add_data_type(NetSim::MessageUnion_Ping);
    msg.add_data(ping.Union());
    builder.Finish(msg.Finish());

    sendMessage(peerSocket, builder);
}

void NetworkEngine::sendPong(SOCKET peerSocket, const NetSim::Ping* ping) {
    flatbuffers::FlatBufferBuilder builder;
    double now = GlobalData::getTimestamp();

    auto pong = NetSim::CreatePong(builder, ping->sent_time(), now);
    auto msgType = builder.CreateString("Pong");

    NetSim::NetworkMessageBuilder msg(builder);
    msg.add_msg_type(msgType);
    msg.add_data_type(NetSim::MessageUnion_Pong);
    msg.add_data(pong.Union());
    builder.Finish(msg.Finish());

    sendMessage(peerSocket, builder);
}

void NetworkEngine::sendRecognize(SOCKET peerSocket) {  
    flatbuffers::FlatBufferBuilder builder;

    auto client_name = builder.CreateString(GlobalData::g_clientName);
    auto msg_type = builder.CreateString("Recognize");
	auto color = NetSim::Vec3{ GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z };

    auto recognize_offset = NetSim::CreateRecognize(builder, 1, GlobalData::g_clientId, client_name, GlobalData::g_listenPort, &color);

    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(msg_type);
    msg_builder.add_data_type(NetSim::MessageUnion_Recognize);
    msg_builder.add_data(recognize_offset.Union());

    auto message = msg_builder.Finish();

    builder.Finish(message);

    sendMessage(peerSocket, builder);

    sendPing(peerSocket);
}

void NetworkEngine::assignOwnersAndBroadcastScenarioCreate(std::string scenarioName, const std::vector<std::shared_ptr<PhysicsObject>>& physicsObjects, const float& gravity,
    std::vector<PhysicsObject*>& ownedObjects, PhysicsObject** unownedObjects) {
    flatbuffers::FlatBufferBuilder builder;

    std::vector<flatbuffers::Offset<NetSim::ObjectState>> objectStates;
    std::unordered_map<int, int> nonStaticObjects;

	m_sharedData->m_outgoingObjectStates[0].clear();
    m_sharedData->m_outgoingObjectStates[1].clear();

    int cnt = 0;
    for (int j = 0; j < physicsObjects.size(); j++) {
        if (!physicsObjects[j]->isStatic()) {
            nonStaticObjects[j] = cnt++;
        }
    }

    for (int i = 0; i < physicsObjects.size(); i++) {
        auto& object = physicsObjects[i];
        DirectX::XMFLOAT3 position;
        DirectX::XMStoreFloat3(&position, object->getTransform().GetPosition(0));

        DirectX::XMFLOAT4 rotation;
        DirectX::XMStoreFloat4(&rotation, object->getTransform().GetRotationQuaternion(0));

        DirectX::XMFLOAT3 scale;
        DirectX::XMStoreFloat3(&scale, object->getTransform().GetScale(0));

        DirectX::XMFLOAT3 colliderSize;
        auto collider = object->getCollider();
        auto type = object->getMeshType();
        switch (type)
        {
        case MeshType::Sphere:
        {
            // Pointer cast
            auto sphereCollider = static_cast<SphereCollider*>(collider);
            colliderSize = { sphereCollider->getRadius(), 0.0f, 0.0f };
            break;
        }
        case MeshType::Box:
        {
            auto boxCollider = static_cast<BoxCollider*>(collider);
            DirectX::XMStoreFloat3(&colliderSize, boxCollider->getHalfSize());
            break;
        }
        case MeshType::Capsule:
        {
            auto capsuleCollider = static_cast<CapsuleCollider*>(collider);
            colliderSize = { capsuleCollider->getRadius(), capsuleCollider->getHeight(), 0.0f };
            break;
        }
        case MeshType::Plane:
        {
            auto planeCollider = static_cast<BoxCollider*>(collider);
            DirectX::XMStoreFloat3(&colliderSize, planeCollider->getHalfSize());
            break;
        }
        default:
            throw std::runtime_error("Unknown collider type");
            break;
        }
        NetSim::Vec3 positionVec3 = { position.x, position.y, position.z };
        NetSim::Vec4 rotationVec4 = { rotation.x, rotation.y, rotation.z, rotation.w };
        NetSim::Vec3 scaleVec3 = { scale.x, scale.y, scale.z };
        NetSim::Vec3 colliderSizeVec3 = { colliderSize.x, colliderSize.y, colliderSize.z };

        uint16_t peerIndex = 0;
        uint32_t peerId = GlobalData::g_clientId;
        NetSim::Vec3 colorVec3 = { GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z };

        if (!object->isStatic()) {
            int distrib = ceil(static_cast<float>(nonStaticObjects.size()) / static_cast<float>(m_peerSockets.size() + 1));
            if (distrib == 0) distrib = 1;
            peerIndex = static_cast<uint16_t>(nonStaticObjects[i] / distrib);
            if (peerIndex > 0) {
                peerIndex--;
                peerId = m_peerInfoMap[m_peerSockets[peerIndex]].peer_id;
                auto clientColor = m_peerInfoMap[m_peerSockets[peerIndex]].color;
                colorVec3 = { clientColor.x, clientColor.y, clientColor.z };
				unownedObjects[object->getId()] = object.get();
			}
			else {
				peerId = GlobalData::g_clientId;
				ownedObjects.push_back(object.get());
            }
            object->setOwnerId(peerId);
        }
        else {
            // Static objects are always white
            colorVec3 = { 1.0f, 1.0f, 1.0f };
        }
        bool isStatic = object->isStatic();
        object->setColor({ colorVec3.x(), colorVec3.y(), colorVec3.z(), 1.0f });
        auto physicsMaterial = object->getPhysicsMaterial();

		NetSim::PhysicsMaterial material = { physicsMaterial.friction, physicsMaterial.angularFriction, physicsMaterial.restitution };
        auto objectState = NetSim::CreateObjectState(builder, object->getId(), isStatic, static_cast<NetSim::MeshType>(type), &material, &colliderSizeVec3, &positionVec3, &rotationVec4, &scaleVec3, &colorVec3, peerId);
        objectStates.push_back(objectState);
    }

	m_sharedData->m_outgoingObjectStates[1] = m_sharedData->m_outgoingObjectStates[0];

    auto objectsVector = builder.CreateVector(objectStates);
    auto scenarioId = builder.CreateString("Scenario");
    auto scenario_offset = NetSim::CreateScenario(builder, GlobalData::g_clientId, objectsVector, gravity);
    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(scenarioId);
    msg_builder.add_data_type(NetSim::MessageUnion_Scenario);
    msg_builder.add_data(scenario_offset.Union());
    auto message = msg_builder.Finish();
    builder.Finish(message);

    std::lock_guard<std::mutex> lock(m_peerMutex);
    for (SOCKET s : m_peerSockets) {
        sendMessage(s, builder);
    }
}

void NetworkEngine::sendMessage(SOCKET peerSocket, flatbuffers::FlatBufferBuilder& builder) {
    uint32_t size = htonl(static_cast<uint32_t>(builder.GetSize()));
    send(peerSocket, reinterpret_cast<const char*>(&size), sizeof(size), 0);
    send(peerSocket, reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize(), 0);
}

void NetworkEngine::cleanDirtyOutgoingObjects() {
	if (m_sharedData->m_ownedObjectsDirty) {
		m_sharedData->m_outgoingObjectStates[0] = m_sharedData->m_outgoingObjectStates[1];
		m_sharedData->m_ownedObjectsDirty = false;
	}
}

void NetworkEngine::sendPeerList(SOCKET to) {
    flatbuffers::FlatBufferBuilder builder;

    std::vector<flatbuffers::Offset<NetSim::PeerInfo>> peerList;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (const auto& [sock, info] : m_peerInfoMap) {
            auto ip = builder.CreateString(info.ip);
			NetSim::Vec3 color = { info.color.x, info.color.y, info.color.z };
            auto peer = NetSim::CreatePeerInfo(builder, info.peer_id, ip, info.port, &color);
            peerList.push_back(peer);
        }
    }

    auto peers = builder.CreateVector(peerList);
    auto list = NetSim::CreatePeerList(builder, peers);
    auto msgType = builder.CreateString("PeerList");

    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(msgType);
    msg_builder.add_data_type(NetSim::MessageUnion_PeerList);
    msg_builder.add_data(list.Union());
    builder.Finish(msg_builder.Finish());

    sendMessage(to, builder);
}

void NetworkEngine::sendObjectUpdatesToPeers(const std::vector<ObjectUpdate>& updates) {
    flatbuffers::FlatBufferBuilder builder;

    std::vector<flatbuffers::Offset<NetSim::ObjectUpdate>> fbUpdates;
    for (const auto& update : updates) {
        NetSim::Vec3 pos(update.position.x, update.position.y, update.position.z);
        NetSim::Vec4 rot(update.rotation.x, update.rotation.y, update.rotation.z, update.rotation.w);

		NetSim::Vec3 vel(update.velocity.x, update.velocity.y, update.velocity.z);
		NetSim::Vec3 angVel(update.angular_velocity.x, update.angular_velocity.y, update.angular_velocity.z);

		auto offset = NetSim::CreateObjectUpdate(builder, update.object_id, GlobalData::g_simulationTime, &pos, &rot, &vel, &angVel);
        fbUpdates.push_back(offset);
    }

    auto updateVec = builder.CreateVector(fbUpdates);
    auto msgType = builder.CreateString("ObjectUpdate");
    auto updatesOffset = NetSim::CreateObjectUpdateList(builder, updateVec); // you'll need this wrapper table

    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(msgType);
    msg_builder.add_data_type(NetSim::MessageUnion_ObjectUpdateList);
    msg_builder.add_data(updatesOffset.Union());
    builder.Finish(msg_builder.Finish());

    // Send to all peers
    std::lock_guard<std::mutex> lock(m_peerMutex);
    for (SOCKET s : m_peerSockets) {
        sendMessage(s, builder);
    }
}

void NetworkEngine::handleNewConnection() {
    SOCKET clientSocket = accept(m_listenSocket, nullptr, nullptr);
    if (clientSocket == INVALID_SOCKET) return;

    u_long nonBlocking = 1;
    ioctlsocket(clientSocket, FIONBIO, &nonBlocking);
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        m_peerSockets.push_back(clientSocket);
    }
    sendRecognize(clientSocket);
}

void NetworkEngine::handlePeerData(SOCKET peerSocket) {
    uint32_t size = 0;
    int bytesReceived = recv(peerSocket, reinterpret_cast<char*>(&size), sizeof(size), 0);
    if (bytesReceived <= 0) {
        removePeer(peerSocket);
        return;
    }

    size = ntohl(size);
    std::vector<char> buffer(size);
    bytesReceived = recv(peerSocket, buffer.data(), size, 0);
    if (bytesReceived <= 0) {
        removePeer(peerSocket);
        return;
    }

    auto message = NetSim::GetNetworkMessage(buffer.data());
    switch (message->data_type()) {
    case NetSim::MessageUnion_Recognize:
        handleRecognize(message->data_as_Recognize());
        break;
    case NetSim::MessageUnion_PeerList:
        handlePeerList(message->data_as_PeerList());
        break;
    case NetSim::MessageUnion_Scenario:
        handleScenario(message->data_as_Scenario());
        break;
    case NetSim::MessageUnion_ObjectUpdateList:
        handleObjectUpdate(peerSocket, message->data_as_ObjectUpdateList());
        break;
	case NetSim::MessageUnion_StartSimulation:
		handleStartSimulation(peerSocket, message->data_as_StartSimulation());
		break;
	case NetSim::MessageUnion_Ping:
		handlePing(peerSocket, message->data_as_Ping());
		break;
	case NetSim::MessageUnion_Pong:
		handlePong(peerSocket, message->data_as_Pong());
		break;
	case NetSim::MessageUnion_GravityChange:
		handleGravityChange(message->data_as_GravityChange());
		break;
    default:
        break;
    }
}

void NetworkEngine::handlePeerList(const NetSim::PeerList* list) {
    if (!list) return;

    for (const auto* peer : *list->peers()) {
        uint32_t peerId = peer->peer_id();
        std::string ip = peer->ip()->str();
        uint16_t port = peer->port();

        // Don't connect to self
        if (peerId == GlobalData::g_clientId) continue;

        // Already connected?
        bool alreadyConnected = false;
        {
            std::lock_guard<std::mutex> lock(m_peerMutex);
            for (const auto& [sock, info] : m_peerInfoMap) {
                if (info.peer_id == peerId) {
                    alreadyConnected = true;
                    break;
                }
            }
        }

        if (!alreadyConnected) {
            connectToPeer(ip, port);
        }
    }
}

void NetworkEngine::handlePing(SOCKET from, const NetSim::Ping* ping) {
    sendPong(from, ping);
}

void NetworkEngine::handlePong(SOCKET from, const NetSim::Pong* pong) {
    auto it = m_sentPingTimestamps.find(pong->ping_sent_time());
    if (it == m_sentPingTimestamps.end()) return;

    double T_send = it->second;
    double T_recv = GlobalData::getTimestamp();
    double T_remote = pong->remote_time();

    double rtt = T_recv - T_send;
    double offset = T_remote - (T_send + rtt * 0.5);
    m_peerClockOffsets[from] = offset;

    m_sentPingTimestamps.erase(it);
}

void NetworkEngine::handleRecognize(const NetSim::Recognize* recognize) {
    if (!recognize) return;

    if (recognize->protocol_version() != PROTOCOL_VERSION) {
        return;
    }

    SOCKET senderSocket = INVALID_SOCKET;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (SOCKET s : m_peerSockets) {
            // Map the recognize message to the correct socket
            // In real systems, you track current parsing socket explicitly
            if (m_peerInfoMap.find(s) == m_peerInfoMap.end()) {
                senderSocket = s;
                break;
            }
        }
    }

    if (senderSocket == INVALID_SOCKET) {
        return;
    }

    // Store peer info
    PeerInfo info;
    info.socket = senderSocket;
    info.peer_id = recognize->peer_id();
    info.client_name = recognize->client_name()->str();
    info.port = recognize->listen_port();
	info.color = { recognize->color()->x(), recognize->color()->y(), recognize->color()->z() };

    sockaddr_in addr;
    int len = sizeof(addr);
    getpeername(senderSocket, (sockaddr*)&addr, &len);

    char ipBuffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, ipBuffer, sizeof(ipBuffer));
    info.ip = ipBuffer;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        m_peerInfoMap[senderSocket] = info;
    }

    sendRecognize(senderSocket);
    sendPeerList(senderSocket);
}

void NetworkEngine::handleScenario(const NetSim::Scenario* scenario) {
	if (!scenario) return;

    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_DIRECT);
    auto commandList = commandQueue->GetCommandList();

	std::vector<std::shared_ptr<PhysicsObject>> objects;

	for (const auto* object : *scenario->objects()) {
		UINT objectId = object->id();
		auto objectType = static_cast<MeshType>(object->type());
		DirectX::XMFLOAT3 colliderSize = { object->collider_size()->x(), object->collider_size()->y(), object->collider_size()->z() };
		DirectX::XMFLOAT3 position = { object->position()->x(), object->position()->y(), object->position()->z() };
		DirectX::XMVECTOR rotation = DirectX::XMVectorSet(object->rotation()->x(), object->rotation()->y(), object->rotation()->z(), object->rotation()->w());
		DirectX::XMVECTOR scale = DirectX::XMVectorSet( object->scale()->x(), object->scale()->y(), object->scale()->z(), 1);
		DirectX::XMFLOAT4 color = { object->color()->x(), object->color()->y(), object->color()->z(), 1 };

        std::shared_ptr<Mesh> objectMesh;
        std::shared_ptr<Collider> collider;
        std::shared_ptr<Texture> texture = GlobalData::g_defaultTexture;;
        switch (objectType) {
		case MeshType::Sphere:
			objectMesh = GlobalData::g_sphereMesh;
			collider = std::make_shared<SphereCollider>(colliderSize.x);
			texture = GlobalData::g_customTexture;
			break;
		case MeshType::Box:
			objectMesh = GlobalData::g_boxMesh;
			collider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(colliderSize.x, colliderSize.y, colliderSize.z, 0));
			break;
		case MeshType::Capsule:
            objectMesh = Mesh::CreateCapsule(*commandList, colliderSize.x, colliderSize.y, 16);
			collider = std::make_shared<CapsuleCollider>(colliderSize.x, colliderSize.y);
			break;
		case MeshType::Plane:
			objectMesh = GlobalData::g_planeMesh;
			collider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(colliderSize.x, colliderSize.y, colliderSize.z, 0));
			break;
		default:
			throw std::runtime_error("Unknown object type");
        }

		auto physicsObject = std::make_shared<PhysicsObject>(objectId, objectType, objectMesh, texture);
		physicsObject->setCollider(collider);
		physicsObject->setColor(color);
		physicsObject->getTransform().SetPosition(position, 0, true);
		physicsObject->getTransform().SetRotationQuaternion(rotation, 0, true);
		physicsObject->getTransform().SetScale(scale, 0, true);
		physicsObject->setStatic(object->is_static());
		physicsObject->setOwnerId(object->authority_peer_id());
		physicsObject->setPhysicsMaterial({ object->material()->friction(), object->material()->angular_friction(), object->material()->restitution() });

		objects.push_back(physicsObject);
	}

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

	// Notify the scenario listener
	if (m_createScenario) {
		m_createScenario(std::move(objects), scenario->gravity());
	}
}

void NetworkEngine::handleObjectUpdate(SOCKET from, const NetSim::ObjectUpdateList* objectUpdateList) {
	if (!objectUpdateList) return;
    {
        std::lock_guard<std::mutex> lock(m_sharedData->m_incomingMutex);
        for (const auto* update : *objectUpdateList->updates()) {
            ObjectUpdate u;
            u.object_id = update->object_id();
            u.position = { update->position()->x(), update->position()->y(), update->position()->z() };
            u.rotation = { update->rotation()->x(), update->rotation()->y(), update->rotation()->z(), update->rotation()->w() };
			u.velocity = { update->velocity()->x(), update->velocity()->y(), update->velocity()->z() };
			u.angular_velocity = { update->angular_velocity()->x(), update->angular_velocity()->y(), update->angular_velocity()->z() };
			u.simulation_time = update->simulation_time() - m_peerClockOffsets[from];

            auto& deque = m_sharedData->m_objectUpdateHistory[update->object_id()];
            deque.push_back(u);

            // Remove outdated snapshots
            while (!deque.empty() && GlobalData::g_simulationTime - deque.front().simulation_time > m_sharedData->maxHistoryDuration) {
                deque.pop_front();
            }
        }

        m_sharedData->m_receivedNewAuthoritativeData = true;
    }
}

void NetworkEngine::handleStartSimulation(SOCKET peerSocket, const NetSim::StartSimulation* startSim) {
    double remoteTime = startSim->simulation_start_time();
    double offset = m_peerClockOffsets[peerSocket];

    double localStartTime = remoteTime - offset;
    m_startSimulation(localStartTime);
}

void NetworkEngine::handleGravityChange(const NetSim::GravityChange* gravityChange) {
	if (!gravityChange) return;
	float newGravity = gravityChange->new_value();
	PhysicsEngine::setGravity(newGravity);
}

void NetworkEngine::onUpdate(float deltaTime) {
    static double lastPingTime = 0.0;
    double now = GlobalData::getTimestamp();
    if (now - lastPingTime > 5.0) {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (SOCKET s : m_peerSockets) {
            sendPing(s);
        }
        lastPingTime = now;
    }

    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(m_listenSocket, &readSet);
    SOCKET maxSocket = m_listenSocket;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (SOCKET s : m_peerSockets) {
            FD_SET(s, &readSet);
            if (s > maxSocket) maxSocket = s;
        }
    }

    timeval timeout = { 0, 0 };
    int result = select(static_cast<int>(maxSocket + 1), &readSet, nullptr, nullptr, &timeout);

    if (result > 0) {
        if (FD_ISSET(m_listenSocket, &readSet)) {
            handleNewConnection();
        }

        for (SOCKET s : m_peerSockets) {
            if (FD_ISSET(s, &readSet)) {
                handlePeerData(s);
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_sharedData->m_outgoingMutex);
		if (m_sharedData->m_ownedObjectsDirty) {
			cleanDirtyOutgoingObjects();
		}
    }

    m_sharedData->m_outgoingMutex.lock();
    if (m_sharedData->m_outgoingObjectStates[1].size()) {
        sendObjectUpdatesToPeers(m_sharedData->m_outgoingObjectStates[1]);
    }
	m_sharedData->m_outgoingMutex.unlock();
}

void NetworkEngine::onStop() {
	for (SOCKET s : m_peerSockets) {
		closesocket(s);
	}
	m_peerSockets.clear();
	m_peerInfoMap.clear();
	closesocket(m_listenSocket);
	m_listenSocket = INVALID_SOCKET;
	WSACleanup();
}

void NetworkEngine::onStart() {}
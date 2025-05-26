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

NetworkEngine::NetworkEngine(RingBufferSPSC<Snapshot, 1024>* outgoingBuf, RingBufferSPSC<Snapshot, 1024>* incomingBuf) :
    m_outgoingBuffer(outgoingBuf), m_incomingBuffer(incomingBuf) {
}

NetworkEngine::~NetworkEngine() {
	if (m_listenSocket) {
		m_listenSocket->close();
	}
    WSACleanup();
}

std::vector<std::tuple<PeerInfo, double>> NetworkEngine::getPeersInfo() const {
    std::vector<std::tuple<PeerInfo, double>> peers(0);
    for (const auto& pair : m_peerInfoMap) {
		auto offset = getPeerRTT(pair.first);
        peers.push_back(std::make_tuple(pair.second, offset));
    }
    return peers;
}

void NetworkEngine::setScnearioListener(std::function<void(std::vector<std::shared_ptr<NetworkedObject>>&&, const float&)> listener) {
	m_createScenario = listener;
}

void NetworkEngine::setStartSimulationListener(std::function<void(double)> listener) {
	m_startSimulation = listener;
}

void NetworkEngine::setStopSimulationListener(std::function<void()> listener) {
	m_stopSimulation = listener;
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

    for (const auto& s : m_peerSockets) {
        sendMessage(s.get(), builder);
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

	for (const auto& s : m_peerSockets) {
		sendMessage(s.get(), builder);
	}
}

void NetworkEngine::stopSimulation() {
	flatbuffers::FlatBufferBuilder builder;
	auto msgType = builder.CreateString("StopSimulation");
	auto stopSim = NetSim::CreateStopSimulation(builder, GlobalData::g_tick);
	NetSim::NetworkMessageBuilder msg(builder);
	msg.add_msg_type(msgType);
	msg.add_data_type(NetSim::MessageUnion_StopSimulation);
	msg.add_data(stopSim.Union());
	builder.Finish(msg.Finish());
	for (const auto& s : m_peerSockets) {
		sendMessage(s.get(), builder);
	}
}

void NetworkEngine::initializeSockets(unsigned short listenPort) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
        throw std::runtime_error("WSAStartup failed");

    IPAddress listenAddr = IPAddress("0.0.0.0", listenPort);
    m_listenSocket = std::make_unique<TCPSocket>(false);   // passive ctor
    m_listenSocket->listen(listenAddr);

    constexpr char GROUP_IP[] = "239.255.42.42";
    const uint16_t GROUP_PORT = GlobalData::g_broadcastPort;

    IPAddress groupAddr(GROUP_IP, GROUP_PORT);
    IPAddress localIface = IPAddress::initializeLocal(GROUP_PORT);   // pick your NIC

    m_multicastSocket = std::make_unique<MulticastSocket>(groupAddr, localIface);
	m_multicastSocket->setBlocking(false);
}

void NetworkEngine::connectToPeer(const std::string& ip, uint16_t port)
{
    try
    {
        auto peer = std::make_unique<TCPSocket>(false); // non-blocking
        peer->connect(IPAddress(ip, port));

        if (isPeerConnected(peer.get())) {
            peer->close();
			Log::Warn() << "Peer " << ip << " already connected" << std::endl;
			return;
        }

        sendRecognize(peer.get());
        {
            std::lock_guard<std::mutex> lock(m_peerMutex);
            m_peerSockets.push_back(std::move(peer));
        }
    }
    catch (const std::exception& e)  // connect() threw -> ignore / retry later
    {
		auto string = std::string("Connect to peer failed: ") + e.what();
		Log::Info() << string << std::endl;
		return;
    }
}

void NetworkEngine::removePeer(TCPSocket* peerSocket) {
    {
        std::lock_guard<std::mutex> lk(m_peerMutex);
        m_peerInfoMap.erase(peerSocket->native());

        auto it = std::remove_if(
            m_peerSockets.begin(), m_peerSockets.end(),
            [&](auto& up) { return up.get() == peerSocket; }
        );
        m_peerSockets.erase(it, m_peerSockets.end());
    }
    // close the socket
    peerSocket->close();
}

void NetworkEngine::broadcastDiscovery(unsigned short discoveryPort) {
    if (!m_multicastSocket) {
		Log::Error() << "Multicast socket not initialized yet." << std::endl;
        return;
    }
    std::string discoveryMessage = constructDiscoveryMessage();
    m_multicastSocket->send(discoveryMessage.c_str(), static_cast<int>(discoveryMessage.size()));
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

void NetworkEngine::sendPing(TCPSocket* peerSocket) {
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

void NetworkEngine::sendPong(TCPSocket* peerSocket, const NetSim::Ping* ping) {
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

void NetworkEngine::sendRecognize(TCPSocket* peerSocket) {
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

void NetworkEngine::assignOwnersAndBroadcastScenarioCreate(const std::vector<std::shared_ptr<PhysicsObject>>& physicsObjects, const float& gravity,
    std::vector<std::shared_ptr<NetworkedObject>>& outNetworkedObjects) {
    flatbuffers::FlatBufferBuilder builder;

    std::vector<flatbuffers::Offset<NetSim::NetworkObjectCreation>> objectCreations;
    std::unordered_map<int, int> nonStaticObjects;

    int cnt = 0;
    for (int j = 0; j < physicsObjects.size(); j++) {
        if (!physicsObjects[j]->isStatic()) {
            nonStaticObjects[j] = cnt++;
        }
    }

    int distrib = ceil(static_cast<float>(nonStaticObjects.size()) / static_cast<float>(m_peerInfoMap.size() + 1));
    if (distrib == 0) distrib = 1;

    for (int i = 0; i < physicsObjects.size(); i++) {
        auto& phObj = physicsObjects[i];

        bool isStatic = phObj->isStatic();

        DirectX::XMFLOAT3 position;
        DirectX::XMStoreFloat3(&position, phObj->getTransform().GetPosition(0));

        DirectX::XMFLOAT4 rotation;
        DirectX::XMStoreFloat4(&rotation, phObj->getTransform().GetRotationQuaternion(0));

        DirectX::XMFLOAT3 scale;
        DirectX::XMStoreFloat3(&scale, phObj->getTransform().GetScale(0));

        auto physicsMaterial = phObj->getPhysicsMaterial();

        DirectX::XMFLOAT3 colliderSize;
        auto collider = phObj->getCollider();
        auto type = phObj->getMeshType();

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
        NetSim::PhysicsMaterial material = { physicsMaterial.friction, physicsMaterial.restitution };

        uint16_t peerIndex = 0;
        uint32_t peerId = GlobalData::g_clientId;
        NetSim::Vec3 colorVec3 = { GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z };

        if (!isStatic) {
            peerIndex = static_cast<uint16_t>(nonStaticObjects[i] / distrib);
            if (peerIndex > 0) {
                peerIndex--;
                peerId = m_peerInfoMap[m_peerSockets[peerIndex]->native()].peer_id;
                auto clientColor = m_peerInfoMap[m_peerSockets[peerIndex]->native()].color;
                colorVec3 = { clientColor.x, clientColor.y, clientColor.z };
			}
        }
        else {
            // Static objects are always white
            colorVec3 = { 1.0f, 1.0f, 1.0f };
        }

		auto ntObj = std::make_shared<NetworkedObject>(i, peerId, phObj);
		phObj->setUserData(ntObj.get());
        outNetworkedObjects.push_back(std::move(ntObj));

        phObj->setColor({ colorVec3.x(), colorVec3.y(), colorVec3.z(), 1.0f });
		auto physicsObjectCreation = NetSim::CreatePhysicsObjectCreation(builder, isStatic, static_cast<NetSim::MeshType>(type), phObj->getMass(), &material, &colliderSizeVec3, &positionVec3, &rotationVec4, &scaleVec3, &colorVec3, peerId);
        auto objectCreation = NetSim::CreateNetworkObjectCreation(builder, i, peerId, physicsObjectCreation);
        
        objectCreations.push_back(objectCreation);
    }

    auto objectsVector = builder.CreateVector(objectCreations);
    auto scenarioId = builder.CreateString("Scenario");
    auto scenario_offset = NetSim::CreateScenario(builder, GlobalData::g_clientId, objectsVector, gravity);
    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(scenarioId);
    msg_builder.add_data_type(NetSim::MessageUnion_Scenario);
    msg_builder.add_data(scenario_offset.Union());
    auto message = msg_builder.Finish();
    builder.Finish(message);
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (const auto& s : m_peerSockets) {
            sendMessage(s.get(), builder);
        }
    }

	Log::Info() << "Broadcasted scenario creation with " << objectCreations.size() << " objects." << std::endl;

}

void NetworkEngine::sendMessage(TCPSocket* peerSocket, flatbuffers::FlatBufferBuilder& builder) {
    uint32_t size = htonl(static_cast<uint32_t>(builder.GetSize()));
    peerSocket->sendAll(reinterpret_cast<const char*>(&size), sizeof(size));
    peerSocket->sendAll(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

TCPSocket* NetworkEngine::socketPtrFromHandle(SOCKET h)
{
    for (auto& sp : m_peerSockets)
        if (sp->native() == h) return sp.get();
    return nullptr;
}
double NetworkEngine::getPeerRTT(SOCKET peerSocket) const
{
    auto it = m_peerRTT.find(peerSocket);
    if (it != m_peerRTT.end())
        return it->second;
    return 0.0;
}

bool NetworkEngine::isPeerConnected(TCPSocket* peerSocket)
{
	char ipStr[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &peerSocket->getSockAddr()->sin_addr, ipStr, sizeof(ipStr));

    {
        std::lock_guard<std::mutex> lk(m_peerMutex);
        for (const auto& s : m_peerSockets) {

            char curIp[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &s->getSockAddr()->sin_addr, curIp, sizeof(curIp));

            if (strcmp(ipStr, curIp) == 0) {
                return true;
            }
        }
    }
	return false;
}

bool NetworkEngine::isPeerConnected(const std::string& ip, unsigned short port)
{
	for (const auto& s : m_peerSockets) {
		char curIp[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &s->getSockAddr()->sin_addr, curIp, sizeof(curIp));

		if (strcmp(ip.c_str(), curIp) == 0 && s->getSockAddr()->sin_port == port) {
			return true;
		}
	}
	return false;
}

bool NetworkEngine::isPeerConnected(const uint16_t peerId)
{
	for (const auto& [s, info] : m_peerInfoMap) {
		if (info.peer_id == peerId) {
			return true;
		}
	}
	return false;
}

void NetworkEngine::sendPeerList(TCPSocket* to) {
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

void NetworkEngine::sendObjectUpdatesToPeers() {
    Snapshot s;
    flatbuffers::FlatBufferBuilder b;

    while (m_outgoingBuffer->pop(s))
    {
        b.Clear();

        std::vector<NetSim::ObjectUpdate> fbUpdates;
        fbUpdates.reserve(s.updates.size());

        for (const auto& u : s.updates) {
            fbUpdates.emplace_back(
                u.object_id,
                NetSim::Vec3(u.position.x, u.position.y, u.position.z),
                NetSim::Vec4(u.rotation.x, u.rotation.y,
                    u.rotation.z, u.rotation.w),
                NetSim::Vec3(u.velocity.x, u.velocity.y, u.velocity.z),
                NetSim::Vec3(u.angular_velocity.x,
                    u.angular_velocity.y,
                    u.angular_velocity.z));
        }

        auto updatesVec = b.CreateVectorOfStructs<NetSim::ObjectUpdate>(fbUpdates);
        auto snapOffset = NetSim::CreateSnapshot(b,
            s.tick,
            updatesVec);

        auto typeStr = b.CreateString("Snapshot");
        NetSim::NetworkMessageBuilder msgBuilder(b);
        msgBuilder.add_msg_type(typeStr);
        msgBuilder.add_data_type(NetSim::MessageUnion::MessageUnion_Snapshot);
        msgBuilder.add_data(snapOffset.Union());
        auto netMsg = msgBuilder.Finish();
        b.Finish(netMsg);

        const uint8_t* buf = b.GetBufferPointer();
        const uint32_t len = b.GetSize();

        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (const auto& peerSock : m_peerSockets)
            sendMessage(peerSock.get(), b);
    }
}

void NetworkEngine::handleNewConnection()
{
    TCPSocket client = m_listenSocket->accept();
    auto sockAdd = client.getSockAddr();

    if (client.isValid()) {

        char ipStr[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sockAdd->sin_addr, ipStr, sizeof(ipStr));

        if (!isPeerConnected(&client)) {
            auto uptr = std::make_unique<TCPSocket>(std::move(client));
            m_peerSockets.push_back(std::move(uptr));
        }

    }
}

void NetworkEngine::handlePeerData(TCPSocket* peerSocket) {
    uint32_t netSize = 0;
    int got = peerSocket->recv(&netSize, sizeof(netSize));
    if (got == 0) {
        removePeer(peerSocket);
        return;
    }
	else if (got < 0) {
		return;
	}

    uint32_t size = ntohl(netSize);
    std::vector<char> buf(size);
    got = peerSocket->recv(buf.data(), buf.size());
    if (got == 0) {
        removePeer(peerSocket);
        return;
    }
	else if (got < 0) {
		return;
	}

    auto message = NetSim::GetNetworkMessage(buf.data());
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
    case NetSim::MessageUnion_Snapshot:
        handleSnapshot(peerSocket, message->data_as_Snapshot());
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
    case:: NetSim::MessageUnion_StopSimulation:
        handleStopSimulation(message->data_as_StopSimulation());
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
        if (!isPeerConnected(ip, port)) {
            connectToPeer(ip, port);
        }
    }
}

void NetworkEngine::handlePing(TCPSocket* from, const NetSim::Ping* ping) {
    sendPong(from, ping);
}

void NetworkEngine::handlePong(TCPSocket* from, const NetSim::Pong* pong) {
    auto it = m_sentPingTimestamps.find(pong->ping_sent_time());
    if (it == m_sentPingTimestamps.end()) return; // Old pong

    double T_send = it->second;
    double T_recv = GlobalData::getTimestamp();
    double T_remote = pong->remote_time();

    double rtt = T_recv - T_send;
    double offset = T_remote - (T_send + rtt * 0.5);
	m_peerRTT[from->native()] = rtt;
    m_peerClockOffsets[from->native()] = offset;

    m_sentPingTimestamps.erase(it);
}

void NetworkEngine::handleRecognize(const NetSim::Recognize* recognize) {
    if (!recognize) return;

    if (recognize->protocol_version() != PROTOCOL_VERSION) {
        return;
    }

    TCPSocket* senderSocket = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        for (const auto& s : m_peerSockets) {
            if (m_peerInfoMap.find(s->native()) == m_peerInfoMap.end()) {
                senderSocket = s.get();
                break;
            }
        }
    }

    if (!senderSocket) {
        return;
    }

    // Store peer info
    PeerInfo info;
    info.socket = senderSocket->native();
    info.peer_id = recognize->peer_id();
    info.client_name = recognize->client_name()->str();
    info.port = recognize->listen_port();
	info.color = { recognize->color()->x(), recognize->color()->y(), recognize->color()->z() };

    sockaddr_in addr;
    int len = sizeof(addr);
    getpeername(senderSocket->native(), (sockaddr*)&addr, &len);

    char ipBuffer[INET_ADDRSTRLEN];
    inet_ntop(AF_INET, &addr.sin_addr, ipBuffer, sizeof(ipBuffer));
    info.ip = ipBuffer;
    {
        std::lock_guard<std::mutex> lock(m_peerMutex);
        m_peerInfoMap[senderSocket->native()] = info;
    }

    sendRecognize(senderSocket);
    sendPeerList(senderSocket);
}

void NetworkEngine::handleScenario(const NetSim::Scenario* scenario) {
	if (!scenario) return;

    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_DIRECT);
    auto commandList = commandQueue->GetCommandList();

	std::vector<std::shared_ptr<NetworkedObject>> objects;

	for (const auto* object : *scenario->objects()) {
		uint16_t objectId = object->id();
		uint16_t ownerId = object->owner_id();
        auto objCreation = object->object_creation();
        
		auto objectType = static_cast<MeshType>(objCreation->type());
		DirectX::XMFLOAT3 colliderSize = { objCreation->collider_size()->x(), objCreation->collider_size()->y(), objCreation->collider_size()->z() };
		DirectX::XMFLOAT3 position = { objCreation->position()->x(), objCreation->position()->y(), objCreation->position()->z() };
		DirectX::XMVECTOR rotation = DirectX::XMVectorSet(objCreation->rotation()->x(), objCreation->rotation()->y(), objCreation->rotation()->z(), objCreation->rotation()->w());
		DirectX::XMVECTOR scale = DirectX::XMVectorSet(objCreation->scale()->x(), objCreation->scale()->y(), objCreation->scale()->z(), 1);
		DirectX::XMFLOAT4 color = { objCreation->color()->x(), objCreation->color()->y(), objCreation->color()->z(), 1 };

        std::shared_ptr<Mesh> objectMesh;
        std::shared_ptr<Collider> collider;
        std::shared_ptr<Texture> texture = objCreation->is_static() ? GlobalData::g_staticTexture : GlobalData::g_customTexture;
        switch (objectType) {
		case MeshType::Sphere:
			objectMesh = GlobalData::g_sphereMesh;
			collider = std::make_shared<SphereCollider>(colliderSize.x);
			break;
		case MeshType::Box:
			objectMesh = GlobalData::g_boxMesh;
			collider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(colliderSize.x, colliderSize.y, colliderSize.z, 0));
			break;
		case MeshType::Capsule:
            objectMesh = GlobalData::createCapsuleMesh(*commandList, colliderSize.x, colliderSize.y, 16);
			collider = std::make_shared<CapsuleCollider>(colliderSize.x, colliderSize.y);
			break;
		case MeshType::Plane:
			objectMesh = GlobalData::g_planeMesh;
			collider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(colliderSize.x, colliderSize.y, colliderSize.z, 0));
            texture = GlobalData::g_defaultTexture;
			break;
		default:
			throw std::runtime_error("Unknown object type");
        }

		auto physicsObject = std::make_shared<PhysicsObject>(objectType, objectMesh.get(), texture.get());
		physicsObject->setCollider(collider);
		physicsObject->setColor(color);
		physicsObject->getTransform().SetPosition(position, 0, true);
		physicsObject->getTransform().SetRotationQuaternion(rotation, 0, true);
		physicsObject->getTransform().SetScale(scale, 0, true);
		physicsObject->setStatic(objCreation->is_static());
        physicsObject->setMass(objCreation->mass());
        physicsObject->setPhysicsMaterial({ objCreation->material()->friction(), objCreation->material()->restitution() });

		auto networkedObject = std::make_shared<NetworkedObject>(objectId, ownerId, physicsObject);
		physicsObject->setUserData(networkedObject.get());

		objects.push_back(networkedObject);
	}

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

	// Notify the scenario listener
	if (m_createScenario) {
		m_createScenario(std::move(objects), scenario->gravity());
	}
}

void NetworkEngine::handleSnapshot(TCPSocket* from, const NetSim::Snapshot* objectUpdateList) {
	if (!objectUpdateList) return;
    {
		Snapshot snapshot;
		snapshot.tick = objectUpdateList->tick();
		for (const auto* update : *objectUpdateList->updates()) {
			ObjectUpdate objUpdate;
			objUpdate.object_id = update->object_id();
			objUpdate.position = { update->position().x(), update->position().y(), update->position().z() };
			objUpdate.rotation = { update->rotation().x(), update->rotation().y(), update->rotation().z(), update->rotation().w() };
			objUpdate.velocity = { update->velocity().x(), update->velocity().y(), update->velocity().z() };
			objUpdate.angular_velocity = { update->angular_velocity().x(), update->angular_velocity().y(), update->angular_velocity().z() };
			snapshot.updates.push_back(objUpdate);
		}
		m_incomingBuffer->push(snapshot);
    }
}

void NetworkEngine::handleDiscoveryDatagrams()
{
    char buffer[1024];
    sockaddr_in senderAddr{};
    int bytesReceived = 0;

    do {
        bytesReceived = m_multicastSocket->receive(buffer, sizeof(buffer), &senderAddr);

        if (bytesReceived <= 0)
        {
            if (WSAGetLastError() == WSAEWOULDBLOCK)
                break;
            else
                return; // real error – silently drop
        }

        const NetSim::NetworkMessage* message = NetSim::GetNetworkMessage(buffer);
        if (message->data_type() != NetSim::MessageUnion_DiscoveryBroadcast)
            continue;

        const auto* discovery = message->data_as_DiscoveryBroadcast();

        if (discovery->peer_id() == GlobalData::g_clientId)
            continue;
        if (discovery->protocol_version() != PROTOCOL_VERSION)
            continue;

		if(!isPeerConnected(discovery->peer_id()))
        {
            char ipStr[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &senderAddr.sin_addr, ipStr, sizeof(ipStr));
            connectToPeer(ipStr, discovery->tcp_port());
        }


    } while (bytesReceived > 0);
}


void NetworkEngine::handleStartSimulation(TCPSocket* peerSocket, const NetSim::StartSimulation* startSim) {
    double remoteTime = startSim->simulation_start_time();
    double offset = m_peerClockOffsets[peerSocket->native()];

    double localStartTime = remoteTime - offset;
    m_startSimulation(localStartTime);
}

void NetworkEngine::handleGravityChange(const NetSim::GravityChange* gravityChange) {
	if (!gravityChange) return;
	float newGravity = gravityChange->new_value();
	PhysicsEngine::setGravity(newGravity);
}

void NetworkEngine::handleStopSimulation(const NetSim::StopSimulation* stopSim) {
	if (!stopSim) return;
	m_stopSimulation();

}

void NetworkEngine::onUpdate(float deltaTime) {
    static double lastBroadcast = 0.0;
    double now = GlobalData::getTimestamp();
    if (now - lastBroadcast > 2.0)
    {
        broadcastDiscovery(GlobalData::g_listenPort);
        lastBroadcast = now;
    }

    static double lastPingTime = 0.0;
    if (now - lastPingTime > 5.0) {
        {
            std::lock_guard<std::mutex> lock(m_peerMutex);
            for (const auto& s : m_peerSockets) {
                sendPing(s.get());
            }
        }
        lastPingTime = now;
    }

    fd_set readSet;
    FD_ZERO(&readSet);
    SOCKET maxSock = 0;

    auto listenSock = m_listenSocket->native();
    FD_SET(listenSock, &readSet);
    maxSock = listenSock;

    {
        std::lock_guard<std::mutex> lk(m_peerMutex);
        for (auto& peerUP : m_peerSockets) {
            SOCKET s = peerUP->native();
            FD_SET(s, &readSet);
            if (s > maxSock) maxSock = s;
        }
    }

    SOCKET multiSock = m_multicastSocket->native();
    FD_SET(multiSock, &readSet);
    if (multiSock > maxSock) maxSock = multiSock;

    timeval tv{ 0, 0 };
    int ready = select(int(maxSock + 1), &readSet, nullptr, nullptr, &tv);
    if (ready > 0) {
        if (FD_ISSET(listenSock, &readSet)) {
			handleNewConnection();
        }

        if (FD_ISSET(multiSock, &readSet))
        {
            handleDiscoveryDatagrams();
        }

        std::vector<TCPSocket*> readyPeers;
        {
            std::lock_guard<std::mutex> lk(m_peerMutex);
            for (auto& peerUP : m_peerSockets) {
                if (FD_ISSET(peerUP->native(), &readSet)) {
                    readyPeers.push_back(peerUP.get());
                }
            }
        }
        for (auto* peer : readyPeers) {
            handlePeerData(peer);
        }
    }
    sendObjectUpdatesToPeers();
}

void NetworkEngine::onStop() noexcept {
    for (auto& s : m_peerSockets) s->shutdown();
    m_listenSocket->shutdown();
    WSACleanup();
}

void NetworkEngine::onStart() noexcept {}
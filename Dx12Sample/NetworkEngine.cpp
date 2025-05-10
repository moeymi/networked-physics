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

NetworkEngine::NetworkEngine()
    : m_listenSocket(INVALID_SOCKET)
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

void NetworkEngine::setScnearioListener(std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&)> listener) {
	m_createScenario = listener;
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
}

void NetworkEngine::sendScenarioCreate(SOCKET peerSocket, const std::vector<std::shared_ptr<PhysicsObject>>& physicsObjects) {
    flatbuffers::FlatBufferBuilder builder;
    std::vector<flatbuffers::Offset<NetSim::ObjectState>> objectStates;

    std::unordered_map<int, int> nonStaticObjects;
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
        NetSim::Vec3 colorVec3 = { GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z };

        if (!object->isStatic()) {
			int distrib = ceil(static_cast<float>(nonStaticObjects.size()) / static_cast<float>(m_peerSockets.size() + 1));
			if (distrib == 0) distrib = 1;
			peerIndex = static_cast<uint16_t>(nonStaticObjects[i] / distrib);
            if (peerIndex > 0) {
                peerIndex--;
                auto clientColor = m_peerInfoMap[m_peerSockets[peerIndex]].color;
                colorVec3 = { clientColor.x, clientColor.y, clientColor.z };
            }
        }
        else {
            // Static objects are always white
            colorVec3 = { 1.0f, 1.0f, 1.0f };
        }
		bool isStatic = object->isStatic();
		object->setColor({ colorVec3.x(), colorVec3.y(), colorVec3.z(), 1.0f});
        auto objectState = NetSim::CreateObjectState(builder, object->getId(), &isStatic, static_cast<NetSim::MeshType>(type), &colliderSizeVec3, &positionVec3, &rotationVec4, &scaleVec3, &colorVec3, 0);
        objectStates.push_back(objectState);
    }

    auto objectsVector = builder.CreateVector(objectStates);
    auto scenarioId = builder.CreateString("Scenario");
    auto scenario_offset = NetSim::CreateScenario(builder, GlobalData::g_clientId, objectsVector);
    NetSim::NetworkMessageBuilder msg_builder(builder);
    msg_builder.add_msg_type(scenarioId);
    msg_builder.add_data_type(NetSim::MessageUnion_Scenario);
    msg_builder.add_data(scenario_offset.Union());
    auto message = msg_builder.Finish();
    builder.Finish(message);
    sendMessage(peerSocket, builder);
}

void NetworkEngine::broadcastScenarioCreate(std::string scenarioName, const std::vector<std::shared_ptr<PhysicsObject>>& objects) {
    std::lock_guard<std::mutex> lock(m_peerMutex);
    for (SOCKET s : m_peerSockets) {
        sendScenarioCreate(s, objects);
    }
}

void NetworkEngine::sendMessage(SOCKET peerSocket, flatbuffers::FlatBufferBuilder& builder) {
    uint32_t size = htonl(static_cast<uint32_t>(builder.GetSize()));
    send(peerSocket, reinterpret_cast<const char*>(&size), sizeof(size), 0);
    send(peerSocket, reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize(), 0);
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
    case NetSim::MessageUnion_ObjectUpdate:
        //handleObjectUpdate(message->data_as_ObjectUpdate());
        break;
        // Handle other message types as needed
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

		objects.push_back(physicsObject);
	}

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

	// Notify the scenario listener
	if (m_createScenario) {
		m_createScenario(std::move(objects));
	}
}

void NetworkEngine::onUpdate(float deltaTime) {
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
}

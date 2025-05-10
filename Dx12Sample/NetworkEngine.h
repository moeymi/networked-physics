#pragma once
#include "pch.h"
#include <mutex>
#include <thread>
#include "ThreadedSystem.h"
#include "game_state_generated.h"
#include "Scenario.h"

struct PeerInfo {
    SOCKET socket;
    uint32_t peer_id;
    std::string client_name;
    std::string ip;
    uint16_t port;
    DirectX::XMFLOAT3 color;
};

class NetworkEngine : public ThreadedSystem {

private:
    const uint32_t PROTOCOL_VERSION = 1;

    void handleNewConnection();
    void handlePeerData(SOCKET peerSocket);

    SOCKET m_listenSocket;
    std::vector<SOCKET> m_peerSockets;
    std::mutex m_peerMutex;
    std::thread m_networkThread;

    void removePeer(SOCKET peerSocket);
    void sendRecognize(SOCKET peerSocket);
    void sendMessage(SOCKET peerSocket, flatbuffers::FlatBufferBuilder& builder);
    void sendPeerList(SOCKET peerSocket);
    void sendScenarioCreate(SOCKET peerSocket, const std::vector<std::shared_ptr<PhysicsObject>>& objects);

    void handleRecognize(const NetSim::Recognize* recognize);
    void handlePeerList(const NetSim::PeerList* list);
	void handleScenario(const NetSim::Scenario* scenario);

    std::unordered_map<SOCKET, PeerInfo> m_peerInfoMap;
	std::unordered_map<uint16_t, Material> m_materialMap;
    std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&)> m_createScenario;

public:
    NetworkEngine();
    ~NetworkEngine();

    float receivedTime = 0.0f;

    void initializeSockets(unsigned short listenPort);
    void connectToPeer(const std::string& ip, unsigned short port);
	void broadcastScenarioCreate(std::string scenarioName, const std::vector<std::shared_ptr<PhysicsObject>>& objects);

	void setScnearioListener(std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&)> listener);

    std::vector<PeerInfo> getPeersInfo() const;

protected:
	void onUpdate(float deltaTime) override;
};
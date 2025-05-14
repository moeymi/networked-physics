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

struct ObjectUpdate
{
    uint32_t object_id;
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT4 rotation;
    DirectX::XMFLOAT3 velocity;
    DirectX::XMFLOAT3 angular_velocity;
};

struct SharedData {
    std::vector<ObjectUpdate> m_outgoingObjectStates[2];
    std::vector<ObjectUpdate> m_incomingObjectStates[2];

	std::mutex m_outgoingMutex;
	std::mutex m_incomingMutex;

    bool m_ownedObjectsDirty = false;
    bool m_unownedObjectsDirty = false;
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

    std::string constructDiscoveryMessage();

    void removePeer(SOCKET peerSocket);

	void broadcastDiscovery(unsigned short port);
	void listenForDiscovery(unsigned short port);
    void connectToPeer(const std::string& ip, unsigned short port);

	void sendPing(SOCKET peerSocket);
	void sendPong(SOCKET peerSocket, const NetSim::Ping* ping);
    void sendRecognize(SOCKET peerSocket);
    void sendMessage(SOCKET peerSocket, flatbuffers::FlatBufferBuilder& builder);
    void sendPeerList(SOCKET peerSocket);
    void sendObjectUpdatesToPeers(const std::vector<ObjectUpdate>& updates);

    void handlePing(SOCKET from, const NetSim::Ping* ping);
    void handlePong(SOCKET from, const NetSim::Pong* pong);
    void handleRecognize(const NetSim::Recognize* recognize);
    void handlePeerList(const NetSim::PeerList* list);
	void handleScenario(const NetSim::Scenario* scenario);
	void handleObjectUpdate(const NetSim::ObjectUpdateList* objectUpdateList);
    void handleStartSimulation(SOCKET peerSocket, const NetSim::StartSimulation* startSim);

	void cleanDirtyOutgoingObjects();

    std::unordered_map<SOCKET, PeerInfo> m_peerInfoMap;
	std::unordered_map<uint16_t, Material> m_materialMap;
    std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&, const float&)> m_createScenario;
	std::function<void(double)> m_startSimulation;

	std::unique_ptr<SharedData> m_sharedData;
    std::unordered_map<SOCKET, double> m_peerClockOffsets;
    std::unordered_map<double, double> m_sentPingTimestamps;

public:
    NetworkEngine();
    ~NetworkEngine();

    float receivedTime = 0.0f;

    void initializeSockets(unsigned short listenPort);
	void assignOwnersAndBroadcastScenarioCreate(std::string scenarioName, const std::vector<std::shared_ptr<PhysicsObject>>& objects, const float& gravity,
        std::vector<PhysicsObject*>& ownedObjects, PhysicsObject** unownedObjects);

	void setScnearioListener(std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&, const float&)> listener);
	void setStartSimulationListener(std::function<void(double)> listener);
	void scheduleSimulationStart(float time);

    std::vector<PeerInfo> getPeersInfo() const;

	SharedData* getSharedData()
	{
		return m_sharedData.get();
	}

protected:
	void onUpdate(float deltaTime) override;
    virtual void onStop() override;
    virtual void onStart() override;
};
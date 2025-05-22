#pragma once
#include "pch.h"
#include <mutex>
#include <thread>
#include "ThreadedSystem.h"
#include "game_state_generated.h"
#include "Scenario.h"
#include "MulticastSocket.h"
#include "TCPSocket.h"

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
	double simulation_time;
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT4 rotation;
    DirectX::XMFLOAT3 velocity;
    DirectX::XMFLOAT3 angular_velocity;
};

struct SharedData {
    // Outgoing (owned object state updates) remain double buffered
    std::vector<ObjectUpdate> m_outgoingObjectStates[2];
    std::mutex m_outgoingMutex;
    bool m_ownedObjectsDirty = false;

    // History of incoming updates per object ID
    std::unordered_map<uint32_t, std::deque<ObjectUpdate>> m_objectUpdateHistory;
    std::mutex m_incomingMutex;

    // A flag for triggering reconciliation logic
    bool m_receivedNewAuthoritativeData = false;

    // Control max buffer duration (in seconds)
    double maxHistoryDuration = 0.5; // Keep only last 500ms of updates
};


class NetworkEngine : public ThreadedSystem {

private:
    const uint32_t PROTOCOL_VERSION = 1;

    void handleNewConnection();
    void handlePeerData(TCPSocket* peerSocket);

    std::unique_ptr<TCPSocket> m_listenSocket;
    std::vector<std::unique_ptr<TCPSocket>> m_peerSockets;
    std::mutex m_peerMutex;
    std::thread m_networkThread;

	std::unique_ptr<MulticastSocket> m_multicastSocket;

    std::string constructDiscoveryMessage();

    void removePeer(TCPSocket* peerSocket);

	void broadcastDiscovery(unsigned short port);
    void connectToPeer(const std::string& ip, unsigned short port);

	void sendPing(TCPSocket* peerSocket);
	void sendPong(TCPSocket* peerSocket, const NetSim::Ping* ping);
    void sendRecognize(TCPSocket* peerSocket);
    void sendMessage(TCPSocket* peerSocket, flatbuffers::FlatBufferBuilder& builder);
    void sendPeerList(TCPSocket* peerSocket);
    void sendObjectUpdatesToPeers(const std::vector<ObjectUpdate>& updates);

    void handlePing(TCPSocket* from, const NetSim::Ping* ping);
    void handlePong(TCPSocket* from, const NetSim::Pong* pong);
    void handleRecognize(const NetSim::Recognize* recognize);
    void handlePeerList(const NetSim::PeerList* list);
	void handleScenario(const NetSim::Scenario* scenario);
	void handleGravityChange(const NetSim::GravityChange* gravityChange);
	void handleObjectUpdate(TCPSocket* from, const NetSim::ObjectUpdateList* objectUpdateList);
    void handleStartSimulation(TCPSocket* peerSocket, const NetSim::StartSimulation* startSim);
    void handleDiscoveryDatagrams();
	void cleanDirtyOutgoingObjects();
    TCPSocket* socketPtrFromHandle(SOCKET h);
    double getPeerRTT(SOCKET peerSocket) const;
	bool isPeerConnected(TCPSocket* peerSocket);
	bool isPeerConnected(const std::string& ip, unsigned short port);
	bool isPeerConnected(const uint16_t peerId);

    std::unordered_map<SOCKET, PeerInfo> m_peerInfoMap;
	std::unordered_map<uint16_t, Material> m_materialMap;
    std::function<void(std::vector<std::shared_ptr<PhysicsObject>>&&, const float&)> m_createScenario;
	std::function<void(double)> m_startSimulation;

	std::unique_ptr<SharedData> m_sharedData;
    std::unordered_map<SOCKET, double> m_peerClockOffsets;
    std::unordered_map<SOCKET, double> m_peerRTT;
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
	void changeGravity(const float& gravity);

    std::vector<std::tuple<PeerInfo, double>> getPeersInfo() const;

	SharedData* getSharedData()
	{
		return m_sharedData.get();
	}

protected:
	void onUpdate(float deltaTime) override;
    virtual void onStop() override;
    virtual void onStart() override;
};
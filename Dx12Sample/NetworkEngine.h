#pragma once
#include "pch.h"
#include "ThreadedSystem.h"
#include "game_state_generated.h"
#include "Scenario.h"
#include "MulticastSocket.h"
#include "TCPSocket.h"
#include "NetworkedObject.h"
#include "RingBufferSPSC.h"

#include <mutex>
#include <thread>

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
    void handlePeerData(TCPSocket* peerSocket);

    std::unique_ptr<TCPSocket> m_listenSocket;
    std::vector<std::unique_ptr<TCPSocket>> m_peerSockets;
    std::mutex m_peerMutex;

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
    void sendObjectUpdatesToPeers();

    void handlePing(TCPSocket* from, const NetSim::Ping* ping);
    void handlePong(TCPSocket* from, const NetSim::Pong* pong);
    void handleRecognize(const NetSim::Recognize* recognize);
    void handlePeerList(const NetSim::PeerList* list);
	void handleScenario(const NetSim::Scenario* scenario);
	void handleGravityChange(const NetSim::GravityChange* gravityChange);
	void handleSnapshot(TCPSocket* from, const NetSim::Snapshot* objectUpdateList);
    void handleStartSimulation(TCPSocket* peerSocket, const NetSim::StartSimulation* startSim);
    void handleDiscoveryDatagrams();
    TCPSocket* socketPtrFromHandle(SOCKET h);
    double getPeerRTT(SOCKET peerSocket) const;
	bool isPeerConnected(TCPSocket* peerSocket);
	bool isPeerConnected(const std::string& ip, unsigned short port);
	bool isPeerConnected(const uint16_t peerId);

    std::unordered_map<SOCKET, PeerInfo> m_peerInfoMap;
    std::function<void(std::vector<std::shared_ptr<NetworkedObject>>&&, const float&)> m_createScenario;
	std::function<void(double)> m_startSimulation;

    std::unordered_map<SOCKET, double> m_peerClockOffsets;
    std::unordered_map<SOCKET, double> m_peerRTT;
    std::unordered_map<double, double> m_sentPingTimestamps;

	RingBufferSPSC<Snapshot, 1024>* m_outgoingBuffer;
	RingBufferSPSC<Snapshot, 1024>* m_incomingBuffer;

public:
	NetworkEngine(RingBufferSPSC<Snapshot, 1024>*, RingBufferSPSC<Snapshot, 1024>*);
    ~NetworkEngine();

    float receivedTime = 0.0f;

    void initializeSockets(unsigned short listenPort);
	void assignOwnersAndBroadcastScenarioCreate(const std::vector<std::shared_ptr<PhysicsObject>>& phObjects, const float& gravity,
        std::vector<std::shared_ptr<NetworkedObject>>& outObjects);

	void setScnearioListener(std::function<void(std::vector<std::shared_ptr<NetworkedObject>>&&, const float&)> listener);
	void setStartSimulationListener(std::function<void(double)> listener);
	void scheduleSimulationStart(float time);
	void changeGravity(const float& gravity);

    std::vector<std::tuple<PeerInfo, double>> getPeersInfo() const;

protected:
	void onUpdate(float deltaTime) override;
    virtual void onStop() override;
    virtual void onStart() override;
};
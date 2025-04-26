#pragma once
#include "pch.h"
#include "ThreadedSystem.h"
#include <vector>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include <string>
#include <memory>
#include <algorithm>

#include <DirectXMath.h>

struct CPPGameState {
    int id;
	int type;
    DirectX::XMFLOAT3 position;
    DirectX::XMFLOAT4 rotation;
    DirectX::XMFLOAT3 velocity;
    DirectX::XMFLOAT3 angularVelocity;
    DirectX::XMFLOAT4 color;
};

struct SharedSimulationData {
    std::vector<CPPGameState> objects;
    std::mutex mutex; // Mutex to protect 'objects'
    // Potentially add other shared data like global physics parameters, etc.
};

enum class NetMessageType : uint32_t {
    Unknown = 0,
    InitialState = 1, // Full state sent on connect
    StateUpdate = 2,  // Periodic state delta/snapshot
    // Add more message types later (e.g., ClientInput, JoinRequest, DisconnectNotification)
};

#pragma pack(push, 1) // Ensure no padding for network struct
struct NetMessageHeader {
    NetMessageType type;
    uint32_t payloadSize; // Size of the data following the header
};
#pragma pack(pop) // Restore default padding

struct ClientConnection {
    SOCKET socket = INVALID_SOCKET;
    sockaddr_in address = {}; // Client address info
    // Add other state like last sent timestamp if needed
};


class NetworkEngine : public ThreadedSystem {
public:
    enum class Role {
        None, // Initial state - not hosting or connected
        Host,
        Client
    };

    NetworkEngine();
    ~NetworkEngine() override;

    // --- Public methods to start/join sessions ---
    // Call one of these based on external input (e.g., button press)
    // Returns true on success, false on failure (already initialized, network error, etc.)
    bool startHostSession(int port, SharedSimulationData* sharedData);
    bool joinClientSession(const char* hostAddress, int port, SharedSimulationData* sharedData);

    // Stop the current session and return to Role::None
    void stopSession();

    Role getRole() const { return m_role; }
    int getPort() const { return m_port; } // Port used for hosting or connecting
    const std::string& getHostAddress() const { return m_hostAddress; } // Host address for client role


protected:
    // --- Inherited from ThreadedSystem ---
    // This method's implementation will now contain logic for BOTH Host and Client roles.
    void onUpdate(float deltaTime) override;

    // --- Networking Specific Methods and Members ---
    // (Declare internal helper methods and member variables as before)

    bool initializeWinsock(); // Unchanged
    void cleanupWinsock();   // Unchanged

    // FlatBuffers methods (Unchanged)
    std::vector<uint8_t> buildStateUpdatePayload(const std::vector<CPPGameState>& objects);
    bool applyStateUpdatePayload(const std::vector<uint8_t>& payload, std::vector<CPPGameState>& outObjects);

    // Message helpers (Unchanged)
    bool sendMessage(SOCKET targetSocket, NetMessageType type, const std::vector<uint8_t>& payload);
    bool receiveMessage(SOCKET sourceSocket, NetMessageType& outType, std::vector<uint8_t>& outPayload);

    // Thread functions (Called internally, implementations largely unchanged)
    void runListeningThread(); // Host only
    void runReceivingThread(); // Client only

    // --- Member Variables (Same as before) ---
    Role m_role = Role::None;
    SharedSimulationData* m_sharedSimulationData = nullptr;

    // Host Specific
    SOCKET m_listenSocket = INVALID_SOCKET;
    std::thread m_listeningThread;
    std::atomic<bool> m_listening = { false };
    std::vector<ClientConnection> m_connectedClients;
    std::mutex m_clientsMutex;

    // Client Specific
    SOCKET m_hostSocket = INVALID_SOCKET;
    std::thread m_receivingThread;
    std::atomic<bool> m_receiving = { false };

    // Configuration
    int m_port = 0;
    std::string m_hostAddress;

    // Constants
    static const int MAX_PAYLOAD_SIZE = 4 * 1024 * 1024;
};
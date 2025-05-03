#pragma once
#include "pch.h"
#include "ThreadedSystem.h"
#include "NetworkMessages.h"
#include "game_state_generated.h"


// --- Helper Struct for Per-Socket Receive Buffers ---
struct SocketReceiveBuffer {
    SOCKET socket;
    std::vector<char> buffer; // Buffer to accumulate incoming data
    // Add state for partial message processing if needed (e.g., expected size)
};

// --- Helper Struct for Per-Socket Send Buffers ---
struct SocketSendBuffer {
    SOCKET socket;
    std::vector<char> buffer; // Data to send
    size_t bytesSent = 0;    // How many bytes have been sent from this buffer
};

// --- Helper Struct for Outgoing Messages (Queued internally by NetworkSystem) ---
// This represents a complete FlatBuffers message ready to be sent to all peers.
struct OutgoingNetMessage {
    std::vector<char> flatbufferData; // The serialized FlatBuffers message
};


// --- NetworkSystem Class Definition ---

class NetworkEngine : public ThreadedSystem {
private:
    // --- Network Specific Members ---

    SOCKET m_listenSocket = INVALID_SOCKET; // Socket for accepting incoming connections
    std::vector<SOCKET> m_connectedPeerSockets; // List of active connections (outgoing and incoming)
    std::mutex m_connectedPeerSocketsMutex; // Protects access to m_connectedPeerSockets

    std::string m_listenPort; // Port this peer listens on
    std::vector<std::string> m_peerAddressesToConnect; // Addresses of peers this instance should connect to

    // Timer for throttling outgoing state updates
    std::chrono::steady_clock::time_point m_lastStateSendTime;

    // Internal queues and buffers for managing network data
    std::map<SOCKET, SocketReceiveBuffer> m_receiveBuffers; // Per-socket receive buffers
    std::mutex m_receiveBuffersMutex; // Protects access to m_receiveBuffers

    std::map<SOCKET, SocketSendBuffer> m_sendBuffers; // Per-socket send buffers
    std::mutex m_sendBuffersMutex; // Protects access to m_sendBuffers

    std::queue<OutgoingNetMessage> m_outgoingNetMessages; // Internal queue of serialized messages to send to all peers
    std::mutex m_outgoingNetMessagesMutex; // Protects access to m_outgoingNetMessages


    // --- Internal Helper Methods (Called by run()) ---

    // Initializes Winsock, creates and binds listen socket, attempts initial outgoing connections
    bool initializeNetworking();
    // Cleans up sockets and Winsock
    void shutdownNetworking();

    // Checks for and accepts new connections on the listen socket
    void handleIncomingConnections();
    // Checks a specific socket for incoming data, buffers it, and processes complete messages
    // Returns true if the socket should be closed.
    bool handleDataReceive(SOCKET sock);

    // Gathers StateUpdate messages from the physics queue and serializes them into a FlatBuffers NetMessage
    void gatherAndSerializeStateUpdates();
    // Takes serialized messages from the internal outgoing queue and distributes them to per-socket send buffers
    void distributeOutgoingMessages();
    // Attempts to send pending data from a specific socket's send buffer
    // Returns true if the socket should be closed.
    bool processSocketSend(SOCKET sock);

    // Helper to attempt connecting to a single peer address
    void connectToPeer(const std::string& peerAddrStr);

    // Helper to close and cleanup a specific socket
    void closeSocket(SOCKET sock);


protected:
    // --- Overrides from ThreadedSystem ---

    // NOTE: The base class's run() implementation, which calls onUpdate based on
    // a fixed time step, is completely overridden in THIS class's run() method.
    // Therefore, this onUpdate method will NOT be called by the base class logic.
    // It must be implemented because it's pure virtual in the base, but it will
    // likely be empty or contain logic specific to shutdown if needed.
    void onUpdate(float deltaTime) override; // Dummy implementation

    // --- Override the base run method entirely ---
    // This is the main loop for the NetworkSystem thread.
    void run() override;


public:
    // --- Public Interface ---

    // Constructor: Takes configuration for the network system
    NetworkEngine(const std::string& listenPort, const std::vector<std::string>& peerAddresses);

    // Destructor: Ensures networking is shut down
    virtual ~NetworkEngine();

    // Methods to send specific message types to all connected peers
    void sendObjectCreation(
        uint32_t id, int object_type,
        const PhysicsFlatBuffers::Networking::Vec3& initial_position,
        const PhysicsFlatBuffers::Networking::Vec4& initial_rotation,
        const PhysicsFlatBuffers::Networking::Vec3& initial_scale,
        const PhysicsFlatBuffers::Networking::Vec3& initial_velocity,
        const PhysicsFlatBuffers::Networking::Vec3& initial_angular_velocity,
        bool is_static,
        float sphere_radius, const PhysicsFlatBuffers::Networking::Vec3& box_size,
        float cylinder_radius, float cylinder_height, const PhysicsFlatBuffers::Networking::Vec3& plane_size
    );

    void sendPeerJoined(uint32_t peer_id);
    void sendPeerLeft(uint32_t peer_id);
    void sendGravityChanged(const PhysicsFlatBuffers::Networking::Vec3& new_gravity);

    // Inherited methods from ThreadedSystem:
    // void start(); // Starts the network thread by calling our run()
    // void stop();  // Signals the thread to stop and waits for it to join
    // bool isRunning() const;
    // void setAffinity(const int& coreId);
    // int getAffinity() const;
    // Note: setFixedTimeStep, getFixedTimeStep, getRealTimeStep, addUpdateListener, removeUpdateListeners
    // are inherited but effectively unused by the NetworkSystem's custom run() loop.


private:
    // Disable copy and assignment to prevent issues with socket handles
    NetworkEngine(const NetworkEngine&) = delete;
    NetworkEngine& operator=(const NetworkEngine&) = delete;
};

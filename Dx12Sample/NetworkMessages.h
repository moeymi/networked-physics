#pragma once

#include <cstdint>
#include <vector>
#include <variant> // Requires C++17
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>

// Include the FlatBuffers generated header
#include "game_state_generated.h"

// Forward declare SimulationSettings if it's defined elsewhere
struct SimulationSettings;

// --- Internal Structs for Queues ---
// These structs represent the data payload of different message types
// as they are passed internally within the application (e.g., in queues).

// Struct used in the queue from Physics to Network
// Holds the state data for a single object that needs to be sent.
struct OutgoingStateUpdate {
    uint32_t objectId;
    // Copy the relevant data from the GameObjectState FlatBuffer struct
    PhysicsFlatBuffers::Networking::Vec3 position;
    PhysicsFlatBuffers::Networking::Vec4 rotation;
    PhysicsFlatBuffers::Networking::Vec3 velocity;
    PhysicsFlatBuffers::Networking::Vec3 angular_velocity;
    PhysicsFlatBuffers::Networking::Vec4 color;
    // Add other state fields as needed from your .fbs

    // Constructor for easy creation
    OutgoingStateUpdate(
        uint32_t id,
        const PhysicsFlatBuffers::Networking::Vec3& pos,
        const PhysicsFlatBuffers::Networking::Vec4& rot,
        const PhysicsFlatBuffers::Networking::Vec3& vel,
        const PhysicsFlatBuffers::Networking::Vec3& angVel,
        const PhysicsFlatBuffers::Networking::Vec4& col
    ) : objectId(id), position(pos), rotation(rot), velocity(vel), angular_velocity(angVel), color(col) {
    }

    // Default constructor needed for some container operations
    OutgoingStateUpdate() : objectId(0), position(), rotation(), velocity(), angular_velocity(), color() {}
};

// Internal struct for Object Creation messages received from the network.
struct InternalObjectCreationMessage {
    uint32_t id;
    int object_type; // Corresponds to your FBS object_type (consider using an enum)
    PhysicsFlatBuffers::Networking::Vec3 initial_position;
    PhysicsFlatBuffers::Networking::Vec4 initial_rotation;
    PhysicsFlatBuffers::Networking::Vec3 initial_scale;
    PhysicsFlatBuffers::Networking::Vec3 initial_velocity;
    PhysicsFlatBuffers::Networking::Vec3 initial_angular_velocity;
    bool is_static;

    // Shape-specific data - copy from the FlatBuffers table
    float sphere_radius;
    PhysicsFlatBuffers::Networking::Vec3 box_size;
    float cylinder_radius;
    float cylinder_height;
    PhysicsFlatBuffers::Networking::Vec3 plane_size;
    // Add other shape fields...

    // Constructor
    InternalObjectCreationMessage(
        uint32_t obj_id, int type,
        const PhysicsFlatBuffers::Networking::Vec3& pos,
        const PhysicsFlatBuffers::Networking::Vec4& rot,
        const PhysicsFlatBuffers::Networking::Vec3& scale,
        const PhysicsFlatBuffers::Networking::Vec3& vel,
        const PhysicsFlatBuffers::Networking::Vec3& angVel,
        bool is_stat,
        float s_rad, const PhysicsFlatBuffers::Networking::Vec3& b_size,
        float c_rad, float c_height, const PhysicsFlatBuffers::Networking::Vec3& p_size
        // Add other shape params...
    ) : id(obj_id), object_type(type), initial_position(pos), initial_rotation(rot),
        initial_scale(scale), initial_velocity(vel), initial_angular_velocity(angVel),
        is_static(is_stat), sphere_radius(s_rad), box_size(b_size),
        cylinder_radius(c_rad), cylinder_height(c_height), plane_size(p_size)
        // Initialize other shape fields...
    {
    }

    // Default constructor
    InternalObjectCreationMessage() : id(0), object_type(0), initial_position(), initial_rotation(),
        initial_scale(), initial_velocity(), initial_angular_velocity(), is_static(false),
        sphere_radius(0.0f), box_size(), cylinder_radius(0.0f), cylinder_height(0.0f), plane_size()
        // Default initialize other shape fields...
    {
    }
};

// Internal struct for Peer Joined messages received from the network.
struct InternalPeerJoinedMessage {
    uint32_t peer_id;
    // Add other peer info if needed
};

// Internal struct for Peer Left messages received from the network.
struct InternalPeerLeftMessage {
    uint32_t peer_id;
    // Add other peer info if needed
};

// Internal struct for Gravity Changed messages received from the network.
struct InternalGravityChangedMessage {
    PhysicsFlatBuffers::Networking::Vec3 new_gravity;
};


// --- Unified Received Message Struct ---
// This struct will be used in the queue from Network to Physics.
// It can hold different types of messages received from the network.
enum class ReceivedMessageType {
    StateUpdate,
    ObjectCreation,
    PeerJoined,
    PeerLeft,
    GravityChanged,
    // Add other message types here
    Unknown
};

struct ReceivedMessage {
    ReceivedMessageType type = ReceivedMessageType::Unknown;
    // Use std::variant to hold the actual message data (requires C++17)
    std::variant<
        // List all possible message types here
        std::vector<OutgoingStateUpdate>, // StateUpdate contains a list of objects (using OutgoingStateUpdate struct for consistency)
        InternalObjectCreationMessage,
        InternalPeerJoinedMessage,
        InternalPeerLeftMessage,
        InternalGravityChangedMessage
        // Add other internal message structs here
    > data;

    // Helper to check message type
    bool isStateUpdate() const { return type == ReceivedMessageType::StateUpdate; }
    bool isObjectCreation() const { return type == ReceivedMessageType::ObjectCreation; }
    bool isPeerJoined() const { return type == ReceivedMessageType::PeerJoined; }
    bool isPeerLeft() const { return type == ReceivedMessageType::PeerLeft; }
    bool isGravityChanged() const { return type == ReceivedMessageType::GravityChanged; }
    // Add helpers for other types...
};


// --- Declare Extern References to Shared Data ---
// These variables must be defined elsewhere in your application (e.g., in your main game state class)
// The NetworkSystem will interact with these shared resources.

// Queue for messages from Physics to Network (primarily OutgoingStateUpdate)
extern std::queue<OutgoingStateUpdate> g_physicsToNetworkQueue;
extern std::mutex g_physicsToNetworkMutex;
extern std::condition_variable g_physicsToNetworkCV; // Physics thread notifies when data is available

// Queue for messages from Network to Physics (can contain different received types)
extern std::queue<ReceivedMessage> g_networkToPhysicsQueue;
extern std::mutex g_networkToPhysicsMutex;
extern std::condition_variable g_networkToPhysicsCV; // Network thread notifies when data is available

// Global shutdown flag (assuming it's used across all threads)
extern std::atomic<bool> g_shutdownFlag;


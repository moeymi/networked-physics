#include "NetworkEngine.h"

NetworkEngine::NetworkEngine() {
    // Winsock initialization is deferred to StartHostSession/JoinClientSession
    // Base ThreadedSystem members are default initialized
}

NetworkEngine::~NetworkEngine() {
    std::cout << "NetworkManager shutting down..." << std::endl;
    stopSession(); // Clean up any active session first
    cleanupWinsock(); // Clean up Winsock last
    std::cout << "NetworkManager destroyed." << std::endl;
}

// --- Public Session Control Methods ---

bool NetworkEngine::startHostSession(int port, SharedSimulationData* sharedData) {
    if (m_role != Role::None) {
        std::cerr << "Error: Cannot start host session, NetworkManager is already initialized." << std::endl;
        return false;
    }
    if (!sharedData) {
        std::cerr << "Error: SharedSimulationData pointer is null." << std::endl;
        return false;
    }

    if (!initializeWinsock()) return false;

    m_role = Role::Host;
    m_port = port;
    m_sharedSimulationData = sharedData;
    m_connectedClients.clear(); // Ensure client list is clean

    m_listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_listenSocket == INVALID_SOCKET) {
        std::cerr << "Host socket creation failed: " << WSAGetLastError() << std::endl;
        stopSession(); // Clean up Winsock
        return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Listen on any IP
    serverAddr.sin_port = htons(port);

    if (bind(m_listenSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::cerr << "Host bind failed: " << WSAGetLastError() << std::endl;
        stopSession();
        return false;
    }

    if (listen(m_listenSocket, SOMAXCONN) == SOCKET_ERROR) {
        std::cerr << "Host listen failed: " << WSAGetLastError() << std::endl;
        stopSession();
        return false;
    }

    m_listening = true;
    m_listeningThread = std::thread(&NetworkEngine::runListeningThread, this);
	auto str = "NetworkManager started as Host, listening on port " + std::to_string(port) + '\n';
    OutputDebugStringA(str.c_str());

    // Start the base ThreadedSystem run loop (calls our onUpdate)
    start();

    return true;
}

bool NetworkEngine::joinClientSession(const char* hostAddress, int port, SharedSimulationData* sharedData) {

    if (m_role != Role::None) {
        std::cerr << "Error: Cannot join client session, NetworkManager is already initialized." << std::endl;
        return false;
    }
    if (!sharedData) {
        std::cerr << "Error: SharedSimulationData pointer is null." << std::endl;
        return false;
    }

    if (!initializeWinsock()) return false;

    m_role = Role::Client;
    m_port = port; // Store port for reference
    m_hostAddress = hostAddress;
    m_sharedSimulationData = sharedData;
    m_hostSocket = INVALID_SOCKET; // Ensure it's invalid before creation

    m_hostSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (m_hostSocket == INVALID_SOCKET) {
        std::cerr << "Client socket creation failed: " << WSAGetLastError() << std::endl;
        stopSession(); // Clean up Winsock
        return false;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    if (inet_pton(AF_INET, hostAddress, &serverAddr.sin_addr) <= 0) {
        std::cerr << "Invalid host address format: " << hostAddress << std::endl;
        stopSession();
        return false;
    }

    OutputDebugStringA(("NetworkManager starting as Client, attempting to connect to " + std::string(hostAddress) + ":" + std::to_string(port) + "...\n").c_str());

    // This connect call is blocking. In a real application,
    // if called from the main thread, this would freeze the UI.
    // You might want to run this connection attempt in a temporary thread.
    if (connect(m_hostSocket, (sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR) {
        std::string errorMessage = "Client connect failed: " + std::to_string(WSAGetLastError()) + "\n";  
        OutputDebugStringA(errorMessage.c_str());
        stopSession();
        return false;
    }

    OutputDebugStringA("Successfully connected to host.\n");

    m_receiving = true;
    m_receivingThread = std::thread(&NetworkEngine::runReceivingThread, this);

    // Start the base ThreadedSystem run loop
    start();

    return true;
}

// New method to stop any active session
void NetworkEngine::stopSession() {
    std::cout << "Stopping current network session..." << std::endl;

    // Signal threads to stop
    m_listening = false;
    m_receiving = false;

    // Unblock blocking socket calls by closing sockets
    if (m_listenSocket != INVALID_SOCKET) {
        closesocket(m_listenSocket);
        m_listenSocket = INVALID_SOCKET;
    }
    if (m_hostSocket != INVALID_SOCKET) {
        closesocket(m_hostSocket);
        m_hostSocket = INVALID_SOCKET;
    }

    // Stop the base ThreadedSystem run loop (calls onUpdate)
    stop(); // This joins m_thread

    // Join network specific threads AFTER unblocking sockets
    if (m_listeningThread.joinable()) {
        m_listeningThread.join();
    }
    if (m_receivingThread.joinable()) {
        m_receivingThread.join();
    }

    // Close all client sockets on host role
    if (m_role == Role::Host) {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        for (auto& client : m_connectedClients) {
            if (client.socket != INVALID_SOCKET) {
                closesocket(client.socket);
                client.socket = INVALID_SOCKET;
            }
        }
        m_connectedClients.clear();
    }

    m_role = Role::None; // Reset role
    m_sharedSimulationData = nullptr; // Clear shared data pointer
    m_port = 0;
    m_hostAddress.clear();

    std::cout << "Session stopped. Role reset to None." << std::endl;
}


bool NetworkEngine::initializeWinsock() {
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "WSAStartup failed: " << iResult << std::endl;
        return false;
    }
    std::cout << "Winsock initialized." << std::endl;
    return true;
}

void NetworkEngine::cleanupWinsock() {
    WSACleanup();
    std::cout << "Winsock cleaned up." << std::endl;
}

void NetworkEngine::onUpdate(float deltaTime) {
    if (m_role == Role::Host && m_sharedSimulationData) {
        // --- HOST LOGIC ---
        // Build the state update payload from the shared data.
        // This assumes the physics thread (or whatever is calling onUpdate)
        // has just finished updating the shared data.
        std::vector<uint8_t> statePayload;
        { // Lock the shared simulation data only while reading and building the payload
            std::lock_guard<std::mutex> lock(m_sharedSimulationData->mutex);
            // buildStateUpdatePayload reads from m_sharedSimulationData->objects
            statePayload = buildStateUpdatePayload(m_sharedSimulationData->objects);
        } // Lock is released here

        if (statePayload.empty() && !m_sharedSimulationData->objects.empty()) {
            // Log a warning if objects exist but payload is empty - something went wrong in serialization
            std::cerr << "Warning: buildStateUpdatePayload returned empty payload despite objects existing!" << std::endl;
        }

        // Send the payload to all connected clients.
        // Protect the client list while iterating and sending.
        std::lock_guard<std::mutex> clientsLock(m_clientsMutex);

        auto it = m_connectedClients.begin();
        while (it != m_connectedClients.end()) {
            // Only send if the socket is valid
            if (it->socket != INVALID_SOCKET) {
                if (!sendMessage(it->socket, NetMessageType::StateUpdate, statePayload)) {
                    // Sending failed, assume client disconnected or error
                    std::cerr << "Failed to send state update to client socket " << it->socket << ". Disconnecting..." << std::endl;
                    // Close the socket
                    if (it->socket != INVALID_SOCKET) {
                        closesocket(it->socket);
                        it->socket = INVALID_SOCKET; // Mark as invalid
                    }
                    // Remove client from the list
                    it = m_connectedClients.erase(it); // erase returns iterator to the next element
                }
                else {
                    // Successfully sent
                    ++it; // Move to the next client
                }
            }
            else {
                // Socket was already invalid, just remove it from the list
                it = m_connectedClients.erase(it);
            }
        }

    }
    else if (m_role == Role::Client) {
        // --- CLIENT LOGIC ---
        // Client's onUpdate doesn't perform network sending based on physics ticks in this phase.
        // The runReceivingThread handles receiving updates.
        // This method could be used for client-side interpolation or prediction updates,
        // which would read from the m_sharedSimulationData (under lock) and update rendering poses.
        // For this basic synchronization phase, the client onUpdate can effectively do nothing.
        // ... Client-side interpolation logic could go here ...
    }
    // If m_role is None, onUpdate should not be running, as start() shouldn't have been called
    // or stop() should have been called.
}

// --- runListeningThread (Host Only) Implementation ---
// (Implementation is exactly the same as in the previous response)
void NetworkEngine::runListeningThread() {
    OutputDebugStringA("Host listening thread started.\n");
    while (m_listening && m_listenSocket != INVALID_SOCKET) { // Check socket validity in loop condition
        sockaddr_in clientAddr;
        int clientAddrSize = sizeof(clientAddr);
        SOCKET clientSocket = accept(m_listenSocket, (sockaddr*)&clientAddr, &clientAddrSize);

        if (clientSocket == INVALID_SOCKET) {
            int error = WSAGetLastError();
            if (m_listening) { // Only print error if we didn't intend to stop
                std::cerr << "accept failed: " << error << std::endl;
            }
            // If m_listening is false and socket is invalid, this break is expected from StopSession
            break;
        }

        // New client connected
        char clientIp[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(clientAddr.sin_addr), clientIp, INET_ADDRSTRLEN);
        OutputDebugStringA(("Client connected from " + std::string(clientIp) + ":" + std::to_string(ntohs(clientAddr.sin_port)) + ". Socket: " + std::to_string(clientSocket) + "\n").c_str());

        ClientConnection newClient;
        newClient.socket = clientSocket;
        newClient.address = clientAddr;

        // Add client to the list under mutex protection
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            m_connectedClients.push_back(newClient);
        }

        // Immediately send the initial full state to the new client
        std::vector<uint8_t> initialStatePayload;
        if (m_sharedSimulationData) { // Ensure shared data is available
            std::lock_guard<std::mutex> stateLock(m_sharedSimulationData->mutex);
            initialStatePayload = buildStateUpdatePayload(m_sharedSimulationData->objects);
        }

        if (!initialStatePayload.empty()) {
            if (!sendMessage(clientSocket, NetMessageType::InitialState, initialStatePayload)) {
                std::cerr << "Failed to send initial state to new client " << clientSocket << ". Disconnecting..." << std::endl;
                // Sending initial state failed, clean up this client
                closesocket(clientSocket);
                // Remove from client list under mutex
                std::lock_guard<std::mutex> lock(m_clientsMutex);
                m_connectedClients.erase(
                    std::remove_if(m_connectedClients.begin(), m_connectedClients.end(),
                        [&](const ClientConnection& c) { return c.socket == clientSocket; }),
                    m_connectedClients.end());
            }
            else {
                OutputDebugStringA(("Sent initial state (" + std::to_string(initialStatePayload.size()) + " bytes) to client " + std::to_string(clientSocket) + "\n").c_str());
                // Client is now set up. If you needed to receive input from this client,
                // you would start a dedicated receiving thread for clientSocket here.
            }
        }
        else {
            std::cerr << "Initial state payload is empty for client " << clientSocket << ". Nothing sent." << std::endl;
            // Decide how to handle empty initial state - maybe disconnect the client?
        }
    }
    std::cout << "Host listening thread stopped." << std::endl;
}


// --- runReceivingThread (Client Only) Implementation ---
// (Implementation is exactly the same as in the previous response)
void NetworkEngine::runReceivingThread() {
    std::cout << "Client receiving thread started." << std::endl;
    NetMessageType type;
    std::vector<uint8_t> payload;
    std::vector<CPPGameState> receivedObjectsTemp; // Temp storage for deserialized objects

    // Continue receiving as long as we are meant to be receiving AND the socket is valid
    while (m_receiving && m_hostSocket != INVALID_SOCKET) {
        if (receiveMessage(m_hostSocket, type, payload)) {
            // Successfully received a message
            // std::cout << "Client received message type: " << (uint32_t)type << " size: " << payload.size() << std::endl;

            if (type == NetMessageType::InitialState || type == NetMessageType::StateUpdate) {
                // Apply the state update from the FlatBuffer payload
                // applyStateUpdatePayload populates receivedObjectsTemp
                if (applyStateUpdatePayload(payload, receivedObjectsTemp)) {
                    // Data successfully deserialized and is in receivedObjectsTemp
                    // Now, update the shared simulation data structure under lock
                    if (m_sharedSimulationData) { // Ensure shared data pointer is valid
                        std::lock_guard<std::mutex> lock(m_sharedSimulationData->mutex);
                        // Replace the entire local object list with the received state
                        // std::move is efficient here
                        m_sharedSimulationData->objects = std::move(receivedObjectsTemp);

                        if (type == NetMessageType::InitialState) {
                            std::cout << "Client applied initial state (" << m_sharedSimulationData->objects.size() << " objects)." << std::endl;
                        }
                    }
                    else {
                        std::cerr << "Error: SharedSimulationData pointer is null while applying received state!" << std::endl;
                        // This indicates a serious logic error - the manager should have a valid pointer if initialized
                    }

                }
                else {
                    std::cerr << "Client failed to apply state update payload (invalid data?)." << std::endl;
                    // Decide how to handle this - ignore update, log, or disconnect?
                }
            }
            else {
                std::cerr << "Client received unknown message type: " << (uint32_t)type << ". Size: " << payload.size() << std::endl;
                // Handle other message types here later (e.g., server messages, events)
            }

        }
        else {
            // receiveMessage returned false: error or host disconnected
            std::cerr << "Host disconnected or receive error (" << WSAGetLastError() << "). Stopping client receiving." << std::endl;
            m_receiving = false; // Signal thread to stop
            // Consider notifying the Main Thread or other systems about the disconnect
            // The NetworkManager destructor/StopSession will handle closing the socket
            break; // Exit the receiving loop
        }
    }

    std::cout << "Client receiving thread stopped." << std::endl;
}

// --- FlatBuffers Serialization/Deserialization ---

// Builds a FlatBuffer payload for the current state
std::vector<uint8_t> NetworkEngine::buildStateUpdatePayload(const std::vector<CPPGameState>& objects) {
    flatbuffers::FlatBufferBuilder builder;

    // Create FlatBuffer objects from your GameObjectState vector
    std::vector<flatbuffers::Offset<Networking::GameObjectState>> fb_objects_vec;
    fb_objects_vec.reserve(objects.size());

    for (const auto& obj_state : objects) {
        // Create nested structs first
        auto fb_pos = Networking::Vec3(obj_state.position.x, obj_state.position.y, obj_state.position.z);
        auto fb_rot = Networking::Vec4(obj_state.rotation.x, obj_state.rotation.y, obj_state.rotation.z, obj_state.rotation.w);
        auto fb_vel = Networking::Vec3(obj_state.velocity.x, obj_state.velocity.y, obj_state.velocity.z);
        auto fb_ang_vel = Networking::Vec3(obj_state.angularVelocity.x, obj_state.angularVelocity.y, obj_state.angularVelocity.z);
        auto fb_color = Networking::Vec4(obj_state.color.x, obj_state.color.y, obj_state.color.z, obj_state.color.w);

        // Create the GameObjectState table
        auto fb_obj = Networking::CreateGameObjectState(builder,
            obj_state.id,
			obj_state.type,
            &fb_pos,
            &fb_rot,
            &fb_vel,
            &fb_ang_vel,
            &fb_color
            // Add other fields matching schema
        );
        fb_objects_vec.push_back(fb_obj);
    }

    // Create the FlatBuffer vector of objects
    auto objects_vector_offset = builder.CreateVector(fb_objects_vec);

    // Create the root StateUpdate table
    auto state_update_offset = Networking::CreateStateUpdate(builder, objects_vector_offset);

    // Finish the buffer
    builder.Finish(state_update_offset);

    // Return the buffer data as a std::vector<uint8_t>
    // flatbuffers::FlatBufferBuilder ensures the buffer is aligned and valid
    uint8_t* buf_ptr = builder.GetBufferPointer();
    int buf_size = builder.GetSize();

    // Copy data from builder's buffer into a standard vector to return
    return std::vector<uint8_t>(buf_ptr, buf_ptr + buf_size);
}

// Takes a received FlatBuffer payload and updates the local state
// Returns true on successful deserialization and application, false otherwise
bool NetworkEngine::applyStateUpdatePayload(const std::vector<uint8_t>& payload, std::vector<CPPGameState>& outObjects) {
    // Optional: Verify the FlatBuffer buffer first (recommended)
    flatbuffers::Verifier verifier(payload.data(), payload.size());
    bool ok = Networking::VerifyStateUpdateBuffer(verifier); // Call the verification function generated by flatc

    if (!ok) {
        std::cerr << "Received invalid FlatBuffer data. Verification failed." << std::endl;
        return false; // Data is corrupt or doesn't match schema
    }

    // Access the root object directly from the buffer - zero copy!
    const Networking::StateUpdate* state_update =
        Networking::GetStateUpdate(payload.data());

    // Access the vector of objects
    const flatbuffers::Vector<flatbuffers::Offset<Networking::GameObjectState>>* fb_objects = state_update->objects();

    if (!fb_objects) {
        // Payload contained a StateUpdate table, but the objects vector field was null/empty.
        // Depending on your protocol, this might be valid (e.g., no objects changed) or an error.
        // Assuming for now an empty list means clear objects.
        outObjects.clear();
        return true; // Successfully processed empty list
    }

    // Update the local GameObjectState vector by copying data from the FlatBuffer
    // This part *does* involve copying, but it's from the received buffer
    // directly into your simulation data structure, avoiding intermediate steps.
    outObjects.clear(); // Simple approach: replace the entire list
    outObjects.reserve(fb_objects->size()); // Reserve space

    for (const auto& fb_obj : *fb_objects) {
        CPPGameState obj_state;
        obj_state.id = fb_obj->id(); // Access data using generated accessors

		obj_state.type = fb_obj->type(); // Assuming type is an enum or int

        // Copy nested struct data
        const auto* fb_pos = fb_obj->position();
        if (fb_pos) obj_state.position = DirectX::XMFLOAT3(fb_pos->x(), fb_pos->y(), fb_pos->z());

        const auto* fb_rot = fb_obj->rotation();
        if (fb_rot) obj_state.rotation = DirectX::XMFLOAT4(fb_rot->x(), fb_rot->y(), fb_rot->z(), fb_rot->w());

        const auto* fb_vel = fb_obj->velocity();
        if (fb_vel) obj_state.velocity = DirectX::XMFLOAT3(fb_vel->x(), fb_vel->y(), fb_vel->z());

        const auto* fb_ang_vel = fb_obj->angular_velocity();
        if (fb_ang_vel) obj_state.angularVelocity = DirectX::XMFLOAT3(fb_ang_vel->x(), fb_ang_vel->y(), fb_ang_vel->z());

        const auto* fb_color = fb_obj->color();
        if (fb_color) obj_state.color = DirectX::XMFLOAT4(fb_color->x(), fb_color->y(), fb_color->z(), fb_color->w());

        // ... copy other fields ...

        outObjects.push_back(obj_state);
    }

    // Note: A more robust update might involve looking up objects by ID in outObjects
    // and updating them individually if they exist, or adding new ones, removing missing ones.
    // This would require outObjects to potentially be a map<int, GameObjectState>.

    return true; // Success
}


// --- Message Sending/Receiving Helpers ---

// Sends a message (header + payload) over a TCP socket (blocking)
bool NetworkEngine::sendMessage(SOCKET targetSocket, NetMessageType type, const std::vector<uint8_t>& payload) {
    if (targetSocket == INVALID_SOCKET) return false;

    NetMessageHeader header;
    header.type = type;
    header.payloadSize = static_cast<uint32_t>(payload.size());

    // Send header
    int totalSent = 0;
    int bytesToSend = sizeof(header);
    const char* headerPtr = (const char*)&header;

    while (totalSent < bytesToSend) {
        int sent = send(targetSocket, headerPtr + totalSent, bytesToSend - totalSent, 0);
        if (sent == SOCKET_ERROR) {
            // Error occurred
            int error = WSAGetLastError();
            std::cerr << "Send header failed (type " << (uint32_t)type << "): " << error << std::endl;
            return false;
        }
        totalSent += sent;
    }

    // Send payload
    if (!payload.empty()) {
        totalSent = 0;
        bytesToSend = payload.size();
        const char* payloadPtr = (const char*)payload.data();

        while (totalSent < bytesToSend) {
            int sent = send(targetSocket, payloadPtr + totalSent, bytesToSend - totalSent, 0);
            if (sent == SOCKET_ERROR) {
                // Error occurred
                int error = WSAGetLastError();
                std::cerr << "Send payload failed (type " << (uint32_t)type << "): " << error << std::endl;
                return false;
            }
            totalSent += sent;
        }
    }

    return true; // Success
}

// Receives a complete message (header + payload) from a TCP socket (blocking)
// Returns true if a message was read successfully, false on error/disconnect
bool NetworkEngine::receiveMessage(SOCKET sourceSocket, NetMessageType& outType, std::vector<uint8_t>& outPayload) {
    if (sourceSocket == INVALID_SOCKET) return false;

    NetMessageHeader header;
    outPayload.clear(); // Clear previous payload

    // Receive header (blocking until full header is received or error/disconnect)
    int bytesReceived = recv(sourceSocket, (char*)&header, sizeof(header), MSG_WAITALL);

    if (bytesReceived <= 0) {
        // 0 indicates graceful shutdown, SOCKET_ERROR indicates an error
        int error = WSAGetLastError();
        if (bytesReceived == 0) {
            // std::cout << "Peer disconnected gracefully." << std::endl;
        }
        else {
            std::cerr << "Receive header failed: " << error << std::endl;
        }
        return false; // Disconnected or error
    }
    if (bytesReceived != sizeof(header)) {
        std::cerr << "Received partial header? Should not happen with MSG_WAITALL. Received " << bytesReceived << " expected " << sizeof(header) << std::endl;
        return false; // Unexpected state
    }

    // Validate header (simple checks)
    if (header.payloadSize > MAX_PAYLOAD_SIZE) {
        std::cerr << "Received payload size too large: " << header.payloadSize << ". Disconnecting peer." << std::endl;
        // Depending on protocol, you might close the socket here
        return false;
    }
    // Add more checks if message types are restricted

    outType = header.type;

    // Receive payload (blocking until full payload or error/disconnect)
    if (header.payloadSize > 0) {
        outPayload.resize(header.payloadSize);
        bytesReceived = recv(sourceSocket, (char*)outPayload.data(), header.payloadSize, MSG_WAITALL);

        if (bytesReceived <= 0 || bytesReceived != header.payloadSize) {
            int error = WSAGetLastError();
            if (bytesReceived == 0) {
                // std::cout << "Peer disconnected gracefully during payload receive." << std::endl;
            }
            else {
                std::cerr << "Receive payload failed: " << error << std::endl;
            }
            return false; // Disconnected or error during payload
        }
    }

    return true;
}

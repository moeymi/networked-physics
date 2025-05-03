#include "NetworkEngine.h"

// FlatBuffers includes
#include "flatbuffers/flatbuffer_builder.h"
#include "game_state_generated.h"

// Define the extern variables declared in NetworkMessages.h
// In a real application, these would be members of a central game state or system manager class
// and references would be passed to the NetworkSystem constructor.
// For this example, we'll define them here as globals for simplicity.
// REMOVE these definitions if you define them elsewhere!
std::queue<OutgoingStateUpdate> g_physicsToNetworkQueue;
std::mutex g_physicsToNetworkMutex;
std::condition_variable g_physicsToNetworkCV;

std::queue<ReceivedMessage> g_networkToPhysicsQueue;
std::mutex g_networkToPhysicsMutex;
std::condition_variable g_networkToPhysicsCV;

// Assuming SimulationSettings and g_shutdownFlag are defined elsewhere
// struct SimulationSettings { double targetNetworkFrequency = 20.0; }; // Example definition
// SimulationSettings g_simulationSettings;
// std::mutex g_simulationSettingsMutex;
// std::atomic<bool> g_shutdownFlag(false);


// --- NetworkSystem Implementation ---

NetworkEngine::NetworkEngine(const std::string& listenPort, const std::vector<std::string>& peerAddresses)
    : m_listenPort(listenPort), m_peerAddressesToConnect(peerAddresses) {
    // Constructor initializes members. Networking setup happens in initializeNetworking()
    // which is called at the start of the run() method.
    m_lastStateSendTime = std::chrono::steady_clock::now(); // Initialize state send timer
}

NetworkEngine::~NetworkEngine() {
    // stop() is called by the base class destructor, which joins the thread.
    // shutdownNetworking() is called at the end of the run() method.
}

// Dummy implementation as the base run() is overridden
void NetworkEngine::onUpdate(float deltaTime) {
    // This method is not used by the NetworkSystem's run loop.
    // It's here only to satisfy the pure virtual requirement of the base class.
}

// --- Core Network Thread Loop ---
void NetworkEngine::run() {
    std::cout << "Network Thread: Starting..." << std::endl;

    // Initialize networking resources
    if (!initializeNetworking()) {
        std::cerr << "Network Thread: Failed to initialize networking. Exiting." << std::endl;
        m_running = false; // Ensure the thread stops
        return;
    }

    // Set the initial time for the state send timer
    m_lastStateSendTime = std::chrono::steady_clock::now();

    // Main network loop
    while (m_running) {
        // Use select to wait for network events on multiple sockets
        fd_set readfds;
        fd_set writefds; // We need to check for writability for send buffering
        FD_ZERO(&readfds);
        FD_ZERO(&writefds);

        // Add listen socket for accepting new connections
        if (m_listenSocket != INVALID_SOCKET) {
            FD_SET(m_listenSocket, &readfds);
        }

        // Add connected peer sockets for receiving data
        // Add sockets with pending send data to writefds
        {
            std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
            for (SOCKET sock : m_connectedPeerSockets) {
                FD_SET(sock, &readfds); // Always check for readability

                // Check if this socket has pending data in the send buffer
                std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
                if (m_sendBuffers.count(sock) && m_sendBuffers[sock].bytesSent < m_sendBuffers[sock].buffer.size()) {
                    FD_SET(sock, &writefds); // Check for writability if there's data to send
                }
            }
        } // Lock released for m_connectedPeerSocketsMutex

        // Set a timeout for select (e.g., 10ms) to allow the loop to check m_running flag
        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000; // 10 milliseconds

        // Use select to wait for activity on sockets
        int ready_count = select(0, &readfds, &writefds, NULL, &timeout);

        if (!m_running) break; // Check shutdown flag after select returns

        if (ready_count == SOCKET_ERROR) {
            int err = WSAGetLastError();
            // WSAEINTR is often a signal interruption, might be okay to ignore or handle gracefully
            if (err != WSAEINTR) {
                std::cerr << "Network Thread: select failed with error: " << err << std::endl;
                // Handle fatal select error - potentially stop the system
                m_running = false;
                break;
            }
        }

        // List of sockets to remove after processing to avoid iterator issues
        std::vector<SOCKET> socketsToRemove;

        if (ready_count > 0) {
            // Handle incoming connections if the listen socket is ready
            if (m_listenSocket != INVALID_SOCKET && FD_ISSET(m_listenSocket, &readfds)) {
                handleIncomingConnections();
            }

            // Handle data receives and sends on connected sockets
            std::vector<SOCKET> currentConnectedSockets;
            {
                std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
                currentConnectedSockets = m_connectedPeerSockets;
            }

            for (SOCKET sock : currentConnectedSockets) {
                // Check if this socket is still valid in the main list
                bool socketIsValid = false;
                {
                    std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
                    for (SOCKET connected_sock : m_connectedPeerSockets) {
                        if (connected_sock == sock) {
                            socketIsValid = true;
                            break;
                        }
                    }
                }
                if (!socketIsValid) continue; // Skip if socket was already removed


                if (FD_ISSET(sock, &readfds)) {
                    // handleDataReceive returns true if the socket should be closed
                    if (handleDataReceive(sock)) {
                        socketsToRemove.push_back(sock);
                        continue; // Skip further processing for this socket in this iteration
                    }
                }

                // Check if this socket is still valid after receive handling
                socketIsValid = false;
                {
                    std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
                    for (SOCKET connected_sock : m_connectedPeerSockets) {
                        if (connected_sock == sock) {
                            socketIsValid = true;
                            break;
                        }
                    }
                }
                if (!socketIsValid) continue; // Skip if socket was removed during receive handling


                // Check if this socket is ready for writing AND has data pending in the send buffer
                std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
                if (FD_ISSET(sock, &writefds) && m_sendBuffers.count(sock) && m_sendBuffers[sock].bytesSent < m_sendBuffers[sock].buffer.size()) {
                    // Attempt to send pending data for this socket
                    // processSocketSend returns true if the socket should be closed
                    if (processSocketSend(sock)) {
                        socketsToRemove.push_back(sock);
                        continue; // Skip further processing for this socket
                    }
                }
            }
        }

        // Periodically gather state updates from physics and serialize them (throttled by time)
        gatherAndSerializeStateUpdates();

        // Distribute any newly serialized messages (from state updates or public send calls)
        // to the per-socket send buffers.
        distributeOutgoingMessages();

        // --- Clean up disconnected sockets ---
        if (!socketsToRemove.empty()) {
            std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);

            for (SOCKET sock : socketsToRemove) {
                std::cout << "Network Thread: Cleaning up disconnected socket " << sock << std::endl;
                closesocket(sock);

                // Remove from connectedPeerSockets vector
                m_connectedPeerSockets.erase(
                    std::remove(m_connectedPeerSockets.begin(), m_connectedPeerSockets.end(), sock),
                    m_connectedPeerSockets.end()
                );

                // Remove from receive buffers map
                {
                    std::lock_guard<std::mutex> receiveBufferLock(m_receiveBuffersMutex);
                    m_receiveBuffers.erase(sock);
                }
                // Remove from send buffers map
                {
                    std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
                    m_sendBuffers.erase(sock);
                }
            }
            socketsToRemove.clear();
        }


        // Small sleep to prevent 100% CPU usage if select timeout is very short
        // std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Shutdown networking resources
    shutdownNetworking();
    std::cout << "Network Thread: Shutting down complete." << std::endl;
}

// --- Networking Initialization ---
bool NetworkEngine::initializeNetworking() {
    std::cout << "Network Thread: Initializing Winsock..." << std::endl;
    WSAData wsaData;
    int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (iResult != 0) {
        std::cerr << "Network Thread: WSAStartup failed: " << iResult << std::endl;
        return false;
    }

    // --- Setup Listen Socket ---
    std::cout << "Network Thread: Setting up listen socket on port " << m_listenPort << "..." << std::endl;
    struct addrinfo* result = NULL, hints;
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;       // IPv4
    hints.ai_socktype = SOCK_STREAM; // TCP
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;     // Listen socket

    iResult = getaddrinfo(NULL, m_listenPort.c_str(), &hints, &result);
    if (iResult != 0) {
        std::cerr << "Network Thread: getaddrinfo for listen failed: " << iResult << std::endl;
        WSACleanup();
        return false;
    }

    m_listenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (m_listenSocket == INVALID_SOCKET) {
        std::cerr << "Network Thread: socket creation failed: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        WSACleanup();
        return false;
    }

    iResult = bind(m_listenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Network Thread: bind failed with error: " << WSAGetLastError() << std::endl;
        freeaddrinfo(result);
        closesocket(m_listenSocket);
        WSACleanup();
        return false;
    }

    freeaddrinfo(result);

    iResult = listen(m_listenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        std::cerr << "Network Thread: listen failed with error: " << WSAGetLastError() << std::endl;
        closesocket(m_listenSocket);
        WSACleanup();
        return false;
    }

    // Set listen socket to non-blocking
    u_long nonBlocking = 1;
    if (ioctlsocket(m_listenSocket, FIONBIO, &nonBlocking) != NO_ERROR) {
        std::cerr << "Network Thread: ioctlsocket failed for listen socket: " << WSAGetLastError() << std::endl;
        closesocket(m_listenSocket);
        WSACleanup();
        return false;
    }
    std::cout << "Network Thread: Listening socket set up." << std::endl;

    // --- Connect to Other Peers ---
    std::cout << "Network Thread: Attempting to connect to initial peers..." << std::endl;
    for (const auto& peerAddr : m_peerAddressesToConnect) {
        connectToPeer(peerAddr); // Attempt connection
    }

    return true;
}

// --- Networking Shutdown ---
void NetworkEngine::shutdownNetworking() {
    std::cout << "Network Thread: Shutting down networking..." << std::endl;

    // Close listen socket
    if (m_listenSocket != INVALID_SOCKET) {
        closesocket(m_listenSocket);
        m_listenSocket = INVALID_SOCKET;
    }

    // Close all connected sockets
    {
        std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
        for (SOCKET sock : m_connectedPeerSockets) {
            closesocket(sock);
        }
        m_connectedPeerSockets.clear();
    }

    // Clear buffers and queues
    {
        std::lock_guard<std::mutex> receiveBufferLock(m_receiveBuffersMutex);
        m_receiveBuffers.clear();
    }
    {
        std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
        m_sendBuffers.clear();
    }
    {
        std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
        while (!m_outgoingNetMessages.empty()) m_outgoingNetMessages.pop();
    }


    // Cleanup Winsock
    WSACleanup();
    std::cout << "Network Thread: Winsock cleaned up." << std::endl;
}

// --- Handle Incoming Connections ---
void NetworkEngine::handleIncomingConnections() {
    SOCKET clientSocket = accept(m_listenSocket, NULL, NULL);
    if (clientSocket != INVALID_SOCKET) {
        // Set the accepted socket to non-blocking
        u_long nonBlocking = 1;
        if (ioctlsocket(clientSocket, FIONBIO, &nonBlocking) != NO_ERROR) {
            std::cerr << "Network Thread: ioctlsocket failed for accepted socket: " << WSAGetLastError() << ". Closing socket." << std::endl;
            closesocket(clientSocket);
        }
        else {
            std::cout << "Network Thread: Accepted new connection. Socket: " << clientSocket << std::endl;
            std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
            m_connectedPeerSockets.push_back(clientSocket);

            // Initialize receive buffer for this new socket
            std::lock_guard<std::mutex> receiveBufferLock(m_receiveBuffersMutex);
            m_receiveBuffers[clientSocket] = { clientSocket, {} };

            // Initialize send buffer for this new socket
            std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
            m_sendBuffers[clientSocket] = { clientSocket, {}, 0 };

            // TODO: Send a PeerJoined message to this new peer, and send a PeerJoined message
            // about this peer to all *other* existing peers. This requires knowing our own peer ID.
            // For now, we'll just log.
        }
    }
    else {
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            std::cerr << "Network Thread: accept failed with error: " << err << std::endl;
            // Handle fatal accept errors if necessary
        }
    }
}

// --- Handle Data Receive from a Specific Socket ---
// Returns true if the socket should be closed due to an error or disconnect.
bool NetworkEngine::handleDataReceive(SOCKET sock) {
    std::lock_guard<std::mutex> receiveBufferLock(m_receiveBuffersMutex);

    auto buffer_it = m_receiveBuffers.find(sock);
    if (buffer_it == m_receiveBuffers.end()) {
        // Should not happen if buffers are managed correctly
        std::cerr << "Network Thread: Error - Receive buffer not found for socket " << sock << ". Closing socket." << std::endl;
        return true; // Indicate socket should be closed
    }

    SocketReceiveBuffer& sockBuffer = buffer_it->second;
    const size_t BUFFER_CHUNK_SIZE = 4096; // Size to read in chunks

    // Attempt to receive data
    std::vector<char> tempRecvBuffer(BUFFER_CHUNK_SIZE);
    int bytesRead = recv(sock, tempRecvBuffer.data(), tempRecvBuffer.size(), 0);

    if (bytesRead > 0) {
        // Append received data to the socket's buffer
        sockBuffer.buffer.insert(sockBuffer.buffer.end(), tempRecvBuffer.begin(), tempRecvBuffer.begin() + bytesRead);
        // std::cout << "Network Thread: Received " << bytesRead << " bytes from socket " << sock << ". Buffer size: " << sockBuffer.buffer.size() << std::endl;

        // Process the buffer - extract complete FlatBuffers messages
        while (sockBuffer.buffer.size() >= sizeof(flatbuffers::uoffset_t)) {
            // FlatBuffers messages are prefixed by their size (uoffset_t, usually 4 bytes)
            const flatbuffers::uoffset_t messageSize = *flatbuffers::GetSizePrefixedRoot<flatbuffers::uoffset_t>(sockBuffer.buffer.data());

            if (sockBuffer.buffer.size() >= sizeof(flatbuffers::uoffset_t) + messageSize) {
                // We have a complete message!
                const uint8_t* messageStart = reinterpret_cast<const uint8_t*>(sockBuffer.buffer.data() + sizeof(flatbuffers::uoffset_t));

                // Verify the FlatBuffers message (important for security and robustness)
                flatbuffers::Verifier verifier(messageStart, messageSize);
                bool ok = PhysicsFlatBuffers::Networking::VerifyNetMessageBuffer(verifier); // Verify the root NetMessage

                if (ok) {
                    // Get the root of the message
                    const PhysicsFlatBuffers::Networking::NetMessage* netMessage =
                        PhysicsFlatBuffers::Networking::GetSizePrefixedNetMessage(sockBuffer.buffer.data());

                    // Process the message based on its type
                    ReceivedMessage receivedMsg;
                    auto message_type_type = netMessage->message_type_type(); // Get the type of the union content

                    if (message_type_type == PhysicsFlatBuffers::Networking::Message_StateUpdate) {
                        receivedMsg.type = ReceivedMessageType::StateUpdate;
                        const PhysicsFlatBuffers::Networking::StateUpdate* stateUpdate = netMessage->message_type_as_StateUpdate();

                        std::vector<OutgoingStateUpdate> stateUpdates; // Use OutgoingStateUpdate struct for consistency
                        const flatbuffers::Vector<flatbuffers::Offset<PhysicsFlatBuffers::Networking::GameObjectState>>* objects = stateUpdate->objects();

                        if (objects) {
                            stateUpdates.reserve(objects->size());
                            for (const auto& obj_fb : *objects) {
                                if (obj_fb) {
                                    OutgoingStateUpdate update( // Use OutgoingStateUpdate struct
                                        obj_fb->id(),
                                        *obj_fb->position(),
                                        *obj_fb->rotation(),
                                        *obj_fb->velocity(),
                                        *obj_fb->angular_velocity(),
                                        *obj_fb->color()
                                        // Copy other fields...
                                    );
                                    stateUpdates.push_back(update);
                                }
                            }
                        }
                        receivedMsg.data = stateUpdates; // Store the vector in the variant

                    }
                    else if (message_type_type == PhysicsFlatBuffers::Networking::Message_ObjectCreation) {
                        receivedMsg.type = ReceivedMessageType::ObjectCreation;
                        const PhysicsFlatBuffers::Networking::ObjectCreation* objCreation_fb = netMessage->message_type_as_ObjectCreation();

                        if (objCreation_fb) {
                            InternalObjectCreationMessage creationMsg(
                                objCreation_fb->id(),
                                objCreation_fb->object_type(),
                                *objCreation_fb->initial_position(),
                                *objCreation_fb->initial_rotation(),
                                *objCreation_fb->initial_scale(),
                                *objCreation_fb->initial_velocity(),
                                *objCreation_fb->initial_angular_velocity(),
                                objCreation_fb->is_static(),
                                objCreation_fb->sphere_radius(),
                                *objCreation_fb->box_size(),
                                objCreation_fb->cylinder_radius(),
                                objCreation_fb->cylinder_height(),
                                *objCreation_fb->plane_size()
                                // Copy other shape parameters...
                            );
                            receivedMsg.data = creationMsg; // Store the struct in the variant
                        }
                        else {
                            std::cerr << "Network Thread: Received null ObjectCreation message from socket " << sock << std::endl;
                            receivedMsg.type = ReceivedMessageType::Unknown; // Mark as unknown if data is null
                        }

                    }
                    else if (message_type_type == PhysicsFlatBuffers::Networking::Message_PeerJoined) {
                        receivedMsg.type = ReceivedMessageType::PeerJoined;
                        const PhysicsFlatBuffers::Networking::PeerJoined* peerJoined_fb = netMessage->message_type_as_PeerJoined();
                        if (peerJoined_fb) {
                            receivedMsg.data = InternalPeerJoinedMessage{ peerJoined_fb->peer_id() };
                        }
                        else {
                            std::cerr << "Network Thread: Received null PeerJoined message from socket " << sock << std::endl;
                            receivedMsg.type = ReceivedMessageType::Unknown;
                        }

                    }
                    else if (message_type_type == PhysicsFlatBuffers::Networking::Message_PeerLeft) {
                        receivedMsg.type = ReceivedMessageType::PeerLeft;
                        const PhysicsFlatBuffers::Networking::PeerLeft* peerLeft_fb = netMessage->message_type_as_PeerLeft();
                        if (peerLeft_fb) {
                            receivedMsg.data = InternalPeerLeftMessage{ peerLeft_fb->peer_id() };
                        }
                        else {
                            std::cerr << "Network Thread: Received null PeerLeft message from socket " << sock << std::endl;
                            receivedMsg.type = ReceivedMessageType::Unknown;
                        }

                    }
                    else if (message_type_type == PhysicsFlatBuffers::Networking::Message_GravityChanged) {
                        receivedMsg.type = ReceivedMessageType::GravityChanged;
                        const PhysicsFlatBuffers::Networking::GravityChanged* gravityChanged_fb = netMessage->message_type_as_GravityChanged();
                        if (gravityChanged_fb && gravityChanged_fb->new_gravity()) {
                            receivedMsg.data = InternalGravityChangedMessage{ *gravityChanged_fb->new_gravity() };
                        }
                        else {
                            std::cerr << "Network Thread: Received null GravityChanged message or null gravity vector from socket " << sock << std::endl;
                            receivedMsg.type = ReceivedMessageType::Unknown;
                        }

                    }
                    else {
                        // Handle unknown or unsupported message types
                        std::cerr << "Network Thread: Received unknown message type (" << (int)message_type_type << ") from socket " << sock << std::endl;
                        receivedMsg.type = ReceivedMessageType::Unknown;
                    }

                    // If a valid message was processed, queue it for the Physics Thread
                    if (receivedMsg.type != ReceivedMessageType::Unknown) {
                        std::lock_guard<std::mutex> queueLock(g_networkToPhysicsMutex);
                        g_networkToPhysicsQueue.push(receivedMsg);
                        g_networkToPhysicsCV.notify_one(); // Notify Physics thread
                        // std::cout << "Network Thread: Queued received message of type " << (int)receivedMsg.type << std::endl;
                    }


                    // Remove the processed message from the buffer
                    sockBuffer.buffer.erase(sockBuffer.buffer.begin(), sockBuffer.buffer.begin() + sizeof(flatbuffers::uoffset_t) + messageSize);

                }
                else {
                    // FlatBuffers verification failed - serious error
                    std::cerr << "Network Thread: FlatBuffers verification failed for message from socket " << sock << ". Closing socket." << std::endl;
                    sockBuffer.buffer.clear(); // Clear buffer to avoid getting stuck
                    return true; // Indicate socket should be closed
                }

            }
            else {
                // Not enough data for a complete message yet, wait for more
                break;
            }
        }

    }
    else if (bytesRead == 0) {
        // Connection closed by peer
        std::cout << "Network Thread: Peer disconnected from socket " << sock << "." << std::endl;
        return true; // Indicate socket should be closed

    }
    else {
        // Error or non-blocking indicated no data available *yet* (shouldn't happen if called based on select)
        int err = WSAGetLastError();
        if (err != WSAEWOULDBLOCK) {
            std::cerr << "Network Thread: recv failed with error: " << err << " on socket " << sock << ". Closing socket." << std::endl;
            return true; // Indicate socket should be closed
        }
    }
    return false; // Socket should remain open
}

// --- Gather State Updates from Physics Queue and Serialize ---
// This function is called periodically by the main loop to check if it's time to send
// and to gather state update messages from the physics queue, building a FlatBuffer message.
void NetworkEngine::gatherAndSerializeStateUpdates() {
    // Check if enough time has passed based on the target network frequency
    auto currentTime = std::chrono::steady_clock::now();
    double elapsedSendTime = std::chrono::duration<double>(currentTime - m_lastStateSendTime).count();

    double targetSendInterval = 1.0 / 60.0;
    if (elapsedSendTime >= targetSendInterval) {
        // Time to send! Gather pending state updates from the Physics queue.
        std::vector<OutgoingStateUpdate> updatesToSend;
        {
            std::lock_guard<std::mutex> queueLock(g_physicsToNetworkMutex);
            while (!g_physicsToNetworkQueue.empty()) {
                updatesToSend.push_back(g_physicsToNetworkQueue.front());
                g_physicsToNetworkQueue.pop();
            }
        } // Queue lock released

        if (!updatesToSend.empty()) {
            // Build the FlatBuffers StateUpdate message
            flatbuffers::FlatBufferBuilder builder;

            std::vector<flatbuffers::Offset<PhysicsFlatBuffers::Networking::GameObjectState>> object_offsets;
            object_offsets.reserve(updatesToSend.size());

            for (const auto& update : updatesToSend) {
                auto pos_fb = PhysicsFlatBuffers::Networking::Vec3(update.position.x(), update.position.y(), update.position.z());
                auto rot_fb = PhysicsFlatBuffers::Networking::Vec4(update.rotation.x(), update.rotation.y(), update.rotation.z(), update.rotation.w());
                auto vel_fb = PhysicsFlatBuffers::Networking::Vec3(update.velocity.x(), update.velocity.y(), update.velocity.z());
                auto angVel_fb = PhysicsFlatBuffers::Networking::Vec3(update.angular_velocity.x(), update.angular_velocity.y(), update.angular_velocity.z());
                auto color_fb = PhysicsFlatBuffers::Networking::Vec4(update.color.x(), update.color.y(), update.color.z(), update.color.w());


                auto obj_offset = PhysicsFlatBuffers::Networking::CreateGameObjectState(
                    builder,
                    update.objectId,
                    0, // type - Assuming type is part of state or creation? Clarify this.
                    &pos_fb,
                    &rot_fb,
                    &vel_fb,
                    &angVel_fb,
                    &color_fb
                    // Add other fields...
                );
                object_offsets.push_back(obj_offset);
            }

            auto objects_vec = builder.CreateVector(object_offsets);
            auto state_update_offset = PhysicsFlatBuffers::Networking::CreateStateUpdate(builder, objects_vec);

            // Now, build the root NetMessage containing the StateUpdate
            auto net_message_offset = PhysicsFlatBuffers::Networking::CreateNetMessage(
                builder,
                PhysicsFlatBuffers::Networking::Message_StateUpdate, // Specify the type in the union
                state_update_offset.Union() // Get the union value
            );

            // Finish the buffer, adding the size prefix
            builder.FinishSizePrefixed(net_message_offset);

            // Get the raw buffer data
            std::vector<char> sendBuffer(builder.GetSize());
            memcpy(sendBuffer.data(), builder.GetBufferPointer(), builder.GetSize());

            // Queue this complete FlatBuffers message for sending to all peers
            {
                std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
                m_outgoingNetMessages.push({ sendBuffer });
            } // Lock released

            // std::cout << "Network Thread: Prepared FlatBuffers StateUpdate message (" << sendBuffer.size() << " bytes) for sending." << std::endl;
        }

        // Reset the state send timer regardless of whether we sent data, to maintain the target frequency
        m_lastStateSendTime = currentTime;
    }
}

// --- Distribute Newly Serialized Messages to Per-Socket Send Buffers ---
// This function is called frequently by the main loop to move completed FlatBuffers
// messages from the internal outgoing queue into the per-socket send buffers.
void NetworkEngine::distributeOutgoingMessages() {
    std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);

    while (!m_outgoingNetMessages.empty()) {
        OutgoingNetMessage& msgToSend = m_outgoingNetMessages.front();

        // Distribute this message to all connected peer sockets' send buffers
        std::lock_guard<std::mutex> connectedLock(m_connectedPeerSocketsMutex);
        std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);

        for (SOCKET sock : m_connectedPeerSockets) {
            // Append this message to the socket's send buffer.
            // This is more robust than overwriting, allowing multiple messages to be queued.
            m_sendBuffers[sock].buffer.insert(m_sendBuffers[sock].buffer.end(),
                msgToSend.flatbufferData.begin(),
                msgToSend.flatbufferData.end());
            // Ensure bytesSent is 0 for a new message being added if the buffer was previously empty
            // If the buffer already had data, bytesSent tracks the progress for the *entire* buffer.
            // No change needed to bytesSent here, it's handled by processSocketSend(sock).
        }

        m_outgoingNetMessages.pop(); // Remove the message from the internal queue
    }
}


// --- Attempt to Send Pending Data from a Specific Socket's Send Buffer ---
// This function is called by the main loop when select indicates a socket is writable
// or after a message is added to the send buffer.
// Returns true if the socket should be closed due to an error.
bool NetworkEngine::processSocketSend(SOCKET sock) {
    std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);

    auto it = m_sendBuffers.find(sock);
    if (it == m_sendBuffers.end()) {
        // No pending data for this socket
        return false;
    }

    SocketSendBuffer& sockBuffer = it->second;

    // Calculate how many bytes are remaining to send
    size_t remainingBytes = sockBuffer.buffer.size() - sockBuffer.bytesSent;

    if (remainingBytes > 0) {
        // Attempt to send the remaining data
        int bytesSent = send(
            sock,
            sockBuffer.buffer.data() + sockBuffer.bytesSent,
            (int)remainingBytes, // send expects int for length
            0
        );

        if (bytesSent > 0) {
            sockBuffer.bytesSent += bytesSent;
            // std::cout << "Network Thread: Sent " << bytesSent << " bytes to socket " << sock << ". Remaining: " << (sockBuffer.buffer.size() - sockBuffer.bytesSent) << std::endl;

            if (sockBuffer.bytesSent == sockBuffer.buffer.size()) {
                // Successfully sent the entire buffer
                // std::cout << "Network Thread: Finished sending buffer to socket " << sock << std::endl;
                m_sendBuffers.erase(it); // Remove the buffer
            }
        }
        else if (bytesSent == SOCKET_ERROR) {
            int err = WSAGetLastError();
            if (err != WSAEWOULDBLOCK) {
                // Fatal send error - mark socket for closure
                std::cerr << "Network Thread: send failed with error " << err << " on socket " << sock << ". Closing socket." << std::endl;
                m_sendBuffers.erase(it); // Remove the buffer
                return true; // Indicate socket should be closed
            }
            // WSAEWOULDBLOCK means the buffer is full, select should handle waiting for writability.
            // If we get WSAEWOULDBLOCK here, it means select indicated writable but send still blocked - less common but possible.
        }
    }
    else {
        // bytesSent == buffer.size(), but buffer wasn't removed? Should not happen.
        // std::cerr << "Network Thread: Warning - Send buffer for socket " << sock << " was empty but still in map. Cleaning up." << std::endl;
        m_sendBuffers.erase(it); // Clean up if somehow in a bad state
    }
    return false; // Socket should remain open
}


// --- Attempt Connection to a Single Peer ---
void NetworkEngine::connectToPeer(const std::string& peerAddrStr) {
    SOCKET connectSocket = INVALID_SOCKET;
    struct addrinfo* connectResult = NULL, connectHints;

    ZeroMemory(&connectHints, sizeof(connectHints));
    connectHints.ai_family = AF_INET;
    connectHints.ai_socktype = SOCK_STREAM;
    connectHints.ai_protocol = IPPROTO_TCP;

    size_t colonPos = peerAddrStr.find(":");
    if (colonPos == std::string::npos) {
        std::cerr << "Network Thread: Invalid peer address format: " << peerAddrStr << ". Use IP:Port" << std::endl;
        return;
    }
    std::string peerIp = peerAddrStr.substr(0, colonPos);
    std::string peerPort = peerAddrStr.substr(colonPos + 1);

    int iResult = getaddrinfo(peerIp.c_str(), peerPort.c_str(), &connectHints, &connectResult);
    if (iResult != 0) {
        std::cerr << "Network Thread: getaddrinfo for peer " << peerAddrStr << " failed: " << iResult << std::endl;
        return;
    }

    connectSocket = socket(connectResult->ai_family, connectResult->ai_socktype, connectResult->ai_protocol);
    if (connectSocket == INVALID_SOCKET) {
        std::cerr << "Network Thread: socket creation failed for peer " << peerAddrStr << ": " << WSAGetLastError() << std::endl;
        freeaddrinfo(connectResult);
        return;
    }

    std::cout << "Network Thread: Attempting to connect to " << peerAddrStr << "..." << std::endl;

    // Connect to peer. This is a blocking call. In a real app, handle connection attempts asynchronously.
    iResult = connect(connectSocket, connectResult->ai_addr, (int)connectResult->ai_addrlen);
    freeaddrinfo(connectResult);

    if (iResult == SOCKET_ERROR) {
        std::cerr << "Network Thread: Connection to " << peerAddrStr << " failed with error: " << WSAGetLastError() << std::endl;
        closesocket(connectSocket);
        // Handle specific errors like connection refused differently
        return;
    }

    std::cout << "Network Thread: Successfully connected to " << peerAddrStr << ". Socket: " << connectSocket << std::endl;

    // Set connected socket to non-blocking for receiving/sending
    u_long nonBlocking = 1;
    if (ioctlsocket(connectSocket, FIONBIO, &nonBlocking) != NO_ERROR) {
        std::cerr << "Network Thread: ioctlsocket failed for connect socket " << peerAddrStr << ": " << WSAGetLastError() << ". Closing socket." << std::endl;
        closesocket(connectSocket);
        return;
    }

    // Add the successfully connected socket to our list
    {
        std::lock_guard<std::mutex> lock(m_connectedPeerSocketsMutex);
        m_connectedPeerSockets.push_back(connectSocket);
    }

    // Initialize receive buffer for this new socket
    {
        std::lock_guard<std::mutex> receiveBufferLock(m_receiveBuffersMutex);
        m_receiveBuffers[connectSocket] = { connectSocket, {} };
    }
    // Initialize send buffer for this new socket (even if empty initially)
    {
        std::lock_guard<std::mutex> sendBufferLock(m_sendBuffersMutex);
        m_sendBuffers[connectSocket] = { connectSocket, {}, 0 };
    }

    // TODO: Send a PeerJoined message about OURSELVES to this new peer, and send
    // PeerJoined messages about all *existing* peers to this new peer.
    // This requires knowing our own peer ID and the IDs of existing peers.
}

// --- Helper to Close and Cleanup a Specific Socket ---
void NetworkEngine::closeSocket(SOCKET sock) {
    // This function should be called when a socket is identified for closure
    // It needs to be careful about mutexes if called from different parts of the loop.
    // The cleanup logic in the main loop after the select iteration is safer.
    // This is a placeholder if you needed to close a socket immediately from within
    // handleDataReceive or processSocketSend.
    std::cerr << "Network Thread: closeSocket() called for socket " << sock << ". Cleanup should happen in main loop." << std::endl;
    // In a real implementation, you'd add sock to a thread-safe list of sockets to close.
}


// --- Public Method to Send Object Creation Message ---
// This would be called by the peer initiating the scene change (e.g., Main thread or Physics thread)
void NetworkEngine::sendObjectCreation(
    uint32_t id, int object_type,
    const PhysicsFlatBuffers::Networking::Vec3& initial_position,
    const PhysicsFlatBuffers::Networking::Vec4& initial_rotation,
    const PhysicsFlatBuffers::Networking::Vec3& initial_scale,
    const PhysicsFlatBuffers::Networking::Vec3& initial_velocity,
    const PhysicsFlatBuffers::Networking::Vec3& initial_angular_velocity,
    bool is_static,
    float sphere_radius, const PhysicsFlatBuffers::Networking::Vec3& box_size,
    float cylinder_radius, float cylinder_height, const PhysicsFlatBuffers::Networking::Vec3& plane_size
    // Add other shape parameters...
) {
    flatbuffers::FlatBufferBuilder builder;

    // Create the ObjectCreation table
    auto pos_fb = PhysicsFlatBuffers::Networking::Vec3(initial_position.x(), initial_position.y(), initial_position.z());
    auto rot_fb = PhysicsFlatBuffers::Networking::Vec4(initial_rotation.x(), initial_rotation.y(), initial_rotation.z(), initial_rotation.w());
    auto scale_fb = PhysicsFlatBuffers::Networking::Vec3(initial_scale.x(), initial_scale.y(), initial_scale.z());
    auto vel_fb = PhysicsFlatBuffers::Networking::Vec3(initial_velocity.x(), initial_velocity.y(), initial_velocity.z());
    auto angVel_fb = PhysicsFlatBuffers::Networking::Vec3(initial_angular_velocity.x(), initial_angular_velocity.y(), initial_angular_velocity.z());
    auto box_size_fb = PhysicsFlatBuffers::Networking::Vec3(box_size.x(), box_size.y(), box_size.z());
    auto plane_size_fb = PhysicsFlatBuffers::Networking::Vec3(plane_size.x(), plane_size.y(), plane_size.z());


    auto obj_creation_offset = PhysicsFlatBuffers::Networking::CreateObjectCreation(
        builder,
        id,
        object_type,
        &pos_fb,
        &rot_fb,
        &scale_fb,
        &vel_fb,
        &angVel_fb,
        is_static,
        sphere_radius,
        &box_size_fb,
        cylinder_radius,
        cylinder_height,
        &plane_size_fb
        // Add other shape parameters...
    );

    // Build the root NetMessage containing the ObjectCreation message
    auto net_message_offset = PhysicsFlatBuffers::Networking::CreateNetMessage(
        builder,
        PhysicsFlatBuffers::Networking::Message_ObjectCreation, // Specify the type in the union
        obj_creation_offset.Union() // Get the union value
    );

    // Finish the buffer, adding the size prefix
    builder.FinishSizePrefixed(net_message_offset);

    // Get the raw buffer data
    std::vector<char> sendBuffer(builder.GetSize());
    memcpy(sendBuffer.data(), builder.GetBufferPointer(), builder.GetSize());

    // Queue this complete FlatBuffers message for sending to all peers
    {
        std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
        m_outgoingNetMessages.push({ sendBuffer });
    } // Lock released

    std::cout << "Network System: Queued ObjectCreation message for sending." << std::endl;
}

// --- Public Method to Send Peer Joined Message ---
void NetworkEngine::sendPeerJoined(uint32_t peer_id) {
    flatbuffers::FlatBufferBuilder builder;

    // Create the PeerJoined table
    auto peer_joined_offset = PhysicsFlatBuffers::Networking::CreatePeerJoined(builder, peer_id);

    // Build the root NetMessage containing the PeerJoined message
    auto net_message_offset = PhysicsFlatBuffers::Networking::CreateNetMessage(
        builder,
        PhysicsFlatBuffers::Networking::Message_PeerJoined, // Specify the type in the union
        peer_joined_offset.Union() // Get the union value
    );

    // Finish the buffer, adding the size prefix
    builder.FinishSizePrefixed(net_message_offset);

    // Get the raw buffer data
    std::vector<char> sendBuffer(builder.GetSize());
    memcpy(sendBuffer.data(), builder.GetBufferPointer(), builder.GetSize());

    // Queue this complete FlatBuffers message for sending to all peers
    {
        std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
        m_outgoingNetMessages.push({ sendBuffer });
    } // Lock released

    std::cout << "Network System: Queued PeerJoined message for sending (Peer ID: " << peer_id << ")." << std::endl;
}

// --- Public Method to Send Peer Left Message ---
void NetworkEngine::sendPeerLeft(uint32_t peer_id) {
    flatbuffers::FlatBufferBuilder builder;

    // Create the PeerLeft table
    auto peer_left_offset = PhysicsFlatBuffers::Networking::CreatePeerLeft(builder, peer_id);

    // Build the root NetMessage containing the PeerLeft message
    auto net_message_offset = PhysicsFlatBuffers::Networking::CreateNetMessage(
        builder,
        PhysicsFlatBuffers::Networking::Message_PeerLeft, // Specify the type in the union
        peer_left_offset.Union() // Get the union value
    );

    // Finish the buffer, adding the size prefix
    builder.FinishSizePrefixed(net_message_offset);

    // Get the raw buffer data
    std::vector<char> sendBuffer(builder.GetSize());
    memcpy(sendBuffer.data(), builder.GetBufferPointer(), builder.GetSize());

    // Queue this complete FlatBuffers message for sending to all peers
    {
        std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
        m_outgoingNetMessages.push({ sendBuffer });
    } // Lock released

    std::cout << "Network System: Queued PeerLeft message for sending (Peer ID: " << peer_id << ")." << std::endl;
}

// --- Public Method to Send Gravity Changed Message ---
void NetworkEngine::sendGravityChanged(const PhysicsFlatBuffers::Networking::Vec3& new_gravity) {
    flatbuffers::FlatBufferBuilder builder;

    // Create the GravityChanged table
    auto gravity_vec_fb = PhysicsFlatBuffers::Networking::Vec3(new_gravity.x(), new_gravity.y(), new_gravity.z());
    auto gravity_changed_offset = PhysicsFlatBuffers::Networking::CreateGravityChanged(builder, &gravity_vec_fb);

    // Build the root NetMessage containing the GravityChanged message
    auto net_message_offset = PhysicsFlatBuffers::Networking::CreateNetMessage(
        builder,
        PhysicsFlatBuffers::Networking::Message_GravityChanged, // Specify the type in the union
        gravity_changed_offset.Union() // Get the union value
    );

    // Finish the buffer, adding the size prefix
    builder.FinishSizePrefixed(net_message_offset);

    // Get the raw buffer data
    std::vector<char> sendBuffer(builder.GetSize());
    memcpy(sendBuffer.data(), builder.GetBufferPointer(), builder.GetSize());

    // Queue this complete FlatBuffers message for sending to all peers
    {
        std::lock_guard<std::mutex> outgoingLock(m_outgoingNetMessagesMutex);
        m_outgoingNetMessages.push({ sendBuffer });
    } // Lock released

    std::cout << "Network System: Queued GravityChanged message for sending." << std::endl;
}

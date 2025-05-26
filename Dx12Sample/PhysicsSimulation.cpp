#include "PhysicsSimulation.h"

#include "Application.h"
#include "CommandQueue.h"
#include "CommandList.h"
#include "Helpers.h"
#include "Light.h"
#include "Material.h"
#include "Window.h"

#include <wrl.h>
using namespace Microsoft::WRL;

#include <d3dx12.h>
#include <d3dcompiler.h>
#include <DirectXColors.h>

using namespace DirectX;

#include <algorithm> // For std::min and std::max.
#if defined(min)
#undef min
#endif
#include "BallToBallScenario.h"

#if defined(max)
#undef max
#endif
#include "BallToWallScenario.h"
#include "BallToCapsuleScenario.h"
#include "SpheresScenario.h"
#include "ScenarioB.h"
#include <omp.h>
#include "EmptyScenario.h"
#include "GlobalData.h"

// Clamp a value between a min and max range.
template<typename T>
constexpr const T& clamp(const T& val, const T& min, const T& max)
{
    return val < min ? min : val > max ? max : val;
}

// Builds a look-at (world) matrix from a point, up and direction vectors.
XMMATRIX XM_CALLCONV LookAtMatrix(FXMVECTOR Position, FXMVECTOR Direction, FXMVECTOR Up)
{
    assert(!XMVector3Equal(Direction, XMVectorZero()));
    assert(!XMVector3IsInfinite(Direction));
    assert(!XMVector3Equal(Up, XMVectorZero()));
    assert(!XMVector3IsInfinite(Up));

    XMVECTOR R2 = XMVector3Normalize(Direction);

    XMVECTOR R0 = XMVector3Cross(Up, R2);
    R0 = XMVector3Normalize(R0);

    XMVECTOR R1 = XMVector3Cross(R2, R0);

    XMMATRIX M(R0, R1, R2, Position);

    return M;
}

PhysicsSimulation::PhysicsSimulation(const std::wstring& name, int width, int height, bool vSync)
    : super(name, width, height, vSync)
    , m_Forward(0)
    , m_Backward(0)
    , m_Left(0)
    , m_Right(0)
    , m_Up(0)
    , m_Down(0)
    , m_Pitch(0)
    , m_Yaw(0)
    , m_Shift(false)
    , m_Width(0)
    , m_Height(0)
    , m_outgoingBuffer()
	, m_incomingBuffer()
	, m_PhysicsEngine(&m_outgoingBuffer, &m_incomingBuffer)
    , m_NetworkingEngine(&m_outgoingBuffer, &m_incomingBuffer)
	, m_simulationScheduled(false)
	, m_simulationStartTime(0.0)
{
    //m_PhysicsEngine.setAffinity(1); // Core 1
    m_PhysicsEngine.setFrequency(GlobalData::g_physicsFreq); // 120 FPS
	m_NetworkingEngine.setFrequency(GlobalData::g_networkFreq); // 60 FPS
	m_NetworkingEngine.setScnearioListener(
		[this](std::vector<std::shared_ptr<NetworkedObject>>&& objects, float gravity)
		{
			CreateEmptyScenario(std::move(objects));
			m_PhysicsEngine.setGravity(gravity);
		}
	);
    m_NetworkingEngine.setStopSimulationListener(
        [this]()
        {
            StopSimulation();
        }
    );
	m_NetworkingEngine.setStartSimulationListener(
		[this](double time)
		{
			m_simulationScheduled = true;
			m_simulationStartTime = time;
		}
	);
}

PhysicsSimulation::~PhysicsSimulation()
{
}

bool PhysicsSimulation::LoadContent()
{
    Log::Warn() << ("Loading content for PhysicsSimulation...") << std::endl;

    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    std::srand(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    GlobalData::g_clientId = rand() % 5000;

	// Load the rendering engine.
	m_RenderingEngine.LoadContent(commandQueue, commandList, device, m_pWindow.get());

    // Load shared textures
    GlobalData::g_customTexture = std::make_shared<Texture>();
    GlobalData::g_defaultTexture = std::make_shared<Texture>();
	GlobalData::g_staticTexture = std::make_shared<Texture>();

    commandList->LoadTextureFromFile(*GlobalData::g_customTexture, L"Assets/Textures/marble.dds");
    commandList->LoadTextureFromFile(*GlobalData::g_staticTexture, L"Assets/Textures/static.dds");
    commandList->LoadTextureFromFile(*GlobalData::g_defaultTexture, L"Assets/Textures/DefaultWhite.bmp");

	// Load the scenario meshes.
	GlobalData::g_sphereMesh = Mesh::CreateSphere(*commandList, 1.0f, 16);
	GlobalData::g_boxMesh = Mesh::CreateCube(*commandList, 1.0f, false);
	GlobalData::g_planeMesh = Mesh::CreatePlane(*commandList);
    GlobalData::g_capsuleMeshes.clear();

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

    return true;
}

void PhysicsSimulation::OnResize(ResizeEventArgs& e)
{
    super::OnResize(e);

    if (m_Width != e.Width || m_Height != e.Height)
    {
        m_Width = std::max(1, e.Width);
        m_Height = std::max(1, e.Height);
		m_RenderingEngine.OnResize(m_Width, m_Height);
    }
}

void PhysicsSimulation::UnloadContent()
{
    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();
    {
        std::lock_guard<std::mutex> lock(m_ScenarioMutex);
        if (m_CurrentScenario)
        {
            m_CurrentScenario->onUnload(*commandList);
        }
        m_CurrentScenario.reset();
        m_CurrentScenario = nullptr;
    }

    m_simulationScheduled = false;
    m_simulationStartTime = 0.0;

    m_PhysicsEngine.stop();
	m_NetworkingEngine.stop();
    m_allNetworkedObjects.clear();

    GlobalData::g_customTexture.reset();
	GlobalData::g_staticTexture.reset();
    GlobalData::g_defaultTexture.reset();
    GlobalData::g_sphereMesh.reset();
	GlobalData::g_boxMesh.reset();
	GlobalData::g_planeMesh.reset();

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);
}

void PhysicsSimulation::OnUpdate(UpdateEventArgs& e)
{
    static uint64_t frameCount = 0;
    static double totalTime = 0.0;

    super::OnUpdate(e);

    if (m_simulationScheduled) {
        double now = GlobalData::getTimestamp();
        if (now >= m_simulationStartTime) {
            m_PhysicsEngine.start();
            m_simulationScheduled = false;
        }
    }

    totalTime += e.ElapsedTime;
    frameCount++;

    if (totalTime > .2)
    {
        GlobalData::g_renderingFPS = static_cast<double>(frameCount) / totalTime;
        GlobalData::g_physicsDt = static_cast<double>(m_PhysicsEngine.getDeltaTime());
        GlobalData::g_networkDt = static_cast<double>(m_NetworkingEngine.getDeltaTime());

        frameCount = 0;
        totalTime = 0.0;
    }

    // Update the camera.
    float speedMultipler = (m_Shift ? 16.0f : 4.0f);

    XMVECTOR cameraTranslate = XMVectorSet(m_Right - m_Left, 0.0f, m_Forward - m_Backward, 1.0f) * speedMultipler * static_cast<float>(e.ElapsedTime);
    XMVECTOR cameraPan = XMVectorSet(0.0f, m_Up - m_Down, 0.0f, 1.0f) * speedMultipler * static_cast<float>(e.ElapsedTime);
    XMVECTOR cameraRotation = XMQuaternionRotationRollPitchYaw(XMConvertToRadians(m_Pitch), XMConvertToRadians(m_Yaw), 0.0f);

    m_RenderingEngine.UpdateCamera(cameraTranslate, cameraPan, cameraRotation);
    m_RenderingEngine.UpdateLights(e.ElapsedTime);
}

void XM_CALLCONV PhysicsSimulation::ComputeMatrices(FXMMATRIX model, CXMMATRIX view, CXMMATRIX viewProjection, Mat& mat)
{
    mat.ModelMatrix = model;
    mat.ModelViewMatrix = model * view;
    mat.InverseTransposeModelViewMatrix = XMMatrixTranspose(XMMatrixInverse(nullptr, mat.ModelViewMatrix));
    mat.ModelViewProjectionMatrix = model * viewProjection;
}

void PhysicsSimulation::OnRender(RenderEventArgs& e)
{
    super::OnRender(e);

    // Wrap the member function in a lambda to match the std::function signature.  
    auto renderCallback = [this](CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
        {
			std::lock_guard<std::mutex> lock(m_ScenarioMutex);
            if (m_CurrentScenario)
            {
                m_CurrentScenario->onRender(commandList, viewMatrix, viewProjectionMatrix);
            }
        };

    auto guiCallback = [this]()
    {
        OnGUI();
    };

	m_RenderingEngine.Render(renderCallback, guiCallback, m_pWindow.get());
}

namespace {
    bool g_AllowFullscreenToggle = true;
    bool g_AllowConsoleToggle = true;
}

void PhysicsSimulation::OnKeyPressed(KeyEventArgs& e)
{
    super::OnKeyPressed(e);

    if (!ImGui::GetIO().WantCaptureKeyboard)
    {
        switch (e.Key)
        {
        case KeyCode::Escape:
            Application::Get().Quit(0);
            break;
        case KeyCode::Enter:
            if (e.Alt)
            {
        case KeyCode::F11:
            if (g_AllowFullscreenToggle)
            {
                m_pWindow->ToggleFullscreen();
                g_AllowFullscreenToggle = false;
            }
            break;
            }
        case KeyCode::V:
            m_pWindow->ToggleVSync();
            break;
        case KeyCode::R:
            // Reset camera transform
            m_RenderingEngine.ResetCamera();
            m_Pitch = 0.0f;
            m_Yaw = 0.0f;
            break;
        case KeyCode::Up:
        case KeyCode::W:
            m_Forward = 1.0f;
            break;
        case KeyCode::Left:
        case KeyCode::A:
            m_Left = 1.0f;
            break;
        case KeyCode::Down:
        case KeyCode::S:
            m_Backward = 1.0f;
            break;
        case KeyCode::Right:
        case KeyCode::D:
            m_Right = 1.0f;
            break;
        case KeyCode::Q:
            m_Down = 1.0f;
            break;
        case KeyCode::E:
            m_Up = 1.0f;
            break;
        case KeyCode::ShiftKey:
            m_Shift = true;
            break;
        case KeyCode::F2:
			g_AllowConsoleToggle = !g_AllowConsoleToggle;
			break;
		default:
			break;
        }
    }
}

void PhysicsSimulation::OnKeyReleased(KeyEventArgs& e)
{
    super::OnKeyReleased(e);

    switch (e.Key)
    {
    case KeyCode::Enter:
        if (e.Alt)
        {
    case KeyCode::F11:
        g_AllowFullscreenToggle = true;
        }
        break;
    case KeyCode::Up:
    case KeyCode::W:
        m_Forward = 0.0f;
        break;
    case KeyCode::Left:
    case KeyCode::A:
        m_Left = 0.0f;
        break;
    case KeyCode::Down:
    case KeyCode::S:
        m_Backward = 0.0f;
        break;
    case KeyCode::Right:
    case KeyCode::D:
        m_Right = 0.0f;
        break;
    case KeyCode::Q:
        m_Down = 0.0f;
        break;
    case KeyCode::E:
        m_Up = 0.0f;
        break;
    case KeyCode::ShiftKey:
        m_Shift = false;
        break;
	default:
		break;
    }
}

void PhysicsSimulation::OnMouseMoved(MouseMotionEventArgs& e)
{
    super::OnMouseMoved(e);
    
    if (!ImGui::GetIO().WantCaptureMouse)
    {
        if (e.RightButton)
        {
        	constexpr float mouseSpeed = 0.1f;
            m_Pitch += static_cast<float>(e.RelY) * mouseSpeed;
            m_Pitch = clamp(m_Pitch, -90.0f, 90.0f);
            m_Yaw += static_cast<float>(e.RelX) * mouseSpeed;
        }
    }
}


void PhysicsSimulation::OnMouseWheel(MouseWheelEventArgs& e)
{
    if (!ImGui::GetIO().WantCaptureMouse)
    {
        m_RenderingEngine.AddFov(-e.WheelDelta);
    }
}

void PhysicsSimulation::OnGUI()
{
    static bool showDemoWindow = false;
    static bool showOptions = true;
    static bool showEngineStats = true;
    static bool showNetworking = true;
    static bool showSimulationControl = true;

    // Main Menu Bar
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Exit", "Esc"))
                Application::Get().Quit();
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("ImGui Demo", nullptr, &showDemoWindow);
            ImGui::MenuItem("Physics Options", nullptr, &showOptions);
            ImGui::MenuItem("Engine Stats", nullptr, &showEngineStats);
            ImGui::MenuItem("Networking", nullptr, &showNetworking);
            ImGui::MenuItem("Simulation Control", nullptr, &showSimulationControl);
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Options")) {
            bool vSync = m_pWindow->IsVSync();
            if (ImGui::MenuItem("V-Sync", "V", &vSync))
                m_pWindow->SetVSync(vSync);

            bool fullscreen = m_pWindow->IsFullScreen();
            if (ImGui::MenuItem("Full screen", "Alt+Enter", &fullscreen))
                m_pWindow->SetFullscreen(fullscreen);

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }

    // Engine Stats
    if (showEngineStats) {
        ImGui::Begin("Engine Stats", &showEngineStats);
        ImGui::Text("FPS: %.2f (%.2f ms)", GlobalData::g_renderingFPS, 1000.0 / GlobalData::g_renderingFPS);

        auto ShowEngineData = [](const char* name, ThreadedSystem& engine, const float& dt, int& freq) {
            float hz = 1.0f / dt;
            float ms = dt * 1000.0f;
            ImGui::Text("%s FPS: %.2f (%.2f ms)", name, hz, ms);
            if (ImGui::InputInt((std::string(name) + " Timestep (Hz)").c_str(), &freq, 1, 2)) {
                freq = std::clamp(freq, 1, 300);
                engine.setFrequency(freq);
            }
        };

        float physicsDeltaTime = m_PhysicsEngine.getSimulationDeltaTime();
        if (ImGui::InputFloat("Physics delta time", &physicsDeltaTime, 0.005f, 0.01f, "%.3f")) {
            physicsDeltaTime = std::clamp(physicsDeltaTime, 0.001f, 0.1f);
            m_PhysicsEngine.setSimulationDeltaTime(physicsDeltaTime);
        }

        ShowEngineData("Physics", m_PhysicsEngine, GlobalData::g_physicsDt, GlobalData::g_physicsFreq);
        ShowEngineData("Network", m_NetworkingEngine, GlobalData::g_networkDt, GlobalData::g_networkFreq);
        ImGui::End();
    }


    ImGui::Begin("Main Panel Stats");

    // Show simulation countdown
    if (m_simulationScheduled) {
        ImGui::Text("Simulation scheduled in: %.2f sec", m_simulationStartTime - GlobalData::getTimestamp());
    }

    // First we should connect
    if (!m_NetworkingEngine.isRunning()) {
        ImGui::Text("Client ID: %d", GlobalData::g_clientId);

        static char clientName[64] = "Client A";
        if (ImGui::InputText("Client Name", clientName, sizeof(clientName)))
            GlobalData::g_clientName = clientName;

        DirectX::XMFLOAT4 clientColor = GlobalData::g_clientColor;
        if (ImGui::ColorEdit4("Client Color", &clientColor.x))
            GlobalData::g_clientColor = clientColor;

        static int port = GlobalData::g_listenPort;
        if (ImGui::InputInt("Port", &port))
            GlobalData::g_listenPort = port;

        if (ImGui::Button("Connect to network")) {
            m_NetworkingEngine.initializeSockets(port);
            m_NetworkingEngine.start();
        }
    }
    else {
        if (!m_PhysicsEngine.isRunning()) {
            if (ImGui::CollapsingHeader("Simulation")) {
                if (ImGui::TreeNode("Scenarios List")) {
                    ImGui::BeginChild("Child2", ImVec2(0, 0), true);
                    if (ImGui::Button("Spheres Scenario", ImVec2(-1.0f, 0.0f))) {
                        ChangeScenario(0);
                    }
                    if (ImGui::Button("Scenario B", ImVec2(-1.0f, 0.0f)))
                        ChangeScenario(1);

					if (ImGui::Button("Ball To Capsule Scenario", ImVec2(-1.0f, 0.0f))) {
						ChangeScenario(2);
					}

                    ImGui::EndChild();
                    ImGui::TreePop();
                }
            }
            if (m_CurrentScenario && !m_simulationScheduled) {
                if (ImGui::Button("Broadcast Scenario")) {
                    BroadCastCurrentScenarioCreate();
                }
                if (m_allNetworkedObjects.size() > 0 && m_CurrentScenario->isReady() && ImGui::Button("Start Simulation")) {
                    double localNow = GlobalData::getTimestamp();
                    double startTime = localNow + 2.0;

                    m_simulationScheduled = true;
                    m_simulationStartTime = startTime;
                    m_NetworkingEngine.scheduleSimulationStart(startTime);
                }
            }
        }
        else {
			if (ImGui::Button("Stop Simulation")) {
				StopSimulation();
				m_NetworkingEngine.stopSimulation();
			}
        }

        if (ImGui::CollapsingHeader("Network")) {
            ImGui::Checkbox("Use better Prediction", &m_clientPrediction);
            ImGui::TextColored({GlobalData::g_clientColor.x, GlobalData::g_clientColor.y, GlobalData::g_clientColor.z, 1},
                "Peer ID: %d, Client Name: %s", GlobalData::g_clientId, GlobalData::g_clientName.c_str());
			ImGui::Spacing();

            auto peers = m_NetworkingEngine.getPeersInfo();
            if (!peers.empty()) {
                ImGui::Text("Connected Peers:");
                for (const auto& [peer, ping] : peers) {
                    ImGui::TextColored(ImColor(peer.color.x, peer.color.y, peer.color.z), "Peer ID: %d, Client Name: %s, Ping: %f", peer.peer_id, peer.client_name.c_str(), ping);
                }
            }
			else {
				ImGui::Text("No connected peers. Listening..");
			}
        }

        if (ImGui::CollapsingHeader("Physics")) {
            static float gravity = m_PhysicsEngine.getGravity();
            bool gravityEnabled = m_PhysicsEngine.isGravityEnabled();
            bool reversed = gravity < 0;

			bool ghostMode = PhysicsEngine::ghostModeEnabled();
			if (ImGui::Checkbox("Ghost Mode", &ghostMode)) {
				m_PhysicsEngine.setGhostMode(ghostMode);
			}

            ImGui::Text("Simulation Time");
			ImGui::SameLine();
			ImGui::Text("%d sec", GlobalData::g_tick);

            ImGui::Text("Gravity:  %f", m_PhysicsEngine.getGravity());
            ImGui::SameLine();

            if (ImGui::SliderFloat("##Gravity", &gravity, -20.0f, 20.0f, "%.5f m/s^2"));
			if (ImGui::Button("Set Gravity")) {
                m_PhysicsEngine.setGravity(gravity);
                m_NetworkingEngine.changeGravity(gravity);
			}

            ImGui::SameLine();
        }

        {
            std::lock_guard<std::mutex> lock(m_ScenarioMutex);
            if (m_CurrentScenario && ImGui::CollapsingHeader("Simulation Info")) {
                int objectCount = static_cast<int>(m_CurrentScenario->getPhysicsObjects().size());
                ImGui::Text("Objects in scenario: %d", objectCount);
            }
        }

        if (ImGui::Button("Reset Camera")) {
            m_RenderingEngine.ResetCamera();
            m_Pitch = 0.0f;
            m_Yaw = 0.0f;
        }
    }
    ImGui::End();

    if (showDemoWindow)
        ImGui::ShowDemoWindow(&showDemoWindow);

    if (m_CurrentScenario)
        m_CurrentScenario->drawImGui();

    RenderFixedBottomLogConsole(Log::GetEntries());
}


void PhysicsSimulation::ChangeScenario(int index)
{
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();
	std::lock_guard<std::mutex> lock(m_ScenarioMutex);
    if (m_CurrentScenario)
    {
        m_PhysicsEngine.stop();
        m_CurrentScenario->onUnload(*commandList);
        m_PhysicsEngine.clearBodies();
        m_allNetworkedObjects.clear();
    }
    switch (index)
    {
    case 0:
        m_CurrentScenario = std::make_unique<SpheresScenario>();
        break;
    case 1:
        m_CurrentScenario = std::make_unique<ScenarioB>();
        break;
	case 2:
		m_CurrentScenario = std::make_unique<BallToCapsuleScenario>();
		break;
    default:
        break;
    }

    if (m_CurrentScenario)
        m_CurrentScenario->onLoad(*commandList);
    

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);
}

void PhysicsSimulation::CreateEmptyScenario(std::vector <std::shared_ptr<NetworkedObject>>&& networkedObjects)
{
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    m_allNetworkedObjects = std::move(networkedObjects);

    std::lock_guard<std::mutex> lock(m_ScenarioMutex);

	auto physicsObjects = std::vector<std::shared_ptr<PhysicsObject>>();
	for (auto& netObj : m_allNetworkedObjects)
	{
		auto physicsObject = netObj->getObject();
		if (physicsObject)
		{
			physicsObjects.push_back(physicsObject);
		}
	}

    if (m_CurrentScenario)
    {
        m_PhysicsEngine.stop();
        m_CurrentScenario->onUnload(*commandList);
        m_PhysicsEngine.clearBodies();
    }

    m_CurrentScenario = std::make_unique<EmptyScenario>(std::move(physicsObjects));
    if (m_CurrentScenario) {
        m_CurrentScenario->onLoad(*commandList);
    }
    for (const auto& body : m_allNetworkedObjects)
    {
        m_PhysicsEngine.addBody(body);
        if (body->getOwnerId() != GlobalData::g_clientId)
        {
            m_unownedObjects[body->getId()] = body.get();
        }
    }
    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

	m_CurrentScenario->setReady(true);
}


void PhysicsSimulation::BroadCastCurrentScenarioCreate()
{
    if (m_PhysicsEngine.isRunning()) {
		// m_lastError = "Cannot create scenario while physics engine is running.";
		return;
    }
    {
        std::lock_guard<std::mutex> lock(m_ScenarioMutex);
        if (m_CurrentScenario)
        {
            m_PhysicsEngine.clearBodies();
            m_allNetworkedObjects.clear();
            m_NetworkingEngine.assignOwnersAndBroadcastScenarioCreate(m_CurrentScenario->getPhysicsObjects(), m_PhysicsEngine.getGravity(), m_allNetworkedObjects);
            for (const auto& obj : m_allNetworkedObjects)
            {
                m_PhysicsEngine.addBody(obj);
            }
			m_CurrentScenario->setReady(true);
        }
    }
}

void PhysicsSimulation::StopSimulation()
{
	m_PhysicsEngine.stop();
	m_simulationScheduled = false;
	m_simulationStartTime = 0.0;

	m_outgoingBuffer.clear();
	m_incomingBuffer.clear();
    {
        std::lock_guard<std::mutex> lock(m_ScenarioMutex);
        if (m_CurrentScenario)
        {
            auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
            auto commandList = commandQueue->GetCommandList();
            m_CurrentScenario->onUnload(*commandList);
            auto fenceValue = commandQueue->ExecuteCommandList(commandList);
            commandQueue->WaitForFenceValue(fenceValue);

            m_CurrentScenario.reset();
            m_CurrentScenario = nullptr;
        }
    }
	m_PhysicsEngine.clearBodies();
	m_allNetworkedObjects.clear();
}

void PhysicsSimulation::RenderFixedBottomLogConsole(const std::vector<LogEntry>& entries) {
#if USE_LOGGER
	if (!g_AllowConsoleToggle)
		return;
    const float consoleHeight = 200.0f;  // Fixed height
    const ImVec2 windowPadding = ImVec2(10, 10);

    ImGuiIO& io = ImGui::GetIO();
    ImVec2 windowPos = ImVec2(0, io.DisplaySize.y - consoleHeight);
    ImVec2 windowSize = ImVec2(io.DisplaySize.x, consoleHeight);

    ImGui::SetNextWindowPos(windowPos);
    ImGui::SetNextWindowSize(windowSize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.0f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse;

    ImGui::Begin("Log Console", nullptr, flags);

    static ImGuiTextFilter filter;
    filter.Draw("Filter");

    ImGui::SameLine();
    if (ImGui::Button("Clear"))
    {
        Log::Clear();
        ImGui::SetScrollHere(0.0f);
    }


    if (ImGui::BeginChild("ConsoleScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_AlwaysVerticalScrollbar)) {
        for (const auto& entry : entries) {
            if (!filter.PassFilter(entry.message.c_str()))
                continue;

            ImVec4 color;
            switch (entry.level) {
            case LogLevel::Log:    color = ImVec4(1, 1, 1, 1); break;
            case LogLevel::Warning:color = ImVec4(1, 1, 0, 1); break;
            case LogLevel::Error:  color = ImVec4(1, 0.4f, 0.4f, 1); break;
            }

            ImGui::PushStyleColor(ImGuiCol_Text, color);
            ImGui::TextWrapped("%s", entry.message.c_str());
            ImGui::PopStyleColor();
        }

        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHere(1.0f);
    }
    ImGui::EndChild();

    ImGui::PopStyleVar(2);
    ImGui::End();
#endif
}
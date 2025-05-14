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
#include "ScenarioA.h"
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
{
    //m_PhysicsEngine.setAffinity(1); // Core 1
	m_PhysicsEngine.setFrequency(120); // 120 FPS
	m_NetworkingEngine.setFrequency(60); // 60 FPS
	m_NetworkingEngine.setScnearioListener(
		[this](std::vector<std::shared_ptr<PhysicsObject>>&& objects, float gravity)
		{
			CreateEmptyScenario(std::move(objects));
            AssignOwnedNonOwnedObjects();
			m_PhysicsEngine.setGravity(gravity);
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
	m_sharedData = m_NetworkingEngine.getSharedData();

    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    std::srand(static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count()));
    GlobalData::g_clientId = rand() % 100;

	// Load the rendering engine.
	m_RenderingEngine.LoadContent(commandQueue, commandList, device, m_pWindow.get());

    // Load shared textures
    GlobalData::g_customTexture = std::make_shared<Texture>();
    GlobalData::g_defaultTexture = std::make_shared<Texture>();

    commandList->LoadTextureFromFile(*GlobalData::g_customTexture, L"Assets/Textures/earth.dds");
    commandList->LoadTextureFromFile(*GlobalData::g_defaultTexture, L"Assets/Textures/DefaultWhite.bmp");

	// Load the scenario meshes.
	GlobalData::g_sphereMesh = Mesh::CreateSphere(*commandList, 1.0f, 16);
	GlobalData::g_boxMesh = Mesh::CreateCube(*commandList, 1.0f, false);
	GlobalData::g_planeMesh = Mesh::CreatePlane(*commandList);

    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);

	// Add update shared simulation data callback.
	m_PhysicsEngine.addPostUpdateListener([this](float deltaTime)
	{
		UpdatePostPhysicsSimulation();
	});

    m_PhysicsEngine.addBeforeUpdateListener([this](float deltaTime)
	{
		UpdateBeforePhysicsSimulation();
	});

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
    m_PhysicsEngine.stop();
	m_NetworkingEngine.stop();
    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    m_ScenarioMutex.lock();
	if (m_CurrentScenario)
	{
		m_CurrentScenario->onUnload(*commandList);
	}

    m_ScenarioMutex.unlock();

    // Unload textures
    GlobalData::g_customTexture = nullptr;
	GlobalData::g_defaultTexture = nullptr;
	GlobalData::g_sphereMesh = nullptr;
	GlobalData::g_boxMesh = nullptr;
	GlobalData::g_planeMesh = nullptr;

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
    else {
        if (m_PhysicsEngine.isRunning()) {
            GlobalData::g_networkFPS += e.ElapsedTime;
        }
    }

    totalTime += e.ElapsedTime;
    frameCount++;

    if (totalTime > 1.0)
    {
        GlobalData::g_physicsFPS = static_cast<double>(frameCount) / totalTime;
        GlobalData::g_physicsFPS = static_cast<double>(m_PhysicsEngine.getDeltaTime());
        if(GlobalData::g_physicsFPS > 0)
		{
            GlobalData::g_physicsFPS = 1.0 / GlobalData::g_physicsFPS;
		}

        char buffer[512];
        sprintf_s(buffer, "FPS: %f\n", GlobalData::g_renderingFPS);
        OutputDebugStringA(buffer);

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

    m_ScenarioMutex.lock();
    // Wrap the member function in a lambda to match the std::function signature.  
    auto renderCallback = [this](CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
    {
		if (m_CurrentScenario)
		{
			m_CurrentScenario->onRender(commandList, viewMatrix, viewProjectionMatrix);
		}
    };
	m_ScenarioMutex.unlock();

    auto guiCallback = [this]()
    {
        OnGUI();
    };

	m_RenderingEngine.Render(renderCallback, guiCallback, m_pWindow.get());
}

// Helper to display a little (?) mark which shows a tooltip when hovered.
static void ShowHelpMarker(const char* desc)
{
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered())
    {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

namespace {
    bool g_AllowFullscreenToggle = true;
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

    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Exit", "Esc"))
            {
                Application::Get().Quit();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("ImGui Demo", nullptr, &showDemoWindow);
            ImGui::MenuItem("Physics Options", nullptr, &showOptions);

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Options")) {
            bool vSync = m_pWindow->IsVSync();
            if (ImGui::MenuItem("V-Sync", "V", &vSync)) {
                m_pWindow->SetVSync(vSync);
            }

            bool fullscreen = m_pWindow->IsFullScreen();
            if (ImGui::MenuItem("Full screen", "Alt+Enter", &fullscreen)) {
                m_pWindow->SetFullscreen(fullscreen);
            }

            // Change scenario
			if (ImGui::MenuItem("Create Scenario A")) {
				ChangeScenario(0);
			}
            if (ImGui::MenuItem("Create Scenario B")) {
				ChangeScenario(1);
            }

            ImGui::EndMenu();
        }
        {
            // Add a new ImGui window to control the timestep of each engine.
            ImGui::Begin("Engine Timesteps");
            char buffer[256];
            float dt;
            float hz;
            float ms;
            {
                sprintf_s(buffer, _countof(buffer), "FPS: %.2f (%.2f ms)  ", GlobalData::g_renderingFPS, 1.0 / GlobalData::g_renderingFPS * 1000.0);
                ImGui::Text(buffer);
            }
            {
                float physicsDeltaTime = m_PhysicsEngine.getSimulationDeltaTime();
                if (ImGui::InputFloat("Physics delta time", &physicsDeltaTime, 0.005f, 0.01f, "%.3f")) {
                    physicsDeltaTime = std::max(0.001f, std::min(0.1f, physicsDeltaTime));
                    m_PhysicsEngine.setSimulationDeltaTime(physicsDeltaTime);
                }
            }
            {
                dt = m_PhysicsEngine.getDeltaTime();
                hz = 1.0f / dt;
                ms = dt * 1000.0f;

                sprintf_s(buffer, _countof(buffer), "Physics FPS: %.2f (%.2f ms)", hz, ms);
                ImGui::Text("%s", buffer);
                int physicsFrequency = m_PhysicsEngine.getFrequency();
                if (ImGui::InputInt("Physics Engine Timestep (s)", &physicsFrequency, 1, 2)) {
					physicsFrequency = std::max(1, std::min(300, physicsFrequency));
                    m_PhysicsEngine.setFrequency(physicsFrequency);
                }
            }
            {
				dt = m_NetworkingEngine.getDeltaTime();
				hz = 1.0f / dt;
				ms = dt * 1000.0f;

				sprintf_s(buffer, _countof(buffer), "Network FPS: %.2f (%.2f ms)", hz, ms);
				ImGui::Text("%s", buffer);
				int networkFrequency = m_NetworkingEngine.getFrequency();
                if (ImGui::InputInt("Network Engine Timestep (s)", &networkFrequency, 1, 2)) {
                    networkFrequency = std::max(1, std::min(300, networkFrequency));
                    m_NetworkingEngine.setFrequency(networkFrequency);
                }
            }
            ImGui::End();
        }


        ImGui::EndMainMenuBar();
    }

    ImGui::Begin("Network");
    // Show two buttons to either start client or host
    if (!m_NetworkingEngine.isRunning()) {

        if (ImGui::CollapsingHeader("Client Config")) {

            ImGui::Text("Client ID: %d", GlobalData::g_clientId);

            static char clientName[64] = "";
            if (ImGui::InputText("Client Name", clientName, sizeof(clientName))) {
                GlobalData::g_clientName = clientName;
            }

            DirectX::XMFLOAT4 clientColor = GlobalData::g_clientColor;
            if (ImGui::ColorEdit4("Client Color", &clientColor.x)) {
                GlobalData::g_clientColor = clientColor;
            }
            static int port = GlobalData::g_listenPort;
            if (ImGui::InputInt("Port", &port)) {
                GlobalData::g_listenPort = port;
            }
            if (ImGui::Button("Start Host")) {
                m_NetworkingEngine.initializeSockets(port);
                m_NetworkingEngine.start();
            }
        }
    }
    else {
        auto peers = m_NetworkingEngine.getPeersInfo();
		if (peers.size() > 0)
		{
            ImGui::Text("Peer Info:");
            for (auto& peer : peers)
            {
                auto color = ImColor(peer.color.x, peer.color.y, peer.color.z);
                ImGui::TextColored(color, "Peer ID: %d, Client Name: %s", peer.peer_id, peer.client_name.c_str());
            }
            if (ImGui::Button("Disconnect")) {
                m_NetworkingEngine.stop();
            }
		}
		else {
			ImGui::Text("Listening for peers..");
		}
        if (m_CurrentScenario && ImGui::Button("Broadcast Scenario")) {
            BroadCastCurrentScenarioCreate();
        }
        if (!m_PhysicsEngine.isRunning()) {
            if (!m_simulationScheduled && ImGui::Button("Start Simulation")) {
                double localNow = GlobalData::getTimestamp();
                double startTime = localNow + 2.0;

                m_simulationScheduled = true;
                m_simulationStartTime = startTime;

                m_NetworkingEngine.scheduleSimulationStart(startTime);
            }
            else if (m_simulationScheduled) {
                ImGui::Text("Simulation scheduled at %.2f", m_simulationStartTime - GlobalData::getTimestamp());
            }
        }

    }
    ImGui::End();

    /*
    // Show last error at the top screen
    if (!m_lastError.empty()) {
        ImGui::SetNextWindowPos(ImVec2(0, m_Height - 30));
        ImGui::SetNextWindowBgAlpha(0.5f);
        ImGui::Begin("Error", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
        ImGui::TextColored(ImColor(1.0f, 0.0f, 0.0f), "%s", m_lastError.c_str());
        ImGui::End();
    }
    */

    if (showDemoWindow)
    {
        ImGui::ShowDemoWindow(&showDemoWindow);
    }

    if (showOptions)
    {
        ImGui::Begin("Physics Control", &showOptions);
        {
            // Gravity Control
			float gravity = m_PhysicsEngine.getGravity();
			bool gravityEnabled = m_PhysicsEngine.isGravityEnabled();
            bool reversed = gravity < 0;

            ImGui::Checkbox("##Client Prediction", &m_clientPrediction);

			ImGui::Text("Gravity");
			ImGui::SameLine();
            if (ImGui::Checkbox("##Gravity Enabled", &gravityEnabled)) {
                m_PhysicsEngine.toggleGravity(gravityEnabled);
            }
            ImGui::SameLine();
            ImGui::Text("Reversed");

			ImGui::SameLine();
            if (ImGui::Checkbox("##Gravity Reversed", &reversed)) {
				gravity = -gravity;
                m_PhysicsEngine.setGravity(gravity);
            }
            if (ImGui::SliderFloat("##Gravity", &gravity, 0.0f, 20.0f, "%.5f m/s^2")) {
                m_PhysicsEngine.setGravity(gravity);
            }
			
			ImGui::SameLine();
			ShowHelpMarker("Gravity acceleration in m/s^2");
			// Reset camera
			if (ImGui::Button("Reset Camera"))
			{
				m_RenderingEngine.ResetCamera();
				m_Pitch = 0.0f;
				m_Yaw = 0.0f;
			}
        }

        ImGui::End();
    }
}

void PhysicsSimulation::ChangeScenario(int index)
{
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();
	m_ScenarioMutex.lock();

	if (m_CurrentScenario)
	{
		m_PhysicsEngine.stop();
		m_CurrentScenario->onUnload(*commandList);
		m_PhysicsEngine.clearBodies();
	}
    switch (index)
    {
	case 0:
		m_CurrentScenario = std::make_unique<ScenarioA>();
		break;
	case 1:
		m_CurrentScenario = std::make_unique<ScenarioB>();
		break;
    default:
        break;
    }
    if (m_CurrentScenario) {

        m_CurrentScenario->onLoad(*commandList);

        for (auto& body : m_CurrentScenario->getPhysicsObjects())
        {
            m_PhysicsEngine.addBody(body);
        }
    }

    m_ScenarioMutex.unlock();
    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);
}

void PhysicsSimulation::CreateEmptyScenario(std::vector <std::shared_ptr<PhysicsObject>>&& objects)
{
	auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
	auto commandList = commandQueue->GetCommandList();
	if (m_CurrentScenario)
	{
		m_PhysicsEngine.stop();
		m_CurrentScenario->onUnload(*commandList);
		m_PhysicsEngine.clearBodies();
	}
	m_CurrentScenario = std::make_unique<EmptyScenario>(std::move(objects));
	m_ownedObjects.clear();
	if (m_CurrentScenario) {
		m_CurrentScenario->onLoad(*commandList);
		for (auto& body : m_CurrentScenario->getPhysicsObjects())
		{
			m_PhysicsEngine.addBody(body);
			if (body->getOwnerId() == GlobalData::g_clientId)
			{
				m_ownedObjects.push_back(body.get());
			}
			else
			{
				m_unownedObjects[body->getId()] = body.get();
			}
		}
	}
	auto fenceValue = commandQueue->ExecuteCommandList(commandList);
	commandQueue->WaitForFenceValue(fenceValue);
}

void PhysicsSimulation::AssignOwnedNonOwnedObjects()
{
	m_ScenarioMutex.lock();
	if (m_CurrentScenario)
	{
		auto objects = m_CurrentScenario->getPhysicsObjects();
		for (size_t i = 0; i < objects.size(); ++i)
		{
			if (objects[i]->getId() == GlobalData::g_clientId)
			{
				m_ownedObjects[objects[i]->getId()] = objects[i].get();
			}
			else
			{
				m_unownedObjects[objects[i]->getId()] = objects[i].get();
			}
		}
	}
	m_ScenarioMutex.unlock();
}


void PhysicsSimulation::BroadCastCurrentScenarioCreate()
{
    if (m_PhysicsEngine.isRunning()) {
		m_lastError = "Cannot create scenario while physics engine is running.";
    }
	if (m_CurrentScenario)
	{
		m_NetworkingEngine.assignOwnersAndBroadcastScenarioCreate("Scenario", m_CurrentScenario->getPhysicsObjects(), m_PhysicsEngine.getGravity(), m_ownedObjects, m_unownedObjects.data());
	}
}

void PhysicsSimulation::UpdatePostPhysicsSimulation()
{
    m_sharedData->m_outgoingMutex.lock();
    m_sharedData->m_outgoingObjectStates[1].clear();
    for (auto& object : m_ownedObjects) {
        if (object) {
            ObjectUpdate objectUpdate;
            auto id = object->getId();

            DirectX::XMFLOAT3 position;
			DirectX::XMFLOAT4 rotation;
            DirectX::XMFLOAT3 velocity;
			DirectX::XMFLOAT3 angularVelocity;

			DirectX::XMStoreFloat3(&position, object->getTransform().GetPosition(1));
			DirectX::XMStoreFloat4(&rotation, object->getTransform().GetRotationQuaternion(1));
			DirectX::XMStoreFloat3(&velocity, object->getVelocity(1));
			DirectX::XMStoreFloat3(&angularVelocity, object->getAngularVelocity(1));

			objectUpdate.object_id = id;
            objectUpdate.position = position;
            objectUpdate.rotation = rotation;
            objectUpdate.velocity = velocity;
            objectUpdate.angular_velocity = angularVelocity;
			m_sharedData->m_outgoingObjectStates[1].push_back(objectUpdate);
        }
    }
    m_sharedData->m_ownedObjectsDirty = true;
    m_sharedData->m_outgoingMutex.unlock();
}

void PhysicsSimulation::UpdateBeforePhysicsSimulation()
{
    double currentSimTime = GlobalData::g_simulationTime;

    m_sharedData->m_incomingMutex.lock();
    if (m_sharedData->m_receivedNewAuthoritativeData) {
        for (auto& [id, history] : m_sharedData->m_objectUpdateHistory) {
            auto* objectPtr = m_unownedObjects[id];
            if (!objectPtr || history.size() < 1)
                continue;
            if (m_clientPrediction) {
                // Find two updates surrounding current time
                ObjectUpdate* prev = nullptr;
                ObjectUpdate* next = nullptr;

                for (size_t i = 1; i < history.size(); ++i) {
                    if (history[i - 1].simulation_time <= currentSimTime &&
                        history[i].simulation_time >= currentSimTime) {
                        prev = &history[i - 1];
                        next = &history[i];
                        break;
                    }
                }

                if (prev && next) {
                    // Interpolate between prev and next (linear interpolation)
                    float t = static_cast<float>((currentSimTime - prev->simulation_time) /
                        (next->simulation_time - prev->simulation_time));

                    // Interpolate position, rotation, velocity
                    DirectX::XMVECTOR pos1 = XMLoadFloat3(&prev->position);
                    DirectX::XMVECTOR pos2 = XMLoadFloat3(&next->position);
                    DirectX::XMVECTOR interpolatedPos = DirectX::XMVectorLerp(pos1, pos2, t);

                    DirectX::XMVECTOR rot1 = XMLoadFloat4(&prev->rotation);
                    DirectX::XMVECTOR rot2 = XMLoadFloat4(&next->rotation);
                    DirectX::XMVECTOR interpolatedRot = DirectX::XMQuaternionSlerp(rot1, rot2, t);

					DirectX::XMVECTOR vel1 = XMLoadFloat3(&prev->velocity);
					DirectX::XMVECTOR vel2 = XMLoadFloat3(&next->velocity);
					DirectX::XMVECTOR interpolatedVel = DirectX::XMVectorLerp(vel1, vel2, t);

                    objectPtr->getTransform().SetPosition(interpolatedPos, 0, true);
                    objectPtr->getTransform().SetRotationQuaternion(interpolatedRot, 0, true);

					DirectX::XMVECTOR angVel1 = XMLoadFloat3(&prev->angular_velocity);
					DirectX::XMVECTOR angVel2 = XMLoadFloat3(&next->angular_velocity);
					DirectX::XMVECTOR interpolatedAngVel = DirectX::XMVectorLerp(angVel1, angVel2, t);

					objectPtr->setVelocity(interpolatedVel, 0);
					objectPtr->setVelocity(interpolatedVel, 1);

					objectPtr->setAngularVelocity(interpolatedAngVel, 0);
					objectPtr->setAngularVelocity(interpolatedAngVel, 1);
				}
                else {
                    const auto& latest = history.back();
                    objectPtr->getTransform().SetPosition(XMLoadFloat3(&latest.position), 0, true);
                    objectPtr->getTransform().SetRotationQuaternion(XMLoadFloat4(&latest.rotation), 0, true);

                    objectPtr->setVelocity(XMLoadFloat3(&latest.velocity), 0);
                    objectPtr->setVelocity(XMLoadFloat3(&latest.velocity), 1);

                    objectPtr->setAngularVelocity(XMLoadFloat3(&latest.angular_velocity), 0);
                    objectPtr->setAngularVelocity(XMLoadFloat3(&latest.angular_velocity), 1);
                }
            }
            else {
                // Extrapolate using the latest entry
                const auto& latest = history.back();
                objectPtr->getTransform().SetPosition(XMLoadFloat3(&latest.position), 0, true);
                objectPtr->getTransform().SetRotationQuaternion(XMLoadFloat4(&latest.rotation), 0, true);

				objectPtr->setVelocity(XMLoadFloat3(&latest.velocity), 0);
                objectPtr->setVelocity(XMLoadFloat3(&latest.velocity), 1);
				objectPtr->setAngularVelocity(XMLoadFloat3(&latest.angular_velocity), 0);
                objectPtr->setAngularVelocity(XMLoadFloat3(&latest.angular_velocity), 1);
            }
        }

        m_sharedData->m_receivedNewAuthoritativeData = false;
    }
    m_sharedData->m_incomingMutex.unlock();
}


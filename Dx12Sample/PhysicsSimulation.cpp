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

    m_PhysicsEngine.setAffinity(1); // Core 1
}

PhysicsSimulation::~PhysicsSimulation()
{
}

bool PhysicsSimulation::LoadContent()
{
    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    m_Scenarios.push_back(std::make_unique<ScenarioA>());
    m_Scenarios.push_back(std::make_unique<ScenarioB>());
    m_Scenarios.push_back(std::make_unique<BallToCapsuleScenario>());
    m_Scenarios.push_back(std::make_unique<BallToWallScenario>());
	m_Scenarios.push_back(std::make_unique<BallToBallScenario>());
    m_CurrentScenario = 0u;

    for (auto& scenario : m_Scenarios)
    {
        scenario->onLoad(*commandList);
    }
	for (auto& body : m_Scenarios[m_CurrentScenario]->getPhysicsObjects())
	{
		m_PhysicsEngine.addBody(body);
	}
	// Load the rendering engine.
	m_RenderingEngine.LoadContent(commandQueue, commandList, device, m_pWindow.get());
    m_PhysicsEngine.start();


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
    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

    for (auto& scenario : m_Scenarios)
    {
        scenario->onUnload(*commandList);
    }
    auto fenceValue = commandQueue->ExecuteCommandList(commandList);
    commandQueue->WaitForFenceValue(fenceValue);
}

namespace {
    double g_FPS = 0.0;
}

void PhysicsSimulation::OnUpdate(UpdateEventArgs& e)
{
    static uint64_t frameCount = 0;
    static double totalTime = 0.0;

    super::OnUpdate(e);

    totalTime += e.ElapsedTime;
    frameCount++;

    if (totalTime > 1.0)
    {
        g_FPS = static_cast<double>(frameCount) / totalTime;

        char buffer[512];
        sprintf_s(buffer, "FPS: %f\n", g_FPS);
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

    // Wrap the member function in a lambda to match the std::function signature.  
    auto renderCallback = [this](CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
    {
        m_Scenarios[m_CurrentScenario]->onRender(commandList, viewMatrix, viewProjectionMatrix);
    };

    // Wrap OnGUI in a lambda to match the std::function signature.  
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

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Exit", "Esc"))
            {
                Application::Get().Quit();
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View"))
        {
            ImGui::MenuItem("ImGui Demo", nullptr, &showDemoWindow);
            ImGui::MenuItem("Physics Options", nullptr, &showOptions);

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Options"))
        {
            bool vSync = m_pWindow->IsVSync();
            if (ImGui::MenuItem("V-Sync", "V", &vSync))
            {
                m_pWindow->SetVSync(vSync);
            }

            bool fullscreen = m_pWindow->IsFullScreen();
            if (ImGui::MenuItem("Full screen", "Alt+Enter", &fullscreen))
            {
                m_pWindow->SetFullscreen(fullscreen);
            }

            // Change scenario
			if (ImGui::MenuItem("Next Scenario", "Ctrl+N"))
			{
				ChangeScenario(m_CurrentScenario+1);
			}

            ImGui::EndMenu();
        }
        {
            char buffer[256];
            sprintf_s(buffer, _countof(buffer), "FPS: %.2f (%.2f ms)  ", g_FPS, 1.0 / g_FPS * 1000.0);
            auto fpsTextSize = ImGui::CalcTextSize(buffer);
            ImGui::SameLine(ImGui::GetWindowWidth() - fpsTextSize.x);
            ImGui::Text(buffer);
        }

        ImGui::EndMainMenuBar();
    }

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
	m_CurrentScenario = index % m_Scenarios.size();
}

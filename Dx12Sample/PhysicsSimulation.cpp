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

#if defined(max)
#undef max
#endif

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
    , m_ScissorRect(CD3DX12_RECT(0, 0, LONG_MAX, LONG_MAX))
    , m_Viewport(CD3DX12_VIEWPORT(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height)))
    , m_Forward(0)
    , m_Backward(0)
    , m_Left(0)
    , m_Right(0)
    , m_Up(0)
    , m_Down(0)
    , m_Pitch(0)
    , m_Yaw(0)
    , m_AnimateLights(false)
    , m_Shift(false)
    , m_Width(0)
    , m_Height(0)
{

    XMVECTOR cameraPos = XMVectorSet(0, 5, -20, 1);
    XMVECTOR cameraTarget = XMVectorSet(0, 5, 0, 1);
    XMVECTOR cameraUp = XMVectorSet(0, 1, 0, 0);

    m_Camera.set_LookAt(cameraPos, cameraTarget, cameraUp);

    m_pAlignedCameraData = static_cast<CameraData*>(_aligned_malloc(sizeof(CameraData), 16));

    m_pAlignedCameraData->m_InitialCamPos = m_Camera.get_Translation();
    m_pAlignedCameraData->m_InitialCamRot = m_Camera.get_Rotation();
}

PhysicsSimulation::~PhysicsSimulation()
{
    _aligned_free(m_pAlignedCameraData);
}

bool PhysicsSimulation::LoadContent()
{
    auto device = Application::Get().GetDevice();
    auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
    auto commandList = commandQueue->GetCommandList();

	m_Scenarios.push_back(std::make_unique<ConcreteScenarioA>());
	m_Scenarios.push_back(std::make_unique<ConcreteScenarioB>());
    m_CurrentScenario = 1u;

    for (auto& scenario : m_Scenarios)
    {
        scenario->onLoad(*commandList);
    }


    // Load the vertex shader.
    ComPtr<ID3DBlob> vertexShaderBlob;
    ThrowIfFailed(D3DReadFileToBlob(FULL_SHADER_PATH(L"VertexShader.cso"), &vertexShaderBlob));

    // Load the pixel shader.
    ComPtr<ID3DBlob> pixelShaderBlob;
    ThrowIfFailed(D3DReadFileToBlob(FULL_SHADER_PATH(L"PixelShader.cso"), &pixelShaderBlob));

    // Create a root signature.
    D3D12_FEATURE_DATA_ROOT_SIGNATURE featureData = {};
    featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_1;
    if (FAILED(device->CheckFeatureSupport(D3D12_FEATURE_ROOT_SIGNATURE, &featureData, sizeof(featureData))))
    {
        featureData.HighestVersion = D3D_ROOT_SIGNATURE_VERSION_1_0;
    }

    // Allow input layout and deny unnecessary access to certain pipeline stages.
    D3D12_ROOT_SIGNATURE_FLAGS rootSignatureFlags =
        D3D12_ROOT_SIGNATURE_FLAG_ALLOW_INPUT_ASSEMBLER_INPUT_LAYOUT |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_HULL_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_DOMAIN_SHADER_ROOT_ACCESS |
        D3D12_ROOT_SIGNATURE_FLAG_DENY_GEOMETRY_SHADER_ROOT_ACCESS;

    CD3DX12_DESCRIPTOR_RANGE1 descriptorRage(D3D12_DESCRIPTOR_RANGE_TYPE_SRV, 1, 2);

    CD3DX12_ROOT_PARAMETER1 rootParameters[RootParameters::NumRootParameters];
    rootParameters[RootParameters::MatricesCB].InitAsConstantBufferView(0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_VERTEX);
    rootParameters[RootParameters::MaterialCB].InitAsConstantBufferView(0, 1, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_PIXEL);
    rootParameters[RootParameters::LightPropertiesCB].InitAsConstants(sizeof(LightProperties) / 4, 1, 0, D3D12_SHADER_VISIBILITY_PIXEL);
    rootParameters[RootParameters::PointLights].InitAsShaderResourceView(0, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_PIXEL);
    rootParameters[RootParameters::SpotLights].InitAsShaderResourceView(1, 0, D3D12_ROOT_DESCRIPTOR_FLAG_NONE, D3D12_SHADER_VISIBILITY_PIXEL);
    rootParameters[RootParameters::Textures].InitAsDescriptorTable(1, &descriptorRage, D3D12_SHADER_VISIBILITY_PIXEL);

    CD3DX12_STATIC_SAMPLER_DESC linearRepeatSampler(0, D3D12_FILTER_COMPARISON_MIN_MAG_MIP_LINEAR);
    CD3DX12_STATIC_SAMPLER_DESC anisotropicSampler(0, D3D12_FILTER_ANISOTROPIC);

    CD3DX12_VERSIONED_ROOT_SIGNATURE_DESC rootSignatureDescription;
    rootSignatureDescription.Init_1_1(RootParameters::NumRootParameters, rootParameters, 1, &linearRepeatSampler, rootSignatureFlags);

    m_RootSignature.SetRootSignatureDesc(rootSignatureDescription.Desc_1_1, featureData.HighestVersion);

    // Setup the pipeline state.
    struct PipelineStateStream
    {
        CD3DX12_PIPELINE_STATE_STREAM_ROOT_SIGNATURE pRootSignature;
        CD3DX12_PIPELINE_STATE_STREAM_INPUT_LAYOUT InputLayout;
        CD3DX12_PIPELINE_STATE_STREAM_PRIMITIVE_TOPOLOGY PrimitiveTopologyType;
        CD3DX12_PIPELINE_STATE_STREAM_VS VS;
        CD3DX12_PIPELINE_STATE_STREAM_PS PS;
        CD3DX12_PIPELINE_STATE_STREAM_DEPTH_STENCIL_FORMAT DSVFormat;
        CD3DX12_PIPELINE_STATE_STREAM_RENDER_TARGET_FORMATS RTVFormats;
        CD3DX12_PIPELINE_STATE_STREAM_SAMPLE_DESC SampleDesc;
    } pipelineStateStream;

    // sRGB formats provide free gamma correction!
    DXGI_FORMAT backBufferFormat = DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    DXGI_FORMAT depthBufferFormat = DXGI_FORMAT_D32_FLOAT;

    // Check the best multisample quality level that can be used for the given back buffer format.
    DXGI_SAMPLE_DESC sampleDesc = Application::Get().GetMultisampleQualityLevels(backBufferFormat, D3D12_MAX_MULTISAMPLE_SAMPLE_COUNT);

    D3D12_RT_FORMAT_ARRAY rtvFormats = {};
    rtvFormats.NumRenderTargets = 1;
    rtvFormats.RTFormats[0] = backBufferFormat;

    pipelineStateStream.pRootSignature = m_RootSignature.GetRootSignature().Get();
    pipelineStateStream.InputLayout = { VertexPositionNormalTexture::InputElements, VertexPositionNormalTexture::InputElementCount };
    pipelineStateStream.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
    pipelineStateStream.VS = CD3DX12_SHADER_BYTECODE(vertexShaderBlob.Get());
    pipelineStateStream.PS = CD3DX12_SHADER_BYTECODE(pixelShaderBlob.Get());
    pipelineStateStream.DSVFormat = depthBufferFormat;
    pipelineStateStream.RTVFormats = rtvFormats;
    pipelineStateStream.SampleDesc = sampleDesc;

    D3D12_PIPELINE_STATE_STREAM_DESC pipelineStateStreamDesc = {
        sizeof(PipelineStateStream), &pipelineStateStream
    };
    ThrowIfFailed(device->CreatePipelineState(&pipelineStateStreamDesc, IID_PPV_ARGS(&m_PipelineState)));

    // Create an off-screen render target with a single color buffer and a depth buffer.
    auto colorDesc = CD3DX12_RESOURCE_DESC::Tex2D(backBufferFormat,
        m_Width, m_Height,
        1, 1,
        sampleDesc.Count, sampleDesc.Quality,
        D3D12_RESOURCE_FLAG_ALLOW_RENDER_TARGET);
    D3D12_CLEAR_VALUE colorClearValue;
    colorClearValue.Format = colorDesc.Format;
    colorClearValue.Color[0] = 0.4f;
    colorClearValue.Color[1] = 0.6f;
    colorClearValue.Color[2] = 0.9f;
    colorClearValue.Color[3] = 1.0f;

    auto colorTexture = Texture(colorDesc, &colorClearValue, TextureUsage::RenderTarget, L"Color Render Target");

    // Create a depth buffer.
    auto depthDesc = CD3DX12_RESOURCE_DESC::Tex2D(depthBufferFormat,
        m_Width, m_Height,
        1, 1,
        sampleDesc.Count, sampleDesc.Quality,
        D3D12_RESOURCE_FLAG_ALLOW_DEPTH_STENCIL);
    D3D12_CLEAR_VALUE depthClearValue;
    depthClearValue.Format = depthDesc.Format;
    depthClearValue.DepthStencil = { 1.0f, 0 };

    auto depthTexture = Texture(depthDesc, &depthClearValue, TextureUsage::Depth, L"Depth Render Target");

    // Attach the textures to the render target.
    m_RenderTarget.AttachTexture(AttachmentPoint::Color0, colorTexture);
    m_RenderTarget.AttachTexture(AttachmentPoint::DepthStencil, depthTexture);

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

        float aspectRatio = m_Width / (float)m_Height;
        m_Camera.set_Projection(45.0f, aspectRatio, 0.1f, 100.0f);

        m_Viewport = CD3DX12_VIEWPORT(0.0f, 0.0f,
            static_cast<float>(m_Width), static_cast<float>(m_Height));

        m_RenderTarget.Resize(m_Width, m_Height);
    }
}

void PhysicsSimulation::UnloadContent()
{
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

	m_Scenarios[m_CurrentScenario]->onUpdate(e.ElapsedTime);

    // Update the camera.
    float speedMultipler = (m_Shift ? 16.0f : 4.0f);

    XMVECTOR cameraTranslate = XMVectorSet(m_Right - m_Left, 0.0f, m_Forward - m_Backward, 1.0f) * speedMultipler * static_cast<float>(e.ElapsedTime);
    XMVECTOR cameraPan = XMVectorSet(0.0f, m_Up - m_Down, 0.0f, 1.0f) * speedMultipler * static_cast<float>(e.ElapsedTime);
    m_Camera.Translate(cameraTranslate, Space::Local);
    m_Camera.Translate(cameraPan, Space::Local);

    XMVECTOR cameraRotation = XMQuaternionRotationRollPitchYaw(XMConvertToRadians(m_Pitch), XMConvertToRadians(m_Yaw), 0.0f);
    m_Camera.set_Rotation(cameraRotation);

    XMMATRIX viewMatrix = m_Camera.get_ViewMatrix();

    constexpr int numPointLights = 4;
    constexpr int numSpotLights = 4;

    static const XMVECTORF32 LightColors[] =
    {
        Colors::White, Colors::Orange, Colors::Yellow, Colors::Green, Colors::Blue, Colors::Indigo, Colors::Violet, Colors::White
    };

    static float lightAnimTime = 0.0f;
    if (m_AnimateLights)
    {
        lightAnimTime += static_cast<float>(e.ElapsedTime) * 0.5f * XM_PI;
    }

    constexpr float radius = 8.0f;
    constexpr float offset = 2.0f * XM_PI / numPointLights;
    constexpr float offset2 = offset + (offset / 2.0f);

    // Setup the light buffers.
    m_PointLights.resize(numPointLights);
    for (int i = 0; i < numPointLights; ++i)
    {
        PointLight& l = m_PointLights[i];

        l.PositionWS = {
            static_cast<float>(std::sin(lightAnimTime + offset * i)) * radius,
            9.0f,
            static_cast<float>(std::cos(lightAnimTime + offset * i)) * radius,
            1.0f
        };
        XMVECTOR positionWS = XMLoadFloat4(&l.PositionWS);
        XMVECTOR positionVS = XMVector3TransformCoord(positionWS, viewMatrix);
        XMStoreFloat4(&l.PositionVS, positionVS);

        l.Color = XMFLOAT4(LightColors[i]);
        l.ConstantAttenuation = 1.0f;
        l.LinearAttenuation = 0.08f;
        l.QuadraticAttenuation = 0.0f;
    }

    m_SpotLights.resize(numSpotLights);
    for (int i = 0; i < numSpotLights; ++i)
    {
        SpotLight& l = m_SpotLights[i];

        l.PositionWS = {
            static_cast<float>(std::sin(lightAnimTime + offset * i + offset2)) * radius,
            9.0f,
            static_cast<float>(std::cos(lightAnimTime + offset * i + offset2)) * radius,
            1.0f
        };
        XMVECTOR positionWS = XMLoadFloat4(&l.PositionWS);
        XMVECTOR positionVS = XMVector3TransformCoord(positionWS, viewMatrix);
        XMStoreFloat4(&l.PositionVS, positionVS);

        XMVECTOR directionWS = XMVector3Normalize(XMVectorSetW(XMVectorNegate(positionWS), 0));
        XMVECTOR directionVS = XMVector3Normalize(XMVector3TransformNormal(directionWS, viewMatrix));
        XMStoreFloat4(&l.DirectionWS, directionWS);
        XMStoreFloat4(&l.DirectionVS, directionVS);

        l.Color = XMFLOAT4(LightColors[numPointLights + i]);
        l.SpotAngle = XMConvertToRadians(45.0f);
        l.ConstantAttenuation = 1.0f;
        l.LinearAttenuation = 0.08f;
        l.QuadraticAttenuation = 0.0f;
    }
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

    const auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_DIRECT);
    const auto commandList = commandQueue->GetCommandList();

    // Clear the render targets.
    {
	    constexpr float clearColor[] = { 0.4f, 0.6f, 0.9f, 1.0f };
        commandList->ClearTexture(m_RenderTarget.GetTexture(AttachmentPoint::Color0), clearColor);
        commandList->ClearDepthStencilTexture(m_RenderTarget.GetTexture(AttachmentPoint::DepthStencil), D3D12_CLEAR_FLAG_DEPTH);
    }

    commandList->SetPipelineState(m_PipelineState);
    commandList->SetGraphicsRootSignature(m_RootSignature);

    // Upload lights
    LightProperties lightProps;
    lightProps.NumPointLights = static_cast<uint32_t>(m_PointLights.size());
    lightProps.NumSpotLights = static_cast<uint32_t>(m_SpotLights.size());

    commandList->SetGraphics32BitConstants(RootParameters::LightPropertiesCB, lightProps);
    commandList->SetGraphicsDynamicStructuredBuffer(RootParameters::PointLights, m_PointLights);
    commandList->SetGraphicsDynamicStructuredBuffer(RootParameters::SpotLights, m_SpotLights);

    commandList->SetViewport(m_Viewport);
    commandList->SetScissorRect(m_ScissorRect);

    commandList->SetRenderTarget(m_RenderTarget);

    // Setup camera
    const XMMATRIX viewMatrix = m_Camera.get_ViewMatrix();
    const XMMATRIX viewProjectionMatrix = viewMatrix * m_Camera.get_ProjectionMatrix();

    m_Scenarios[m_CurrentScenario]->onRender(*commandList, viewMatrix, viewProjectionMatrix);

    commandQueue->ExecuteCommandList(commandList);

    // Render GUI.
    OnGUI();

    // Present
    m_pWindow->Present(m_RenderTarget.GetTexture(AttachmentPoint::Color0));
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
            m_Camera.set_Translation(m_pAlignedCameraData->m_InitialCamPos);
            m_Camera.set_Rotation(m_pAlignedCameraData->m_InitialCamRot);
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
        case KeyCode::Space:
            m_AnimateLights = !m_AnimateLights;
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
        auto fov = m_Camera.get_FoV();

        fov -= e.WheelDelta;
        fov = clamp(fov, 12.0f, 90.0f);

        m_Camera.set_FoV(fov);

        char buffer[256];
        sprintf_s(buffer, "FoV: %f\n", fov);
        OutputDebugStringA(buffer);
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
            ImGui::MenuItem("Tonemapping", nullptr, &showOptions);

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
        ImGui::Begin("Tonemapping", &showOptions);
        {
        }

        if (ImGui::Button("Reset to Defaults"))
        {
        }

        ImGui::End();
    }
}

void PhysicsSimulation::ChangeScenario(int index)
{
	m_CurrentScenario = index % m_Scenarios.size();
}

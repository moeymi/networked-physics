#pragma once
#include "pch.h"
#include "Camera.h"
#include "Light.h"
#include "RenderTarget.h"
#include "CommandQueue.h"
#include "CommandList.h"
#include "RootSignature.h"
#include "Texture.h"
#include "Window.h"
#include "Helpers.h"

#include <DirectXMath.h>
#include <d3d12.h>
#include <wrl.h>
#include <functional>
#include <memory>


class CommandList;

class RenderingEngine {
public:
    using RenderCallback = std::function<void(CommandList&, const DirectX::XMMATRIX&, const DirectX::XMMATRIX&)>;
    using GUICallback = std::function<void()>;

    RenderingEngine();

    void LoadContent(std::shared_ptr<CommandQueue>& commandQueue,
        std::shared_ptr<CommandList>& commandList,
        Microsoft::WRL::ComPtr<ID3D12Device2>& device,
        Window* window);

    void OnResize(uint32_t width, uint32_t height);
	void UpdateCamera(const DirectX::XMVECTOR& cameraTranslate, const DirectX::XMVECTOR& cameraPan, const DirectX::XMVECTOR& cameraRotation);
    void AddFov(const float& fov);
    void ResetCamera();
    void UpdateLights(const float& deltaTime);
    void Render(RenderCallback renderCallback, GUICallback guiCallback, Window* window);

    Camera& GetCamera() { return m_Camera; }
    void SetClearColor(const DirectX::XMFLOAT4& color) { m_ClearColor = color; }
    void SetLightAnimation(bool animate) { m_AnimateLights = animate; }

private:
    void CreateRenderTarget(Window* window, const DXGI_SAMPLE_DESC& sampleDesc);

    DirectX::XMMATRIX m_ViewMatrix;

    Camera m_Camera;
    struct alignas(16) CameraData
    {
        DirectX::XMVECTOR m_InitialCamPos;
        DirectX::XMVECTOR m_InitialCamRot;
    };
    std::unique_ptr<CameraData> m_pAlignedCameraData;

    RenderTarget m_RenderTarget;
    RootSignature m_RootSignature;
    Microsoft::WRL::ComPtr<ID3D12PipelineState> m_PipelineState;

    CD3DX12_VIEWPORT m_Viewport;
    CD3DX12_RECT m_ScissorRect;

    DirectX::XMFLOAT4 m_ClearColor;
    std::vector<PointLight> m_PointLights;
    std::vector<SpotLight> m_SpotLights;

    bool m_AnimateLights = false;
    float m_LightAnimTime = 0.0f;
};
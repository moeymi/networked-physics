#pragma once
#include "pch.h"
#include <DirectXMath.h>
#include <memory>
#include <vector>
#include <functional>

class Transform final {
private:
    struct State {
        DirectX::XMVECTOR position = { 0, 0, 0, 0 };
        DirectX::XMVECTOR rotationQuaternion = DirectX::XMQuaternionIdentity();;
        DirectX::XMVECTOR scale = { 1.0f, 1.0f, 1.0f };;

        DirectX::XMVECTOR lookDirection = { 0.0f, 0.0f, 1.0f, 0.0f };
        DirectX::XMVECTOR upDirection = { 0.0f, 1.0f, 0.0f, 0.0f };

        DirectX::XMMATRIX worldMatrix;

        bool positionScaleDirty = true;
        bool rotationDirty = true;
    } m_states[2];

    std::shared_ptr<Transform> parent = nullptr;

    bool isStatic = false;

    // Listeners for dirtifying changes
    std::vector<std::function<void()>> dirtyListeners;

    void CalculateWorldMatrix(const int& bufferIndex, const bool& bothBuffers);
    void CalculateNewLookDirection(const int& bufferIndex, const bool& bothBuffers);
    void CleanDirty(const int& bufferIndex, const bool& bothBuffers);

public:
    Transform() = default;
    Transform(const Transform&) = delete;

    void SetParent(const std::shared_ptr<Transform>& parent);

    bool IsStatic() const;
    void SetStatic(bool value);

    void swapStates();

    DirectX::XMVECTOR GetPosition(const int& bufferIndex) const;
    void SetPosition(const DirectX::XMFLOAT3& pos, const int& bufferIndex, const bool& bothBuffers = false);
    void SetPosition(const DirectX::XMVECTOR& pos, const int& bufferIndex, const bool& bothBuffers = false);

    const DirectX::XMVECTOR& GetLocalPosition(const int& bufferIndex) const;
    void SetLocalPosition(const DirectX::XMFLOAT3& pos, const int& bufferIndex, const bool& bothBuffers = false);
    void SetLocalPosition(const DirectX::XMVECTOR& pos, const int& bufferIndex, const bool& bothBuffers = false);

    DirectX::XMVECTOR GetScale(const int& bufferIndex) const;
    void SetScale(const DirectX::XMFLOAT3& s, const int& bufferIndex, const bool& bothBuffers = false);
    void SetScale(const DirectX::XMVECTOR& s, const int& bufferIndex, const bool& bothBuffers = false);

    const DirectX::XMVECTOR& GetLocalScale(const int& bufferIndex) const;
    void SetLocalScale(const DirectX::XMFLOAT3& s, const int& bufferIndex, const bool& bothBuffers = false);
    void SetLocalScale(const DirectX::XMVECTOR& s, const int& bufferIndex, const bool& bothBuffers = false);

    DirectX::XMVECTOR GetRotationQuaternion(const int& bufferIndex) const;
    void SetRotationQuaternion(const DirectX::XMVECTOR& rot, const int& bufferIndex, const bool& bothBuffers = false);

    const DirectX::XMVECTOR& GetLocalRotationQuaternion(const int& bufferIndex) const;
    void SetLocalRotationQuaternion(const DirectX::XMVECTOR& rot, const int& bufferIndex, const bool& bothBuffers = false);

    [[nodiscard]] DirectX::XMFLOAT3 GetRotationEulerAngles(const int& bufferIndex) const;
    void SetRotationEulerAngles(const DirectX::XMFLOAT3& rot, const int& bufferIndex, const bool& bothBuffers = false);

    [[nodiscard]] DirectX::XMFLOAT3 GetLocalRotationEulerAngles(const int& bufferIndex) const;
    void SetLocalRotationEulerAngles(const DirectX::XMFLOAT3& rot, const int& bufferIndex, const bool& bothBuffers = false);

    const DirectX::XMVECTOR& GetLookDirection(const int& bufferIndex) const;
    const DirectX::XMVECTOR& GetUpDirection(const int& bufferIndex) const;

    void Translate(const DirectX::XMFLOAT3& translation, const int& bufferIndex, const bool& bothBuffers = false);
    void Translate(const DirectX::XMVECTOR& translation, const int& bufferIndex, const bool& bothBuffers = false);

    void RotateEulerAngles(const DirectX::XMFLOAT3& rotation, const int& bufferIndex, const bool& bothBuffers = false);

    void RotateQuaternion(const DirectX::XMVECTOR& rotation, const int& bufferIndex, const bool& bothBuffers = false);

    void LookAt(const DirectX::XMVECTOR& target, const DirectX::XMVECTOR& up, const int& bufferIndex, const bool& bothBuffers = false);

    DirectX::XMMATRIX GetWorldMatrix(const int& bufferIndex);

    void AddDirtyListener(const std::function<void()>& listener) {
        dirtyListeners.push_back(listener);
    }

};
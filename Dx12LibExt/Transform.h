#pragma once
#include <DirectXMath.h>
#include <memory>
#include <vector>
#include <functional>

class Transform final {
private:
    DirectX::XMVECTOR position;
    DirectX::XMVECTOR rotationQuaternion = DirectX::XMQuaternionIdentity();
    DirectX::XMVECTOR scale = { 1.0f, 1.0f, 1.0f };

    DirectX::XMVECTOR lookDirection = { 0.0f, 0.0f, 1.0f, 0.0f };
    DirectX::XMVECTOR upDirection = { 0.0f, 1.0f, 0.0f, 0.0f };

    std::shared_ptr<Transform> parent = nullptr;

    bool isStatic = false;

    DirectX::XMMATRIX worldMatrix;

    // Listeners for dirtifying changes
    std::vector<std::function<void()>> dirtyListeners;

	bool positionScaleDirty = true;
	bool rotationDirty = true;

	void CalculateWorldMatrix();
    void CalculateNewLookDirection();
    void CleanDirty();

public:
    Transform() = default;
    Transform(const Transform&) = delete;

    void SetParent(const std::shared_ptr<Transform>& parent);

    bool IsStatic() const;
    void SetStatic(bool value);

    DirectX::XMVECTOR GetPosition() const;
    void SetPosition(const DirectX::XMFLOAT3& pos);
    void SetPosition(const DirectX::XMVECTOR& pos);

    const DirectX::XMVECTOR& GetLocalPosition() const;
    void SetLocalPosition(const DirectX::XMFLOAT3& pos);
    void SetLocalPosition(const DirectX::XMVECTOR& pos);

    DirectX::XMVECTOR GetScale() const;
    void SetScale(const DirectX::XMFLOAT3& s);
    void SetScale(const DirectX::XMVECTOR& s);

    const DirectX::XMVECTOR& GetLocalScale() const;
    void SetLocalScale(const DirectX::XMFLOAT3& s);
    void SetLocalScale(const DirectX::XMVECTOR& s);

    DirectX::XMVECTOR GetRotationQuaternion() const;
    void SetRotationQuaternion(const DirectX::XMVECTOR& rot);

    const DirectX::XMVECTOR& GetLocalRotationQuaternion() const;
    void SetLocalRotationQuaternion(const DirectX::XMVECTOR& rot);

    [[nodiscard]] DirectX::XMFLOAT3 GetRotationEulerAngles() const;
    void SetRotationEulerAngles(const DirectX::XMFLOAT3& rot);

    [[nodiscard]] DirectX::XMFLOAT3 GetLocalRotationEulerAngles() const;
    void SetLocalRotationEulerAngles(const DirectX::XMFLOAT3& rot);

    const DirectX::XMVECTOR& GetLookDirection() const;
    const DirectX::XMVECTOR& GetUpDirection() const;

    void Translate(const DirectX::XMFLOAT3& translation);
    void Translate(const DirectX::XMVECTOR& translation);

    void RotateEulerAngles(const DirectX::XMFLOAT3& rotation);

    void RotateQuaternion(const DirectX::XMVECTOR& rotation);

    void LookAt(const DirectX::XMVECTOR& target, const DirectX::XMVECTOR& up = { 0.0f, 1.0f, 0.0f, 0.0f });

    DirectX::XMMATRIX GetWorldMatrix();

    void AddDirtyListener(const std::function<void()>& listener) {
        dirtyListeners.push_back(listener);
    }

};
#pragma once
#include "pch.h"
#include "Collider.h"

class SphereCollider : public Collider {
public:
    SphereCollider(float radius);

    ColliderType getType() const { return ColliderType::Sphere; }

    inline AABB getWorldAABB(Transform* transform) const;

    inline DirectX::XMMATRIX getInertiaTensor(float mass);
    inline DirectX::XMMATRIX getInverseInertiaTensor(float mass);

    float getRadius() const { return m_radius; }
	void setRadius(float radius) { m_radius = radius; }
private:
    float m_radius;
    DirectX::XMVECTOR m_radiusVec;

    bool m_inertiaReady = false;
    bool m_invInertiaReady = false;
    float m_inertiaVal = 0.0f;
    DirectX::XMMATRIX m_inertiaTensor;
    DirectX::XMMATRIX m_inverseInertiaTensor;
};

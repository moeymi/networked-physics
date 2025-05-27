#pragma once
#include "pch.h"
#include "Collider.h"

class SphereCollider : public Collider {
public:
    SphereCollider(float radius);

    ColliderType getType() const { return ColliderType::Sphere; }

    // mark inline so the compiler can really optimize away the function call
    inline AABB getWorldAABB(Transform* transform) const;

    // inertia calls are cheap now—no matrix invert in the hot path
    inline DirectX::XMMATRIX getInertiaTensor(float mass);
    inline DirectX::XMMATRIX getInverseInertiaTensor(float mass);

    float getRadius() const { return m_radius; }

private:
    float m_radius;
    // cache a replicated-vector of the raw radius so you do it only once
    DirectX::XMVECTOR m_radiusVec;

    bool m_inertiaReady = false;
    bool m_invInertiaReady = false;
    float m_inertiaVal = 0.0f;
    DirectX::XMMATRIX m_inertiaTensor;
    DirectX::XMMATRIX m_inverseInertiaTensor;
};

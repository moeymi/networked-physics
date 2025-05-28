#include "SphereCollider.h"

using namespace DirectX;

SphereCollider::SphereCollider(float radius)
    : m_radius(radius)
    , m_radiusVec(XMVectorReplicate(radius))
{
}

inline AABB SphereCollider::getWorldAABB(Transform* transform) const
{
    XMMATRIX W = transform->GetWorldMatrix(1);
    XMVECTOR center = W.r[3];
    return AABB(center - m_radiusVec,
        center + m_radiusVec);
}

inline XMMATRIX SphereCollider::getInertiaTensor(float mass)
{
    if (!m_inertiaReady) {
        m_inertiaVal = (2.0f / 5.0f) * mass * m_radius * m_radius;
        m_inertiaTensor = XMMatrixScaling(m_inertiaVal,
            m_inertiaVal,
            m_inertiaVal);
        m_inertiaReady = true;
    }
    return m_inertiaTensor;
}

inline XMMATRIX SphereCollider::getInverseInertiaTensor(float mass)
{
    if (!m_invInertiaReady) {
        if (!m_inertiaReady)
            getInertiaTensor(mass);

        float invI = (m_inertiaVal > FLT_EPSILON)
            ? (1.0f / m_inertiaVal)
            : 0.0f;
        m_inverseInertiaTensor = XMMatrixScaling(invI, invI, invI);
        m_invInertiaReady = true;
    }
    return m_inverseInertiaTensor;
}

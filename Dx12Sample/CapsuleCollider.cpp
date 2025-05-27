#include "CapsuleCollider.h"
#include <Helpers.h>

CapsuleCollider::CapsuleCollider(const float& radius, const float& cylinderHeight)
    : m_radius(radius), m_cylinderHeight(cylinderHeight)
{
    // Ensure valid dimensions
    if (m_radius < 0) m_radius = 0;
    if (m_cylinderHeight < 0) m_cylinderHeight = 0;
}

ColliderType CapsuleCollider::getType() const
{
    return ColliderType::Capsule;
}

void CapsuleCollider::getLocalSegmentEndpoints(DirectX::XMVECTOR& out_p1, DirectX::XMVECTOR& out_p2) const
{
    using namespace DirectX;
    float halfCylinderHeight = m_cylinderHeight * 0.5f;
    // Capsule along local Y-axis
    out_p1 = XMVectorSet(0.0f, -halfCylinderHeight, 0.0f, 0.0f);
    out_p2 = XMVectorSet(0.0f, halfCylinderHeight, 0.0f, 0.0f);
}


AABB CapsuleCollider::getWorldAABB(Transform* transform) const
{
    using namespace DirectX;

    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    XMMATRIX worldMatrix = transform->GetWorldMatrix(1);
    XMVECTOR worldP1 = Math::LocalToWorld(worldMatrix, localP1);
    XMVECTOR worldP2 = Math::LocalToWorld(worldMatrix, localP2);

    XMVECTOR minPoint = XMVectorMin(worldP1, worldP2);
    XMVECTOR maxPoint = XMVectorMax(worldP1, worldP2);

    XMVECTOR radiusExtent = XMVectorReplicate(m_radius);

    return AABB(
        XMVectorSubtract(minPoint, radiusExtent),
        XMVectorAdd(maxPoint, radiusExtent)
    );
}


DirectX::XMMATRIX CapsuleCollider::getInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInertiaTensor) {
        float r = m_radius;
        float h = m_cylinderHeight;
        float H = h * 0.5f;

        float cylinderVolume = XM_PI * r * r * h;
        float hemisphereVolume = (2.0f / 3.0f) * XM_PI * r * r * r;
        float totalVolume = cylinderVolume + 2.0f * hemisphereVolume;

        if (totalVolume < 1e-6f) {
            m_inertiaTensor = XMMatrixIdentity();
            m_calculatedInertiaTensor = true;
            return m_inertiaTensor;
        }

        float massPerVolume = mass / totalVolume;
        float massCylinder = massPerVolume * cylinderVolume;
        float massHemisphere = massPerVolume * hemisphereVolume;

        float Iyy_cylinder = 0.5f * massCylinder * r * r;
        float Iyy_hemisphere = (2.0f / 5.0f) * massHemisphere * r * r + massHemisphere * H * H;
        float Iyy = Iyy_cylinder + 2.0f * Iyy_hemisphere;

        float Ixx_cylinder = (1.0f / 12.0f) * massCylinder * (3.0f * r * r + h * h);
        float Ixx_hemisphere = (2.0f / 5.0f) * massHemisphere * r * r + massHemisphere * H * H;
        float Ixx = Ixx_cylinder + 2.0f * Ixx_hemisphere;
        float Izz = Ixx;

        m_inertiaTensor = XMMatrixScaling(Ixx, Iyy, Izz);
        m_calculatedInertiaTensor = true;
    }
    return m_inertiaTensor;
}

DirectX::XMMATRIX CapsuleCollider::getInverseInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInverseInertiaTensor) {
        m_inverseInertiaTensor = getInertiaTensor(mass);
        XMVECTOR determinant;
        m_inverseInertiaTensor = XMMatrixInverse(&determinant, m_inverseInertiaTensor);

        // Handle non-invertible matrix (e.g., zero mass)
        if (XMVectorGetX(XMVectorAbs(determinant)) < 1e-6f) {
            m_inverseInertiaTensor = XMMatrixIdentity();
        }

        m_calculatedInverseInertiaTensor = true;
    }
    return m_inverseInertiaTensor;
}

DirectX::XMVECTOR CapsuleCollider::closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    XMMATRIX worldMatrix = transform->GetWorldMatrix(1);
    XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, worldMatrix);
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorldMatrix);

    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    XMVECTOR closestPointOnSegment = Math::ClosestPointOnLineSegment(localPoint, localP1, localP2);

    XMVECTOR vectorToPoint = XMVectorSubtract(localPoint, closestPointOnSegment);
    XMVECTOR directionToPoint = Math::NormalizeSafe(vectorToPoint);

    XMVECTOR localClosestPoint = XMVectorAdd(closestPointOnSegment, XMVectorScale(directionToPoint, m_radius));

    return Math::LocalToWorld(worldMatrix, localClosestPoint);
}
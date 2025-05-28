#include "BoxCollider.h"
#include <Helpers.h>

BoxCollider::BoxCollider(const DirectX::XMVECTOR& halfSize)
    : m_halfSize(halfSize)
{
}

BoxCollider::~BoxCollider()
{
}

ColliderType BoxCollider::getType() const
{
    return ColliderType::Box;
}

AABB BoxCollider::getWorldAABB(Transform* transform) const
{
    using namespace DirectX;

    auto verts = getWorldVertices(transform);
    if (verts.empty()) {
        XMVECTOR zero = XMVectorZero();
        return AABB(zero, zero);
    }

    XMVECTOR minV = verts[0];
    XMVECTOR maxV = verts[0];

    for (size_t i = 1; i < verts.size(); ++i) {
        minV = XMVectorMin(minV, verts[i]);
        maxV = XMVectorMax(maxV, verts[i]);
    }

    return AABB(minV, maxV);
}

DirectX::XMMATRIX BoxCollider::getInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInertiaTensor) {
        XMFLOAT3 size;
        XMStoreFloat3(&size, XMVectorMultiply(m_halfSize, XMVectorReplicate(2.0f)));

        float Ix = (mass / 12.0f) * (size.y * size.y + size.z * size.z);
        float Iy = (mass / 12.0f) * (size.x * size.x + size.z * size.z);
        float Iz = (mass / 12.0f) * (size.x * size.x + size.y * size.y);

        m_inertiaTensor = XMMatrixScaling(Ix, Iy, Iz);
        m_calculatedInertiaTensor = true;
    }
    return m_inertiaTensor;
}

DirectX::XMMATRIX BoxCollider::getInverseInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInverseInertiaTensor) {
        m_inverseInertiaTensor = getInertiaTensor(mass);
        m_inverseInertiaTensor = XMMatrixInverse(nullptr, m_inverseInertiaTensor);
        m_calculatedInverseInertiaTensor = true;
    }
    return m_inverseInertiaTensor;
}


std::vector<DirectX::XMVECTOR> BoxCollider::getWorldVertices(Transform* tf) const {
    using namespace DirectX;

    std::vector<XMVECTOR> vertices;
    vertices.reserve(8);

    const XMVECTOR half = m_halfSize;
    for (int x = -1; x <= 1; x += 2) {
        for (int y = -1; y <= 1; y += 2) {
            for (int z = -1; z <= 1; z += 2) {
                XMVECTOR localPoint = XMVectorSet(
                    x * XMVectorGetX(half),
                    y * XMVectorGetY(half),
                    z * XMVectorGetZ(half),
                    0.0f
                );
                vertices.push_back(Math::LocalToWorld(tf->GetWorldMatrix(1), localPoint));
            }
        }
    }
    return vertices;
}

std::vector<DirectX::XMVECTOR> BoxCollider::getFaceNormals(Transform* tf) const {
    using namespace DirectX;

    std::vector<XMVECTOR> locals = {
        XMVectorSet(1, 0, 0, 0),
        XMVectorSet(0, 1, 0, 0),
        XMVectorSet(0, 0, 1, 0)
    };

    std::vector<XMVECTOR> worldNormals;
    XMMATRIX rotation = XMMatrixRotationQuaternion(tf->GetRotationQuaternion(1));
    for (const auto& n : locals) {
        worldNormals.push_back(XMVector3TransformNormal(n, rotation));
    }
    return worldNormals;
}

std::vector<DirectX::XMVECTOR> BoxCollider::getEdgeDirections(Transform* tf) const {
    using namespace DirectX;
    XMMATRIX rotation = XMMatrixRotationQuaternion(tf->GetRotationQuaternion(1));
    return {
        XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), rotation),
        XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), rotation),
        XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), rotation)
    };
}

DirectX::XMVECTOR BoxCollider::closestPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    XMVECTOR localPoint = Math::WorldToLocal(tf->GetWorldMatrix(1), point);

    XMVECTOR localMin = -m_halfSize;
    XMVECTOR localMax = m_halfSize;

    XMVECTOR localClosestPoint = localPoint;
    localClosestPoint = XMVectorMax(localClosestPoint, localMin);
    localClosestPoint = XMVectorMin(localClosestPoint, localMax);

    return Math::LocalToWorld(tf->GetWorldMatrix(1), localClosestPoint);
}

bool BoxCollider::containsPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    XMMATRIX invWorld = XMMatrixInverse(nullptr, tf->GetWorldMatrix(1));
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorld);

    return XMVector3InBounds(localPoint, m_halfSize);
}
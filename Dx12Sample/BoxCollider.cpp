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

    // 1) grab all 8 world-space vertices
    auto verts = getWorldVertices(transform);
    if (verts.empty()) {
        // fallback to a degenerate box at the origin
        XMVECTOR zero = XMVectorZero();
        return AABB(zero, zero);
    }

    // 2) init min/max to the first vertex
    XMVECTOR minV = verts[0];
    XMVECTOR maxV = verts[0];

    // 3) sweep the rest
    for (size_t i = 1; i < verts.size(); ++i) {
        minV = XMVectorMin(minV, verts[i]);
        maxV = XMVectorMax(maxV, verts[i]);
    }

    // 4) return the world-space AABB
    return AABB(minV, maxV);
}

DirectX::XMMATRIX BoxCollider::getInertiaTensor(float mass) {
    using namespace DirectX;
    XMFLOAT3 h;  XMStoreFloat3(&h, m_halfSize);  // half-extents
    const float sx = h.x * 2.0f, sy = h.y * 2.0f, sz = h.z * 2.0f;

    const float ix = mass * (sy * sy + sz * sz) * (1.f / 12.f);
    const float iy = mass * (sx * sx + sz * sz) * (1.f / 12.f);
    const float iz = mass * (sx * sx + sy * sy) * (1.f / 12.f);

    // Diagonal – we can build it directly; no XMMatrixScaling needed
    return XMMATRIX(
        ix, 0, 0, 0,
        0, iy, 0, 0,
        0, 0, iz, 0,
        0, 0, 0, 1);
}

DirectX::XMMATRIX BoxCollider::getInverseInertiaTensor(float mass) {
    DirectX::XMMATRIX I = getInertiaTensor(mass);
    I.r[0] = DirectX::XMVectorReciprocal(I.r[0]);
    I.r[1] = DirectX::XMVectorReciprocal(I.r[1]);
    I.r[2] = DirectX::XMVectorReciprocal(I.r[2]);
    return I;
}


std::array<DirectX::XMVECTOR, 8> BoxCollider::getWorldVertices(Transform* tf) const {
    using namespace DirectX;

	std::array<XMVECTOR, 8> v;

    // Sign-bit tricks: build ±halfSize by XORing sign masks
    for (int i = 0; i < 8; ++i)
    {
        const auto local = XMVectorSet(
            (i & 1) ? XMVectorGetX(m_halfSize) : -XMVectorGetX(m_halfSize),
            (i & 2) ? XMVectorGetY(m_halfSize) : -XMVectorGetY(m_halfSize),
            (i & 4) ? XMVectorGetZ(m_halfSize) : -XMVectorGetZ(m_halfSize), 0.f);
        v[i] = XMVector3Transform(local, tf->GetWorldMatrix(1));
    }
    return v;
}

std::array<DirectX::XMVECTOR, 3> BoxCollider::getFaceNormals(Transform* tf) const {
    using namespace DirectX;

    const XMMATRIX rot = XMMatrixRotationQuaternion(tf->GetRotationQuaternion(1));

    return {
        XMVector3TransformNormal(XMVectorSet(1.f, 0.f, 0.f, 0.f), rot),
        XMVector3TransformNormal(XMVectorSet(0.f, 1.f, 0.f, 0.f), rot),
        XMVector3TransformNormal(XMVectorSet(0.f, 0.f, 1.f, 0.f), rot)
    };
}

std::array<DirectX::XMVECTOR, 3> BoxCollider::getEdgeDirections(Transform* tf) const {
    using namespace DirectX;
    const XMMATRIX rot = XMMatrixRotationQuaternion(tf->GetRotationQuaternion(1));

    return {
        XMVector3TransformNormal(XMVectorSet(1.f, 0.f, 0.f, 0.f), rot),
        XMVector3TransformNormal(XMVectorSet(0.f, 1.f, 0.f, 0.f), rot),
        XMVector3TransformNormal(XMVectorSet(0.f, 0.f, 1.f, 0.f), rot)
    };
}

// Closest Point on Box to External Point on the surface
DirectX::XMVECTOR BoxCollider::closestPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    XMVECTOR localPoint = Math::WorldToLocal(tf->GetWorldMatrix(1), point);

    // Define local box extents (min and max)
    XMVECTOR localMin = -m_halfSize;
    XMVECTOR localMax = m_halfSize;

    // Calculate the closest point in local space by clamping each component
    XMVECTOR localClosestPoint = localPoint;
    localClosestPoint = XMVectorMax(localClosestPoint, localMin);
    localClosestPoint = XMVectorMin(localClosestPoint, localMax);

    // Transform back to world space
    return Math::LocalToWorld(tf->GetWorldMatrix(1), localClosestPoint);
}

bool BoxCollider::containsPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    // Transform point to local space
    XMMATRIX invWorld = XMMatrixInverse(nullptr, tf->GetWorldMatrix(1));
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorld);

    // Check if inside box bounds
    return XMVector3InBounds(localPoint, m_halfSize);
}
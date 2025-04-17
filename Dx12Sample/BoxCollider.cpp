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
	DirectX::XMVECTOR center = transform->GetPosition();
	return AABB(
		DirectX::XMVectorSubtract(center, m_halfSize),
		DirectX::XMVectorAdd(center, m_halfSize)
	);
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

DirectX::XMVECTOR BoxCollider::support(Transform* tf, const DirectX::XMVECTOR& direction) const {
    using namespace DirectX;

    // Transform direction to local space
    XMMATRIX invWorld = XMMatrixInverse(nullptr, tf->GetWorldMatrix());
    XMVECTOR localDir = XMVector3TransformNormal(direction, invWorld);

    // Calculate sign manually
    XMVECTOR sign = XMVectorSelect(
        XMVectorReplicate(-1.0f),
        XMVectorReplicate(1.0f),
        XMVectorGreater(localDir, XMVectorZero())
    );

    // Find farthest point in local space
    XMVECTOR localPoint = XMVectorMultiply(m_halfSize, sign);

    // Transform back to world space
    return Math::LocalToWorld(tf, localPoint);
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
                vertices.push_back(Math::LocalToWorld(tf, localPoint));
            }
        }
    }
    return vertices;
}

std::vector<DirectX::XMVECTOR> BoxCollider::getFaceNormals(Transform* tf) const {
    using namespace DirectX;

    // Local space normals (assuming axis-aligned)
    std::vector<XMVECTOR> locals = {
        XMVectorSet(1, 0, 0, 0),
        XMVectorSet(0, 1, 0, 0),
        XMVectorSet(0, 0, 1, 0)
    };

    // Transform to world space
    std::vector<XMVECTOR> worldNormals;
    XMMATRIX rotation = XMMatrixRotationQuaternion(tf->GetRotationQuaternion());
    for (const auto& n : locals) {
        worldNormals.push_back(XMVector3TransformNormal(n, rotation));
    }
    return worldNormals;
}

std::vector<DirectX::XMVECTOR> BoxCollider::getEdgeDirections(Transform* tf) const {
    using namespace DirectX;
    XMMATRIX rotation = XMMatrixRotationQuaternion(tf->GetRotationQuaternion());
    return {
        XMVector3TransformNormal(XMVectorSet(1, 0, 0, 0), rotation),
        XMVector3TransformNormal(XMVectorSet(0, 1, 0, 0), rotation),
        XMVector3TransformNormal(XMVectorSet(0, 0, 1, 0), rotation)
    };
}

// Closest Point on Box to External Point
DirectX::XMVECTOR BoxCollider::closestPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

	XMVECTOR localPoint = Math::WorldToLocal(tf, point);

    // Clamp to box extents
    XMVECTOR clamped = XMVectorClamp(localPoint, -m_halfSize, m_halfSize);

    // Transform back to world space
    return Math::LocalToWorld(tf, clamped);
}

bool BoxCollider::containsPoint(Transform* tf, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    // Transform point to local space
    XMMATRIX invWorld = XMMatrixInverse(nullptr, tf->GetWorldMatrix());
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorld);

    // Check if inside box bounds
    return XMVector3InBounds(localPoint, m_halfSize);
}
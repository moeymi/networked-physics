#pragma once
#include "pch.h"
#include "Collider.h"
class BoxCollider : public Collider
{
private:
	DirectX::XMVECTOR m_halfSize;

public:
	BoxCollider(const DirectX::XMVECTOR& halfSize);
	~BoxCollider();
	ColliderType getType() const override;
	AABB getWorldAABB(Transform* transform) const override;

    DirectX::XMMATRIX getInertiaTensor(float mass) override;
	DirectX::XMMATRIX getInverseInertiaTensor(float mass) override;

    DirectX::XMVECTOR getHalfSize() const { return m_halfSize; }

    // SAT Helpers
    std::array<DirectX::XMVECTOR, 8> getWorldVertices(Transform* transform) const;
    std::array<DirectX::XMVECTOR, 3> getFaceNormals(Transform* transform) const;
    std::array<DirectX::XMVECTOR, 3> getEdgeDirections(Transform* transform) const;

    DirectX::XMVECTOR closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const;
	bool containsPoint(Transform* transform, const DirectX::XMVECTOR& point) const;

};


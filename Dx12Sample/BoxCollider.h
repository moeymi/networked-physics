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
	void setHalfSize(const DirectX::XMVECTOR& halfSize) {
		m_halfSize = halfSize;
	}

    std::vector<DirectX::XMVECTOR> getWorldVertices(Transform* transform) const;
    std::vector<DirectX::XMVECTOR> getFaceNormals(Transform* transform) const;
    std::vector<DirectX::XMVECTOR> getEdgeDirections(Transform* transform) const;

    DirectX::XMVECTOR closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const;
	bool containsPoint(Transform* transform, const DirectX::XMVECTOR& point) const;

};


#pragma once
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

    // GJK/EPA Support
    DirectX::XMVECTOR support(Transform* transform, const DirectX::XMVECTOR& direction) const;

    // SAT Helpers
    std::vector<DirectX::XMVECTOR> getWorldVertices(Transform* transform) const;
    std::vector<DirectX::XMVECTOR> getFaceNormals(Transform* transform) const;
    std::vector<DirectX::XMVECTOR> getEdgeDirections(Transform* transform) const;

    DirectX::XMVECTOR closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const;
	bool containsPoint(Transform* transform, const DirectX::XMVECTOR& point) const;

};


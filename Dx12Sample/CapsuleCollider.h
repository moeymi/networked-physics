#pragma once

#include "pch.h"
#include "Collider.h"

class CapsuleCollider : public Collider
{
private:
	float m_radius;
	float m_cylinderHeight;

public:
	CapsuleCollider(const float& radius, const float& cylinderHeight);
	~CapsuleCollider() = default;

	ColliderType getType() const override;
	AABB getWorldAABB(Transform* transform) const override;

	DirectX::XMMATRIX getInertiaTensor(float mass) override;
	DirectX::XMMATRIX getInverseInertiaTensor(float mass) override;

	float getRadius() const { return m_radius; }
	float getHeight() const { return m_cylinderHeight; }

	void getLocalSegmentEndpoints(DirectX::XMVECTOR& out_p1, DirectX::XMVECTOR& out_p2) const;

	DirectX::XMVECTOR closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const;
};


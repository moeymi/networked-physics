#pragma once

#include "pch.h"
#include "Collider.h"

class CapsuleCollider : public Collider
{
private:
	float m_radius;
	float m_cylinderHeight; // Height of the cylindrical part

public:
	// Constructor takes radius and the height of the cylinder section
	CapsuleCollider(const float& radius, const float& cylinderHeight);
	~CapsuleCollider() = default; // Use default destructor

	ColliderType getType() const override;
	AABB getWorldAABB(Transform* transform) const override;

	DirectX::XMMATRIX getInertiaTensor(float mass) override;
	DirectX::XMMATRIX getInverseInertiaTensor(float mass) override;

	// Getters for dimensions
	float getRadius() const { return m_radius; }
	float getCylinderHeight() const { return m_cylinderHeight; }

	// Helper to get the local-space endpoints of the capsule's central segment
	void getLocalSegmentEndpoints(DirectX::XMVECTOR& out_p1, DirectX::XMVECTOR& out_p2) const;

	// GJK/EPA Support
	DirectX::XMVECTOR support(Transform* transform, const DirectX::XMVECTOR& direction) const;

	// Closest point on capsule surface to a given point
	DirectX::XMVECTOR closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const;

	// Check if a point is inside the capsule
	bool containsPoint(Transform* transform, const DirectX::XMVECTOR& point) const;

};


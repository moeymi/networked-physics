#pragma once
#include "Collider.h"

class SphereCollider : public Collider
{
private:
	float m_radius;

public:
	SphereCollider(float radius);
	~SphereCollider();
	ColliderType getType() const override;
	AABB getWorldAABB(Transform* transform) const override;

	DirectX::XMMATRIX getInertiaTensor(float mass) override;
	DirectX::XMMATRIX getInverseInertiaTensor(float mass) override;

	float getRadius() const;
};


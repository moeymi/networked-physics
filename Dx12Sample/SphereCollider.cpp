#include "SphereCollider.h"

SphereCollider::SphereCollider(float radius)
	: m_radius(radius)
{
}

SphereCollider::~SphereCollider()
{
}

ColliderType SphereCollider::getType() const
{
	return ColliderType::Sphere;
}

AABB SphereCollider::getWorldAABB(Transform* transform) const
{
	DirectX::XMVECTOR center = transform->GetPosition();
	return AABB(
		DirectX::XMVectorSubtract(center, DirectX::XMVectorReplicate(m_radius)), 
		DirectX::XMVectorAdd(center, DirectX::XMVectorReplicate(m_radius))
	);
}

DirectX::XMMATRIX SphereCollider::getInertiaTensor(float mass) {
	using namespace DirectX;
	if (!m_calculatedInertiaTensor) {
		float I = (2.0f / 5.0f) * mass * m_radius * m_radius;
		m_inertiaTensor = XMMatrixScaling(I, I, I);
		m_calculatedInertiaTensor = true;
	}
	return m_inertiaTensor;
}

DirectX::XMMATRIX SphereCollider::getInverseInertiaTensor(float mass) {
	using namespace DirectX;
	if (!m_calculatedInverseInertiaTensor) {
		m_inverseInertiaTensor = getInertiaTensor(mass);
		m_inverseInertiaTensor = XMMatrixInverse(nullptr, m_inverseInertiaTensor);
		m_calculatedInverseInertiaTensor = true;
	}
	return m_inverseInertiaTensor;
}

float SphereCollider::getRadius() const
{
	return m_radius;
}

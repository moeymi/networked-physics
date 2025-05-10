#pragma once

#include "pch.h"
#include <memory>
#include <DirectXMath.h>
#include "Transform.h"

enum class ColliderType : UINT16 
{
	Sphere,
	Box,
	Capsule,
	Count
};


// Specialize std::hash for ColliderType
namespace std {
	template <>
	struct hash<ColliderType> {
		std::size_t operator()(const ColliderType& type) const noexcept {
			return static_cast<std::size_t>(type);
		}
	};
}

class AABB
{
public:
	AABB() = default;
	AABB(const DirectX::XMVECTOR& min, const DirectX::XMVECTOR& max) : m_min(min), m_max(max) {}
	~AABB() = default;
	DirectX::XMVECTOR getMin() const { return m_min; }
	DirectX::XMVECTOR getMax() const { return m_max; }
	DirectX::XMVECTOR getCenter() const { return DirectX::XMVectorScale(DirectX::XMVectorAdd(m_min, m_max), 0.5f); }
	DirectX::XMVECTOR getSize() const { return DirectX::XMVectorSubtract(m_max, m_min); }

private:
	DirectX::XMVECTOR m_min;
	DirectX::XMVECTOR m_max;
};

class Collider
{
public:
	Collider() = default;
	virtual ~Collider() = default;

	virtual ColliderType getType() const = 0;
	virtual AABB getWorldAABB(Transform* transform) const = 0;

	virtual DirectX::XMMATRIX getInertiaTensor(float mass) = 0;
	virtual DirectX::XMMATRIX getInverseInertiaTensor(float mass) = 0;


protected:
	DirectX::XMMATRIX m_inertiaTensor = DirectX::XMMatrixScaling(0, 0, 0);
	DirectX::XMMATRIX m_inverseInertiaTensor = DirectX::XMMatrixScaling(0, 0, 0);

	bool m_calculatedInertiaTensor = false;
	bool m_calculatedInverseInertiaTensor = false;
};


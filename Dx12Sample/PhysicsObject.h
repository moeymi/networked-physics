#pragma once
#include "pch.h"
#include "Mesh.h"
#include "Transform.h"
#include "Collider.h"
#include "Material.h"

enum class MeshType : UINT16 {
	Sphere,
	Box,
	Capsule,
	Plane,
	Count
};

class CommandList;

struct PhysicsMaterial {
	float friction = 0.5f;
	float angularFriction = 0.5f;
	float restitution = 1.0f;
};

struct ContactPoint {
	DirectX::XMVECTOR position;    // World-space contact
	DirectX::XMVECTOR normal;      // From A to B
	float penetration;             // Overlap distance

	float normalMass = 0.0f;
	float tangentMass = 0.0f;
	float angularMass = 0.0f;
	float velocityBias = 0.0f;

	float accumulatedNormalImpulse = 0.0f;
	float accumulatedFrictionImpulse = 0.0f;
	float accumulatedAngularFrictionImpulse = 0.0f;

	DirectX::XMVECTOR tangent = DirectX::XMVectorZero();
};

class PhysicsObject;
struct CollisionManifold {
	PhysicsObject* objectA = nullptr;
	PhysicsObject* objectB = nullptr;
	std::vector<ContactPoint> contacts;
};

class PhysicsObject final {
public:
	enum class MotionIntegrationType {
		SemiImplicitEuler,
		Euler,
		RK4,
		Verlet
	};
	PhysicsObject(const UINT& id, const MeshType& meshType, std::shared_ptr<Mesh> mesh, std::shared_ptr<Texture> texture);
	~PhysicsObject() = default;

	void onLoad();
	void onUnload();
	void onUpdate(float deltaTime);
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix);
	void onCollision(CollisionManifold collisionManifold);

	void updateInertiaTensor();

	void setOwnerId(const uint32_t& ownerId);
	void setIntegrationType(const MotionIntegrationType& integrationType);

	void applyConstantForce(const DirectX::XMVECTOR& force);
	void applyImpulse(const DirectX::XMVECTOR& impulse);
	void applyImpulseAtPosition(const DirectX::XMVECTOR& impulse, const DirectX::XMVECTOR& contactPoint);
	void applyAngularImpulse(const DirectX::XMVECTOR& impulse);
	void resetConstantForces();

	void setVelocity(const DirectX::XMVECTOR& velocity, const USHORT& bufferIndex);
	void setAngularVelocity(const DirectX::XMVECTOR& angularVelocity, const USHORT& bufferIndex);

	void setCollider(std::shared_ptr<Collider> collider);
	void setMass(const float& mass);
	void setStatic(const bool& isStatic);
	void setPhysicsMaterial(const PhysicsMaterial& material);
	void setColor(const DirectX::XMFLOAT4& color);

	bool isStatic() const;
	float getMass() const;
	UINT getId() const;
	uint32_t getOwnerId() const;
	MeshType getMeshType() const;
	DirectX::XMVECTOR getCenterOfMass() const;
	DirectX::XMVECTOR getVelocity(const USHORT& bufferIndex) const;
	DirectX::XMVECTOR getAngularVelocity(const USHORT& bufferIndex) const;
	DirectX::XMMATRIX getInverseWorldInertiaTensor(const USHORT& bufferIndex) const;

	Material getMaterial() const;
	PhysicsMaterial getPhysicsMaterial() const;
	Collider* getCollider() const;
	Transform& getTransform();

	void swapStates();

private:
	UINT m_id = 0u;
	uint32_t m_ownerId;
	Transform m_transform;
	MeshType m_meshType;
	std::shared_ptr<Mesh> m_mesh;
	std::shared_ptr<Texture> m_texture;
	std::shared_ptr<Collider> m_collider;
	Material m_material = Material::White;

	struct State {
		DirectX::XMVECTOR m_velocity =			{ 0, 0, 0, 0 };
		DirectX::XMVECTOR m_angularVelocity =	{ 0, 0, 0, 0 };
	} m_states[2];

	float m_mass = 1.0f;
	PhysicsMaterial m_physicsMaterial;

	DirectX::XMVECTOR m_centerOfMass = { 0.0f, 0.0f, 0.0f, 1.0f };
	DirectX::XMVECTOR m_constantForces = { 0.0f, 0.0f, 0.0f, 0.0f };
	DirectX::XMMATRIX m_inverseWorldInertiaTensor = DirectX::XMMatrixScaling(0.0f, 0.0f, 0.0f);

	void (PhysicsObject::* m_integrateMotion)(const float& deltaTime);
	MotionIntegrationType m_integrationType = MotionIntegrationType::SemiImplicitEuler;

	DirectX::XMVECTOR calculateForces(const DirectX::XMVECTOR& position, const DirectX::XMVECTOR& velocity);

	void integrateMotion(const float& deltaTime);
	void integrateAngularMotion(const float& deltaTime);

	void integrateEuler(const float& deltaTime);
	void integrateSemiImplicitEuler(const float& deltaTime);
	void integrateRK4(const float& deltaTime);
	void integrateVerlet(const float& deltaTime);
};
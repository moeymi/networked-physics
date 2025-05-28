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
	float restitution = 1.0f;
};

struct ContactPoint {
	DirectX::XMVECTOR position;
	DirectX::XMVECTOR normal;
	float penetration;

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
	PhysicsObject(const MeshType& meshType, Mesh* mesh, Texture* texture);
	~PhysicsObject();

	void setUserData(void* userData);
	void* getUserData() const;

	void onLoad();
	void onUnload();
	void onUpdate(float deltaTime);
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix);
	void onCollision(CollisionManifold collisionManifold);

	void updateInertiaTensor();

	void setIntegrationType(const MotionIntegrationType& integrationType);

	void applyConstantForce(const DirectX::XMVECTOR& force);
	void applyImpulse(const DirectX::XMVECTOR& impulse);
	void applyImpulseAtPosition(const DirectX::XMVECTOR& impulse, const DirectX::XMVECTOR& contactPoint);
	void applyAngularImpulse(const DirectX::XMVECTOR& impulse);
	void resetConstantForces();

	void setVelocity(const DirectX::XMVECTOR& velocity, const USHORT& bufferIndex);
	void setVelocity(const DirectX::XMFLOAT3& velocity, const USHORT& bufferIndex);
	void setAngularVelocity(const DirectX::XMVECTOR& angularVelocity, const USHORT& bufferIndex);
	void setAngularVelocity(const DirectX::XMFLOAT3& angularVelocity, const USHORT& bufferIndex);

	void setCollider(std::shared_ptr<Collider> collider);
	void setMass(const float& mass);
	void setStatic(const bool& isStatic);
	void setPhysicsMaterial(const PhysicsMaterial& material);
	void setColor(const DirectX::XMFLOAT4& color);

	int getIsland() const;
	void setIsland(const int& island) const;

	bool isStatic() const;
	float getMass() const;
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
	Transform m_transform;
	MeshType m_meshType;
	Mesh* m_mesh;
	Texture* m_texture;
	std::shared_ptr<Collider> m_collider;
	Material m_material = Material::White;

	void* m_userData = nullptr;

	struct State {
		DirectX::XMVECTOR m_velocity =			{ 0, 0, 0, 0 };
		DirectX::XMVECTOR m_angularVelocity =	{ 0, 0, 0, 0 };
	} m_states[2];

	std::unordered_map<uint16_t, std::function<void()>> m_onDirtifyCallbacks;

	float m_mass = 1.0f;
	PhysicsMaterial m_physicsMaterial;
	mutable int m_island = -1;

	DirectX::XMVECTOR m_centerOfMass = { 0.0f, 0.0f, 0.0f, 1.0f };
	DirectX::XMVECTOR m_constantForces = { 0.0f, 0.0f, 0.0f, 0.0f };
	DirectX::XMMATRIX m_inverseWorldInertiaTensor = DirectX::XMMatrixScaling(0.0f, 0.0f, 0.0f);

	void (PhysicsObject::* m_integrateMotion)(const float& deltaTime);
	MotionIntegrationType m_integrationType = MotionIntegrationType::SemiImplicitEuler;

	DirectX::XMVECTOR calculateForces(const DirectX::XMVECTOR& position, const DirectX::XMVECTOR& velocity);
	void integrateMotion(const float& deltaTime);
	void integrateAngularMotion(const float& deltaTime);

	void adjustBrightness();

	void integrateEuler(const float& deltaTime);
	void integrateSemiImplicitEuler(const float& deltaTime);
	void integrateRK4(const float& deltaTime);
	void integrateVerlet(const float& deltaTime);
};
#pragma once
#include "Mesh.h"
#include "Transform.h"
#include "Collider.h"

class CommandList;

struct PhysicsMaterial {
	float friction = 0.0f;
	float restitution = 1.0f;
};

struct ContactPoint {
	DirectX::XMVECTOR position;    // World-space contact
	DirectX::XMVECTOR normal;      // From A to B
	float penetration;             // Overlap distance

	float accumulatedNormalImpulse = 0.0f;
	float accumulatedFrictionImpulse = 0.0f;
	float normalMass = 0.0f;
	float tangentMass = 0.0f;
	float bias = 0.0f;
};

class PhysicsObject;
struct CollisionManifold {
	PhysicsObject* objectA;
	PhysicsObject* objectB;
	std::vector<ContactPoint> contacts;
};

enum ForceType {
	Impulse,
	Constant,
};

class PhysicsObject final
{
public:
	enum class MotionIntegrationType {
		SemiImplicitEuler,
		Euler,
		RK4,
		Verlet
	};
	PhysicsObject(std::shared_ptr<Mesh> mesh, std::shared_ptr<Texture> texture);
	~PhysicsObject() = default;

	void onLoad(CommandList& commandList);
	void onUnload(CommandList& commandList);
	void onUpdate(float deltaTime);
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix);
	void onCollision(CollisionManifold collisionManifold);

	void updateInertiaTensor();

	void setIntegrationType(const MotionIntegrationType& integrationType);

	void applyConstantForce(const DirectX::XMVECTOR& force);
	void applyImpulse(const DirectX::XMVECTOR& impulse);
	void applyImpulseAtPosition(const DirectX::XMVECTOR& impulse, const DirectX::XMVECTOR& contactPoint);
	void resetConstantForces();

	void setFutureVelocity(const DirectX::XMVECTOR& velocity);
	void setFutureAngularVelocity(const DirectX::XMVECTOR& angularVelocity);

	void setCurrentVelocity(const DirectX::XMVECTOR& velocity);
	void setCurrentAngularVelocity(const DirectX::XMVECTOR& angularVelocity);

	void setCollider(std::shared_ptr<Collider> collider);
	void setMass(const float& mass);
	void setStatic(const bool& isStatic);
	void setMaterial(const PhysicsMaterial& material);

	bool isStatic() const;
	float getMass() const;
	DirectX::XMVECTOR getCenterOfMass() const;
	DirectX::XMVECTOR getVelocity() const;
	DirectX::XMVECTOR getAngularVelocity() const;
	DirectX::XMMATRIX getInverseWorldInertiaTensor() const;

	PhysicsMaterial getMaterial() const;
	Collider* getCollider() const;
	Transform& getTransform();

	void swapStates();


private:
	Transform m_transform;
	std::shared_ptr<Mesh> m_mesh;
	std::shared_ptr<Texture> m_texture;
	std::shared_ptr<Collider> m_collider;

	struct State {
		DirectX::XMVECTOR m_velocity =			{ 0, 0, 0, 0 };
		DirectX::XMVECTOR m_angularVelocity =	{ 0, 0, 0, 0 };
	};

	State m_currentState;
	State m_futureState;

	// Physics properties
	bool m_isStatic = false;
	float m_mass = 1.0f;
	PhysicsMaterial m_material;

	DirectX::XMVECTOR m_centerOfMass = { 0.0f, 0.0f, 0.0f, 1.0f };

	// Forces and motion integration
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
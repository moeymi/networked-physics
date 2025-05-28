#include "PhysicsObject.h"

#include <iostream>
#include <random>
#include <algorithm>

#include "CommandList.h"
#include "Material.h"
#include "PhysicsSimulation.h"

PhysicsObject::PhysicsObject(const MeshType& meshType, Mesh* mesh, Texture* texture) :
    m_mesh(mesh),
	m_meshType(meshType),
    m_texture(texture),
	m_collider(nullptr),
	m_mass(1.0f),
	m_material(Material::White),
	m_physicsMaterial(),
    m_constantForces({ 0.0f, 0.0f, 0.0f, 0.0f }),
	m_integrateMotion(&PhysicsObject::integrateSemiImplicitEuler)
{
};

PhysicsObject::~PhysicsObject()
{
}

void PhysicsObject::setUserData(void* userData) { m_userData = userData; }
void* PhysicsObject::getUserData() const { return m_userData; }

void PhysicsObject::onLoad()
{
}

void PhysicsObject::onUnload()
{
}

void PhysicsObject::onUpdate(float deltaTime)
{
    if (m_transform.IsStatic()) return;

    updateInertiaTensor();

    integrateMotion(deltaTime);

	integrateAngularMotion(deltaTime);
}

void PhysicsObject::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
{
    const auto worldMatrix = m_transform.GetWorldMatrix(0);

    Mat matrices;
    PhysicsSimulation::ComputeMatrices(worldMatrix, viewMatrix, viewProjectionMatrix, matrices);

    commandList.SetGraphicsDynamicConstantBuffer(RootParameters::MatricesCB, matrices);
    commandList.SetGraphicsDynamicConstantBuffer(RootParameters::MaterialCB, m_material);
    commandList.SetShaderResourceView(RootParameters::Textures, 0, *m_texture, D3D12_RESOURCE_STATE_PIXEL_SHADER_RESOURCE);

	m_mesh->Draw(commandList);
}

void PhysicsObject::onCollision(CollisionManifold collisionManfold)
{
	std::cout << "Collision detected!" << std::endl;
}

void PhysicsObject::setMass(const float& mass)
{
	m_mass = max(mass, 0.00001f);
}

void PhysicsObject::setStatic(const bool& isStatic)
{
	m_transform.SetStatic(isStatic);
}

void PhysicsObject::setPhysicsMaterial(const PhysicsMaterial& material) {
	m_physicsMaterial = material;
    adjustBrightness();
}

void PhysicsObject::setColor(const DirectX::XMFLOAT4& color) {
    m_material.Ambient = { color.x * 0.1f, color.y * 0.1f, color.z * 0.1f, 1 };
	m_material.Diffuse = color;
	adjustBrightness();
}

void PhysicsObject::setIsland(const int& i) const
{
	m_island = i;
}

int PhysicsObject::getIsland() const
{
	return m_island;
}

void PhysicsObject::adjustBrightness() {
    using namespace DirectX;

    auto clamp = [](float v, float mn, float mx) {
        return max(mn, min(mx, v));
        };

    float friction = clamp(m_physicsMaterial.friction, 0.0f, 1.0f);
    float restitution = clamp(m_physicsMaterial.restitution, 0.0f, 1.0f);

    float brightness = 1.0f - friction;
    float whiteness = 1.0f - friction;

    XMFLOAT3 baseColor = { m_material.Diffuse.x, m_material.Diffuse.y, m_material.Diffuse.z };

    XMFLOAT4 newDiffuse = {
        clamp(baseColor.x * brightness + whiteness * (1.0f - brightness), 0.0f, 1.0f),
        clamp(baseColor.y * brightness + whiteness * (1.0f - brightness), 0.0f, 1.0f),
        clamp(baseColor.z * brightness + whiteness * (1.0f - brightness), 0.0f, 1.0f),
        1.0f
    };
    m_material.Diffuse = newDiffuse;

    float ambientScale = 0.3f;
    m_material.Ambient = XMFLOAT4(
        m_material.Diffuse.x * ambientScale,
        m_material.Diffuse.y * ambientScale,
        m_material.Diffuse.z * ambientScale,
        1.0f
    );

    float specularStrength = 0.2f + restitution * 0.8f; // 0.2 to 1.0
    m_material.Specular = XMFLOAT4(
        specularStrength,
        specularStrength,
        specularStrength,
        1.0f
    );

    m_material.SpecularPower = 16.0f + restitution * 112.0f; // 16 to 128
}

void PhysicsObject::setCollider(std::shared_ptr<Collider> collider)
{
	m_collider = collider;
}

void PhysicsObject::swapStates()
{
	m_states[0] = m_states[1];
	m_transform.swapStates();
}

Transform& PhysicsObject::getTransform()
{
	return m_transform;
}

Collider* PhysicsObject::getCollider() const
{
	return m_collider.get();
}

bool PhysicsObject::isStatic() const
{
	return m_transform.IsStatic();
}

MeshType PhysicsObject::getMeshType() const
{
	return m_meshType;
}

float PhysicsObject::getMass() const
{
    return m_mass;
}

DirectX::XMVECTOR PhysicsObject::getVelocity(const USHORT& bufferIndex) const
{
	return m_states[bufferIndex].m_velocity;
}

DirectX::XMVECTOR PhysicsObject::getAngularVelocity(const USHORT& bufferIndex) const
{
	return m_states[bufferIndex].m_angularVelocity;
}

DirectX::XMVECTOR PhysicsObject::getCenterOfMass() const
{
	return m_centerOfMass;
}

Material PhysicsObject::getMaterial() const
{
	return m_material;
}

PhysicsMaterial PhysicsObject::getPhysicsMaterial() const
{
	return m_physicsMaterial;
}

DirectX::XMMATRIX PhysicsObject::getInverseWorldInertiaTensor(const USHORT& bufferIndex) const
{
	return m_inverseWorldInertiaTensor;
}

void PhysicsObject::updateInertiaTensor()
{
	using namespace DirectX;
	if (m_collider)
	{
        XMMATRIX rotation = XMMatrixRotationQuaternion(m_transform.GetRotationQuaternion(0));

        m_inverseWorldInertiaTensor = XMMatrixMultiply(
            XMMatrixMultiply(rotation, m_collider->getInverseInertiaTensor(m_mass)),
            XMMatrixTranspose(rotation)
        );
	}
}

void PhysicsObject::setVelocity(const DirectX::XMVECTOR& velocity, const USHORT& bufferIndex)
{
    m_states[bufferIndex].m_velocity = velocity;
}

void PhysicsObject::setVelocity(const DirectX::XMFLOAT3& velocity, const USHORT& bufferIndex)
{
	m_states[bufferIndex].m_velocity = DirectX::XMLoadFloat3(&velocity);
}

void PhysicsObject::setAngularVelocity(const DirectX::XMVECTOR& angularVelocity, const USHORT& bufferIndex)
{
    m_states[bufferIndex].m_angularVelocity = angularVelocity;
}

void PhysicsObject::setAngularVelocity(const DirectX::XMFLOAT3& angularVelocity, const USHORT& bufferIndex)
{
	m_states[bufferIndex].m_angularVelocity = DirectX::XMLoadFloat3(&angularVelocity);
}

void PhysicsObject::setIntegrationType(const MotionIntegrationType& integrationType)
{
	switch (integrationType)
	{
	case MotionIntegrationType::SemiImplicitEuler:
		m_integrateMotion = &PhysicsObject::integrateSemiImplicitEuler;
		break;
	case MotionIntegrationType::Euler:
		m_integrateMotion = &PhysicsObject::integrateEuler;
		break;
	case MotionIntegrationType::RK4:
		m_integrateMotion = &PhysicsObject::integrateRK4;
		break;
	case MotionIntegrationType::Verlet:
		m_integrateMotion = &PhysicsObject::integrateVerlet;
		break;
	default:
		std::cerr << "Invalid integration type!" << std::endl;
	}
}

void PhysicsObject::applyConstantForce(const DirectX::XMVECTOR& force)
{
	m_constantForces = DirectX::XMVectorAdd(m_constantForces, force);
}

void PhysicsObject::resetConstantForces()
{
	m_constantForces = { 0.0f, 0.0f, 0.0f, 0.0f };
}

DirectX::XMVECTOR PhysicsObject::calculateForces(const DirectX::XMVECTOR& position, const DirectX::XMVECTOR& velocity)
{
    DirectX::XMVECTOR force = DirectX::XMVectorZero();

    force = DirectX::XMVectorAdd(force, m_constantForces);

    return force;
}

void PhysicsObject::applyImpulse(const DirectX::XMVECTOR& impulse)
{
	if (m_transform.IsStatic()) return;
	m_states[1].m_velocity = DirectX::XMVectorAdd(m_states[1].m_velocity, DirectX::XMVectorScale(impulse, 1.0f / m_mass));
}

void PhysicsObject::applyImpulseAtPosition(const DirectX::XMVECTOR& impulse, const DirectX::XMVECTOR& contactPoint)
{
    using namespace DirectX;

    if (m_transform.IsStatic()) return;

    XMVECTOR worldCOM = XMVector3Transform(m_centerOfMass,
        m_transform.GetWorldMatrix(0));

    XMVECTOR r = XMVectorSubtract(contactPoint, worldCOM);

    XMVECTOR torque = XMVector3Cross(r, impulse);

    applyImpulse(impulse);

    XMVECTOR angularImpulse = XMVector3Transform(torque, m_inverseWorldInertiaTensor);
    m_states[1].m_angularVelocity = XMVectorAdd(m_states[1].m_angularVelocity, angularImpulse);
}

void PhysicsObject::applyAngularImpulse(const DirectX::XMVECTOR& impulse)
{
	if (m_transform.IsStatic()) return;
	m_states[1].m_angularVelocity = DirectX::XMVectorAdd(m_states[1].m_angularVelocity, DirectX::XMVector3Transform(impulse, m_inverseWorldInertiaTensor));
}

void PhysicsObject::integrateMotion(const float& deltaTime)
{
	(this->*m_integrateMotion)(deltaTime);
}

void PhysicsObject::integrateAngularMotion(const float& deltaTime)
{
    using namespace DirectX;

    if (m_transform.IsStatic()) return;

    XMVECTOR angVelQuat = XMVectorSet(
        XMVectorGetX(m_states[0].m_angularVelocity),
        XMVectorGetY(m_states[0].m_angularVelocity),
        XMVectorGetZ(m_states[0].m_angularVelocity),
        0.0f
    );

    XMVECTOR orientation = m_transform.GetRotationQuaternion(0);
    XMVECTOR qDot = XMQuaternionMultiply(orientation, angVelQuat);
    qDot = XMVectorScale(qDot, 0.5f * deltaTime);

    m_transform.SetRotationQuaternion(XMQuaternionNormalize(XMVectorAdd(orientation, qDot)), 1);
}

void PhysicsObject::integrateEuler(const float& deltaTime) {
    DirectX::XMVECTOR currentPosition = m_transform.GetPosition(0);
    currentPosition = DirectX::XMVectorAdd(currentPosition, DirectX::XMVectorScale(m_states[0].m_velocity, deltaTime));
    m_transform.SetPosition(currentPosition, 1);

	auto force = calculateForces(m_transform.GetPosition(0), m_states[0].m_velocity);
	auto acceleration = DirectX::XMVectorScale(force, 1.0f / m_mass);

    m_states[1].m_velocity = DirectX::XMVectorAdd(m_states[1].m_velocity, DirectX::XMVectorScale(acceleration, deltaTime));
}

void PhysicsObject::integrateSemiImplicitEuler(const float& deltaTime) {
	auto force = calculateForces(m_transform.GetPosition(0), m_states[0].m_velocity);
	auto acceleration = DirectX::XMVectorScale(force, 1.0f / m_mass);

    m_states[1].m_velocity = DirectX::XMVectorAdd(m_states[1].m_velocity, DirectX::XMVectorScale(acceleration, deltaTime));

    DirectX::XMVECTOR currentPosition = m_transform.GetPosition(0);
    currentPosition = DirectX::XMVectorAdd(currentPosition, DirectX::XMVectorScale(m_states[1].m_velocity, deltaTime));

    m_transform.SetPosition(currentPosition, 1);
}

void PhysicsObject::integrateRK4(const float& deltaTime) {
    DirectX::XMVECTOR currentPosition = m_transform.GetPosition(0);
    DirectX::XMVECTOR currentVelocity = m_states[0].m_velocity;

    auto ComputeAcceleration = [this](const DirectX::XMVECTOR& pos, const DirectX::XMVECTOR& vel) {
        DirectX::XMVECTOR force = calculateForces(pos, vel);
        return DirectX::XMVectorScale(force, 1.0f / m_mass);
    };

    DirectX::XMVECTOR k1Vel = ComputeAcceleration(currentPosition, currentVelocity);
    DirectX::XMVECTOR k1Pos = currentVelocity;

    DirectX::XMVECTOR pos2 = DirectX::XMVectorAdd(currentPosition, DirectX::XMVectorScale(k1Pos, deltaTime * 0.5f));
    DirectX::XMVECTOR vel2 = DirectX::XMVectorAdd(currentVelocity, DirectX::XMVectorScale(k1Vel, deltaTime * 0.5f));
    DirectX::XMVECTOR k2Vel = ComputeAcceleration(pos2, vel2);
    DirectX::XMVECTOR k2Pos = vel2;

    DirectX::XMVECTOR pos3 = DirectX::XMVectorAdd(currentPosition, DirectX::XMVectorScale(k2Pos, deltaTime * 0.5f));
    DirectX::XMVECTOR vel3 = DirectX::XMVectorAdd(currentVelocity, DirectX::XMVectorScale(k2Vel, deltaTime * 0.5f));
    DirectX::XMVECTOR k3Vel = ComputeAcceleration(pos3, vel3);
    DirectX::XMVECTOR k3Pos = vel3;

    DirectX::XMVECTOR pos4 = DirectX::XMVectorAdd(currentPosition, DirectX::XMVectorScale(k3Pos, deltaTime));
    DirectX::XMVECTOR vel4 = DirectX::XMVectorAdd(currentVelocity, DirectX::XMVectorScale(k3Vel, deltaTime));
    DirectX::XMVECTOR k4Vel = ComputeAcceleration(pos4, vel4);
    DirectX::XMVECTOR k4Pos = vel4;

    DirectX::XMVECTOR velocityIncrement = DirectX::XMVectorScale(
        DirectX::XMVectorAdd(
            DirectX::XMVectorAdd(k1Vel, DirectX::XMVectorScale(DirectX::XMVectorAdd(k2Vel, k3Vel), 2.0f)),
            k4Vel
        ),
        deltaTime / 6.0f
    );

    DirectX::XMVECTOR positionIncrement = DirectX::XMVectorScale(
        DirectX::XMVectorAdd(
            DirectX::XMVectorAdd(k1Pos, DirectX::XMVectorScale(DirectX::XMVectorAdd(k2Pos, k3Pos), 2.0f)),
            k4Pos
        ),
        deltaTime / 6.0f
    );

    m_states[1].m_velocity = DirectX::XMVectorAdd(currentVelocity, velocityIncrement);
    m_transform.SetPosition(DirectX::XMVectorAdd(currentPosition, positionIncrement), 1);
}


void PhysicsObject::integrateVerlet(const float& deltaTime) {
    DirectX::XMVECTOR currentPosition = m_transform.GetPosition(0);
    DirectX::XMVECTOR currentVelocity = m_states[0].m_velocity;

    DirectX::XMVECTOR currentForce = calculateForces(currentPosition, currentVelocity);
    DirectX::XMVECTOR currentAcceleration = DirectX::XMVectorScale(currentForce, 1.0f / m_mass);

    DirectX::XMVECTOR newPosition = DirectX::XMVectorAdd(
        currentPosition,
        DirectX::XMVectorAdd(
            DirectX::XMVectorScale(currentVelocity, deltaTime),
            DirectX::XMVectorScale(currentAcceleration, 0.5f * deltaTime * deltaTime)
        )
    );

    DirectX::XMVECTOR newForce = calculateForces(newPosition, currentVelocity);
    DirectX::XMVECTOR newAcceleration = DirectX::XMVectorScale(newForce, 1.0f / m_mass);

    DirectX::XMVECTOR averageAcceleration = DirectX::XMVectorScale(
        DirectX::XMVectorAdd(currentAcceleration, newAcceleration),
        0.5f
    );
    DirectX::XMVECTOR newVelocity = DirectX::XMVectorAdd(
        currentVelocity,
        DirectX::XMVectorScale(averageAcceleration, deltaTime)
    );

    m_transform.SetPosition(newPosition, 1);
    m_states[1].m_velocity = newVelocity;
}

#include "BallToBallScenario.h"

#include "SphereCollider.h"


void BallToBallScenario::onLoad(CommandList& commandList)
{
	m_SphereMesh = Mesh::CreateSphere(commandList, 1.0f, 16, false);
	m_customTexture = std::make_shared<Texture>();
	m_defaultTexture = std::make_shared<Texture>();

	commandList.LoadTextureFromFile(*m_customTexture, L"Assets/Textures/earth.dds");
	commandList.LoadTextureFromFile(*m_defaultTexture, L"Assets/Textures/DefaultWhite.bmp");

	PhysicsMaterial material = {
		0.2f, // static friction
		1.0f,
	};

	auto particle1 = std::make_shared<PhysicsObject>(m_SphereMesh, m_customTexture);
	auto sphereCollider = std::make_shared<SphereCollider>(.5f);
	particle1->setCollider(sphereCollider);
	particle1->setMaterial(material);
	particle1->onLoad(commandList);
	particle1->getTransform().SetPosition({ 0, 0, 2, 1 }, 0, true);
	particle1->setVelocity({ 3, 0, 0, 0 }, 0);
	particle1->setVelocity({ 3, 0, 0, 0 }, 1);
	particle1->setAngularVelocity({ 0, 0, 5, 0 }, 0);
	particle1->setAngularVelocity({ 0, 0, 5, 0 }, 1);

	auto particle2 = std::make_shared<PhysicsObject>(m_SphereMesh, m_customTexture);
	particle2->setCollider(sphereCollider);
	particle2->setMaterial(material);
	particle2->onLoad(commandList);
	particle2->getTransform().SetPosition({ 3, 0, 2, 1 }, 0, true);

	m_physicsObjects.push_back(particle1);
	m_physicsEngine.addBody(particle1);
	m_physicsObjects.push_back(particle2);
	m_physicsEngine.addBody(particle2);
}

void BallToBallScenario::onUpdate(const float& dt)
{
	// Update physics engine
	m_physicsEngine.onUpdate(dt);
}

void BallToBallScenario::onUnload(CommandList& commandList)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onUnload(commandList);
	}
	m_physicsObjects.clear();
}

void BallToBallScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}
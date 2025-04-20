#include "BallToWallScenario.h"

#include "SphereCollider.h"


void BallToWallScenario::onLoad(CommandList& commandList)
{
	m_SphereMesh = Mesh::CreateSphere(commandList, 1.0f, 16, false);
	m_PlaneMesh = Mesh::CreatePlane(commandList);
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
	//particle1->setVelocity({ 3, 0, 0, 0 }, 1);
	particle1->setAngularVelocity({ 0, 0, 1, 0 }, 1);
	//particle1->applyConstantForce({ 0.0f, -9.81f * particle1->getMass(), 0.0f, 0.0f });

	auto wall = std::make_shared<PhysicsObject>(m_PlaneMesh, m_defaultTexture);
	auto planeCollider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(.5f, 0.01f, .5f, 0.0f));
	wall->setCollider(planeCollider);
	wall->setMaterial(material);
	wall->onLoad(commandList);
	wall->getTransform().SetPosition({ 0.0f, -10.0f, 0.0f, 1 }, 0, true);
	wall->getTransform().SetScale({ 20.0f, 1.0f, 20.0f, 1 }, 0, true);
	wall->getTransform().SetRotationEulerAngles({ 0.0f, 0.0f, 0.0f }, 0, true);
	wall->setStatic(true);

	m_physicsObjects.push_back(particle1);
	m_physicsEngine.addBody(particle1);
	m_physicsObjects.push_back(wall);
	m_physicsEngine.addBody(wall);
}

void BallToWallScenario::onUpdate(const float& dt)
{
	// Update physics engine
	m_physicsEngine.onUpdate(dt);
}

void BallToWallScenario::onUnload(CommandList& commandList)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onUnload(commandList);
	}
	m_physicsObjects.clear();
}

void BallToWallScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}
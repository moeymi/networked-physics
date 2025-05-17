#include "BallToCapsuleScenario.h"

#include "SphereCollider.h"
#include "CapsuleCollider.h"

void BallToCapsuleScenario::onLoad(CommandList& commandList)
{
	m_SphereMesh = Mesh::CreateSphere(commandList, 1.0f, 16, false);
	m_CapsuleMesh = Mesh::CreateCapsule(commandList, 1.0f, 2.0f, 16, false);
	m_customTexture = std::make_shared<Texture>();
	m_defaultTexture = std::make_shared<Texture>();

	commandList.LoadTextureFromFile(*m_customTexture, L"Assets/Textures/earth.dds");
	commandList.LoadTextureFromFile(*m_defaultTexture, L"Assets/Textures/DefaultWhite.bmp");

	PhysicsMaterial material = {
		0.2f, // static friction
		.01f,
	};

	auto particle = std::make_shared<PhysicsObject>(static_cast<UINT>(4000), MeshType::Sphere, m_SphereMesh, m_customTexture);
	auto sphereCollider = std::make_shared<SphereCollider>(.5f);
	particle->setCollider(sphereCollider);
	particle->setPhysicsMaterial(material);
	particle->onLoad();
	particle->getTransform().SetPosition({ -1.5, 0.98, 1, 1 }, 0, true);
	particle->setVelocity({ 2, 0, 0, 0 }, 1);
	//particle->setAngularVelocity({ 0, 0, 5, 0 }, 1);

	auto capsule = std::make_shared<PhysicsObject>(static_cast<UINT>(4000), MeshType::Capsule, m_CapsuleMesh, m_customTexture);
	auto capsuleCollider = std::make_shared<CapsuleCollider>(0.5f, 1.0f);
	capsule->setCollider(capsuleCollider);
	capsule->setPhysicsMaterial(material);
	capsule->onLoad();
	capsule->getTransform().SetPosition({ 1.5, -0.5, 1, 1 }, 0, true);

	m_physicsObjects.push_back(particle);
	m_physicsObjects.push_back(capsule);
}

void BallToCapsuleScenario::onUnload(CommandList& commandList)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onUnload();
	}
	m_physicsObjects.clear();
}

void BallToCapsuleScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}

void BallToCapsuleScenario::drawImGui() {

}
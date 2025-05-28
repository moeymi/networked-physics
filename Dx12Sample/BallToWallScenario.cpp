#include "BallToWallScenario.h"

#include "BoxCollider.h"
#include "SphereCollider.h"


void BallToWallScenario::onLoad(CommandList& commandList)
{

	PhysicsMaterial material = {
		0.2f, // friction
		1.0f,
	};

	auto particle1 = std::make_shared<PhysicsObject>( MeshType::Sphere, GlobalData::g_sphereMesh.get(), GlobalData::g_customTexture.get());
	auto sphereCollider = std::make_shared<SphereCollider>(.5f);
	particle1->setCollider(sphereCollider);
	particle1->setPhysicsMaterial(material);
	particle1->onLoad();
	particle1->getTransform().SetPosition({ 0, 0, 2, 1 }, 0, true);
	particle1->setAngularVelocity({ 0, 0, 1, 0 }, 1);

	auto wall = std::make_shared<PhysicsObject>(MeshType::Plane, GlobalData::g_planeMesh.get(), GlobalData::g_defaultTexture.get());
	auto planeCollider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(.5f, 0.01f, .5f, 0.0f));
	wall->setCollider(planeCollider);
	wall->setPhysicsMaterial(material);
	wall->onLoad();
	wall->getTransform().SetPosition({ 0.0f, -10.0f, 0.0f, 1 }, 0, true);
	wall->getTransform().SetScale({ 20.0f, 1.0f, 20.0f, 1 }, 0, true);
	wall->getTransform().SetRotationEulerAngles({ 0.0f, 0.0f, 0.0f }, 0, true);
	wall->setStatic(true);

	m_physicsObjects.push_back(particle1);
	m_physicsObjects.push_back(wall);
}

void BallToWallScenario::onUnload(CommandList& commandList)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onUnload();
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

void BallToWallScenario::drawImGui() {

}
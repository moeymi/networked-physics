#include "RoomScenario.h"

#include "BoxCollider.h"
#include "SphereCollider.h"
#include "GlobalData.h"

#include <algorithm>
#include <random>
#include "CollisionSystem.h"

void RoomScenario::onLoad(CommandList& commandList)
{
	std::normal_distribution<float> d{ 0.0, 1.0 };

	// Create 6 planes to make a box
	auto planeCollider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(0.5, 0.01f, 0.5, 0.0f));

	auto plane1 = std::make_shared<PhysicsObject>(static_cast<UINT>(4000), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);
	auto plane2 = std::make_shared<PhysicsObject>(static_cast<UINT>(4001), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);
	auto plane3 = std::make_shared<PhysicsObject>(static_cast<UINT>(4002), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);
	auto plane4 = std::make_shared<PhysicsObject>(static_cast<UINT>(4003), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);
	auto plane5 = std::make_shared<PhysicsObject>(static_cast<UINT>(4004), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);
	auto plane6 = std::make_shared<PhysicsObject>(static_cast<UINT>(4005), MeshType::Plane, GlobalData::g_planeMesh, GlobalData::g_defaultTexture);

	plane1->onLoad();
	plane2->onLoad();
	plane3->onLoad();
	plane4->onLoad();
	plane5->onLoad();
	plane6->onLoad();

	plane1->setCollider(planeCollider);
	plane2->setCollider(planeCollider);
	plane3->setCollider(planeCollider);
	plane4->setCollider(planeCollider);
	plane5->setCollider(planeCollider);
	plane6->setCollider(planeCollider);

	plane1->getTransform().SetPosition({ 0.0f, -1.5f, 0.0f, 1 }, 0, true);
	plane1->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane1->getTransform().SetRotationEulerAngles({ 0.0f, 0.0f, 0.0f }, 0, true);

	plane2->getTransform().SetPosition({ 0.0f, 1.5, 0.0f, 1 }, 0, true);
	plane2->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane2->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(180), 0.0f, 0.0f }, 0, true);

	plane3->getTransform().SetPosition({ -1.5, 0.0f, 0.0f, 1 }, 0, true);
	plane3->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane3->getTransform().SetRotationEulerAngles({ 0, 0, DirectX::XMConvertToRadians(-90) }, 0, true);

	plane4->getTransform().SetPosition({ 1.5, 0.0f, 0.0f, 1 }, 0, true);
	plane4->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane4->getTransform().SetRotationEulerAngles({ 0, 0, DirectX::XMConvertToRadians(90) }, 0, true);

	plane5->getTransform().SetPosition({ 0.0f, 0.0f, -1.5, 1 }, 0, true);
	plane5->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane5->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(90), 0, 0 }, 0, true);

	plane6->getTransform().SetPosition({ 0.0f, 0.0f, 1.5, 1 }, 0, true);
	plane6->getTransform().SetScale({ 3.0f, 1.0f, 3.0f, 1 }, 0, true);
	plane6->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(-90), 0, 0 }, 0, true);

	plane1->setStatic(true);
	plane2->setStatic(true);
	plane3->setStatic(true);
	plane4->setStatic(true);
	plane5->setStatic(true);
	plane6->setStatic(true);

	m_physicsObjects.push_back(plane1);
	m_physicsObjects.push_back(plane2);
	m_physicsObjects.push_back(plane3);
	m_physicsObjects.push_back(plane4);
	m_physicsObjects.push_back(plane5);
	m_physicsObjects.push_back(plane6);
	onLoadInternal(commandList);
}

void RoomScenario::onUnload(CommandList& commandList)
{
	for (auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onUnload();
	}
	m_physicsObjects.clear();
}

void RoomScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix)
{
	for (const auto& physicsObject : m_physicsObjects)
	{
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}
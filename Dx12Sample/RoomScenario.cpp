#include "RoomScenario.h"

#include "BoxCollider.h"
#include "SphereCollider.h"
#include "GlobalData.h"

#include <algorithm>
#include <random>
#include "CollisionSystem.h"

void RoomScenario::createWalls(CommandList& commandList) {

	// Create 6 planes to make a box
	if (m_wallsObjects.size() > 0)
	{
		for (auto& wallsObject : m_wallsObjects)
		{
			wallsObject->onUnload();
		}
		m_wallsObjects.clear();
	}

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

	DirectX::XMVECTOR scale_xz = DirectX::XMVectorSet(2 * m_bounds[0], 1, 2 * m_bounds[2], 1);
	DirectX::XMVECTOR scale_xy = DirectX::XMVectorSet(2 * m_bounds[0], 1, 2 * m_bounds[1], 1);
	DirectX::XMVECTOR scale_yz = DirectX::XMVectorSet(2 * m_bounds[1], 1, 2 * m_bounds[2], 1);

	plane1->getTransform().SetScale(scale_xz, 0, true);
	plane1->getTransform().SetPosition({ 0.0f, -m_bounds[1], 0.0f, 1 }, 0, true);
	plane1->getTransform().SetRotationEulerAngles({ 0.0f, 0.0f, 0.0f }, 0, true);

	plane2->getTransform().SetScale(scale_xz, 0, true);
	plane2->getTransform().SetPosition({ 0.0f, m_bounds[1], 0.0f, 1 }, 0, true);
	plane2->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(180), 0.0f, 0.0f }, 0, true);

	plane3->getTransform().SetScale(scale_yz, 0, true);
	plane3->getTransform().SetPosition({ -m_bounds[0], 0.0f, 0.0f, 1 }, 0, true);
	plane3->getTransform().SetRotationEulerAngles({ 0, 0, DirectX::XMConvertToRadians(-90) }, 0, true);

	plane4->getTransform().SetScale(scale_yz, 0, true);
	plane4->getTransform().SetPosition({ m_bounds[0], 0.0f, 0.0f, 1 }, 0, true);
	plane4->getTransform().SetRotationEulerAngles({ 0, 0, DirectX::XMConvertToRadians(90) }, 0, true);

	plane5->getTransform().SetScale(scale_xy, 0, true);
	plane5->getTransform().SetPosition({ 0.0f, 0.0f, -m_bounds[2], 1 }, 0, true);
	plane5->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(90), 0, 0 }, 0, true);

	plane6->getTransform().SetScale(scale_xy, 0, true);
	plane6->getTransform().SetPosition({ 0.0f, 0.0f, m_bounds[2], 1 }, 0, true);
	plane6->getTransform().SetRotationEulerAngles({ DirectX::XMConvertToRadians(-90), 0, 0 }, 0, true);

	plane1->setStatic(true);
	plane2->setStatic(true);
	plane3->setStatic(true);
	plane4->setStatic(true);
	plane5->setStatic(true);
	plane6->setStatic(true);

	m_wallsObjects.push_back(plane1);
	m_wallsObjects.push_back(plane2);
	m_wallsObjects.push_back(plane3);
	m_wallsObjects.push_back(plane4);
	m_wallsObjects.push_back(plane5);
	m_wallsObjects.push_back(plane6);

	if (m_physicsObjects.size() >= 6) {
		m_physicsObjects[0]->onUnload();
		m_physicsObjects[0] = plane1;

		m_physicsObjects[1]->onUnload();
		m_physicsObjects[1] = plane2;

		m_physicsObjects[2]->onUnload();
		m_physicsObjects[2] = plane3;

		m_physicsObjects[3]->onUnload();
		m_physicsObjects[3] = plane4;

		m_physicsObjects[4]->onUnload();
		m_physicsObjects[4] = plane5;

		m_physicsObjects[5]->onUnload();
		m_physicsObjects[5] = plane6;
	}
	else {
		m_physicsObjects.push_back(plane1);
		m_physicsObjects.push_back(plane2);
		m_physicsObjects.push_back(plane3);
		m_physicsObjects.push_back(plane4);
		m_physicsObjects.push_back(plane5);
		m_physicsObjects.push_back(plane6);
	}
}

void RoomScenario::onLoad(CommandList& commandList)
{
	createWalls(commandList);
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
#include "BallToCapsuleScenario.h"

#include "SphereCollider.h"
#include "CapsuleCollider.h"
#include "BoxCollider.h"

void BallToCapsuleScenario::onLoad(CommandList& commandList)
{
	m_CapsuleMesh = Mesh::CreateCapsule(commandList, 1.0f, 2.0f, 16, false);

	PhysicsMaterial material = {
		0.2f, // static friction
		.01f,
	};

	m_particle = std::make_shared<PhysicsObject>(MeshType::Sphere, GlobalData::g_sphereMesh.get(), GlobalData::g_customTexture.get());
	auto sphereCollider = std::make_shared<SphereCollider>(.5f);
	m_particle->setCollider(sphereCollider);
	m_particle->setPhysicsMaterial(material);
	m_particle->onLoad();
	m_particle->getTransform().SetPosition({ -3, 0.48, 1, 1 }, 0, true);
	//particle->setVelocity({ 2, 0, 0, 0 }, 1);
	//particle->setAngularVelocity({ 0, 0, 5, 0 }, 1);

	m_capsule = std::make_shared<PhysicsObject>(MeshType::Capsule, m_CapsuleMesh.get(), GlobalData::g_customTexture.get());
	auto capsuleCollider = std::make_shared<CapsuleCollider>(0.5f, 2.0f);
	m_capsule->setCollider(capsuleCollider);
	m_capsule->setPhysicsMaterial(material);
	m_capsule->onLoad();
	m_capsule->getTransform().SetPosition({ 0, -0.5, 1, 1 }, 0, true);
	m_capsule->getTransform().SetRotationEulerAngles({ 0, 0, DirectX::XMConvertToRadians(90) }, 0, true);
	m_capsule->setVelocity({ 2, 0, 0, 0 }, 1);

	m_box = std::make_shared<PhysicsObject>(MeshType::Box, GlobalData::g_boxMesh.get(), GlobalData::g_defaultTexture.get());
	auto boxCollider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(0.5f, 0.5f, 0.5f, 0.0f));
	m_box->setCollider(boxCollider);
	m_box->setPhysicsMaterial(material);
	m_box->onLoad();
	m_box->getTransform().SetPosition({ 2.5f, -.75f, 1.0f, 1 }, 0, true);

	m_physicsObjects.push_back(m_particle);
	m_physicsObjects.push_back(m_capsule);
	m_physicsObjects.push_back(m_box);
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

	ImGui::Begin("Scenario");
	// Capsule Position
	DirectX::XMFLOAT3 capsulePos;
	DirectX::XMStoreFloat3(&capsulePos, m_capsule->getTransform().GetPosition(0));
	ImGui::InputFloat3("Capsule Position", &capsulePos.x);
	m_capsule->getTransform().SetPosition(capsulePos, 0, true);

	// Capsule Rotation
	DirectX::XMFLOAT3 capsuleRot = m_capsule->getTransform().GetRotationEulerAngles(0);
	if (ImGui::InputFloat3("Capsule Rotation", &capsuleRot.x)) {
		m_capsule->getTransform().SetRotationEulerAngles(capsuleRot, 0, true);
	}

	// Box Position
	DirectX::XMFLOAT3 boxPos;
	DirectX::XMStoreFloat3(&boxPos, m_box->getTransform().GetPosition(0));
	if(ImGui::InputFloat3("Box Position", &boxPos.x)) {
		m_box->getTransform().SetPosition(boxPos, 0, true);
	}
	ImGui::End();
}
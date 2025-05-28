#include "SandboxScenario.h"

#include "BoxCollider.h"
#include "SphereCollider.h"
#include "CollisionSystem.h"

#include "CapsuleCollider.h"
#include "Application.h"
#include "CommandQueue.h"

#include <algorithm>
#include <random>

void SandboxScenario::onLoad(CommandList& commandList)
{

}

void SandboxScenario::onUnload(CommandList& commandList) {
	for (const auto& physicsObject : m_physicsObjects) {
		physicsObject->onUnload();
	}
	m_physicsObjects.clear();
}

void SandboxScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) {
	for (const auto& physicsObject : m_physicsObjects) {
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}

void SandboxScenario::drawImGui() {
	ImGui::Begin("Scenario");
	static PhysicsObject* selectedObject = nullptr;

	if (!selectedObject) {
		for (int i = 0; i < m_physicsObjects.size(); i++) {
			const auto& obj = m_physicsObjects[i];
			auto meshType = obj->getMeshType();
			if (meshType == MeshType::Sphere) {
				ImGui::Text("Sphere: ");
			}
			else if (meshType == MeshType::Capsule) {
				ImGui::Text("Capsule: ");
			}
			else if (meshType == MeshType::Box) {
				ImGui::Text("Box: ");
			}
			else {
				ImGui::Text("Unknown: ");
			}
			ImGui::SameLine();
			if (ImGui::Selectable(std::to_string(i).c_str(), selectedObject == obj.get())) {
				selectedObject = obj.get();
			}
		}

		if (ImGui::Button("Create Sphere")) {
			auto obj = std::make_shared<PhysicsObject>(MeshType::Sphere, GlobalData::g_sphereMesh.get(), GlobalData::g_customTexture.get());
			obj->setCollider(std::make_shared<SphereCollider>(0.5f));
			obj->getTransform().SetScale({ 1.0f, 1.0f, 1.0f, 1 }, 0, true);
			obj->getTransform().SetPosition({ 0, 0, 0, 1 }, 0, true);
			obj->onLoad();
			m_physicsObjects.push_back(obj);
		}

		ImGui::Separator();
		static float capsuleHeight = 1.0f;
		static float capsuleRadius = 0.1f;

		ImGui::InputFloat("Capsule Height", &capsuleHeight, 0.01f, 0.1f, "%.2f");
		ImGui::InputFloat("Capsule Radius", &capsuleRadius, 0.01f, 0.1f, "%.2f");

		if (ImGui::Button("Create Capsule")) {
			auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
			auto commandList = commandQueue->GetCommandList();

			auto mesh = GlobalData::createCapsuleMesh(*commandList, capsuleRadius, capsuleHeight, 16);
			auto obj = std::make_shared<PhysicsObject>(MeshType::Capsule, mesh.get(), GlobalData::g_customTexture.get());
			obj->setCollider(std::make_shared<CapsuleCollider>(capsuleRadius, capsuleHeight));
			obj->getTransform().SetScale({ 1.0f, 1.0f, 1.0f, 1 }, 0, true);
			obj->getTransform().SetPosition({ 0, 0, 0, 1 }, 0, true);
			obj->onLoad();

			auto fenceValue = commandQueue->ExecuteCommandList(commandList);
			commandQueue->WaitForFenceValue(fenceValue);

			m_physicsObjects.push_back(obj);
		}
	}
	else {
		if (ImGui::Button("Clear Selection")) {
			selectedObject = nullptr;
		}
		if (ImGui::Button("Delete Object")) {
			auto it = std::find_if(m_physicsObjects.begin(), m_physicsObjects.end(),
				[](const std::shared_ptr<PhysicsObject>& obj) {
					return obj.get() == selectedObject;
				});
			if (it != m_physicsObjects.end()) {
				(*it)->onUnload();
				m_physicsObjects.erase(it);
				selectedObject = nullptr;
			}
		}

		auto colliderType = selectedObject->getCollider()->getType();
		{
			// Position
			ImGui::Text("Position:");
			DirectX::XMFLOAT3 position;
			DirectX::XMStoreFloat3(&position, selectedObject->getTransform().GetPosition(0));
			ImGui::InputFloat3("##Position", &position.x, "%.2f");
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				selectedObject->getTransform().SetPosition(position, 0, true);
			}
		}
		ImGui::Separator();
		{
			// Rotation
			ImGui::Text("Rotation:");
			DirectX::XMFLOAT3 rotation = selectedObject->getTransform().GetRotationEulerAngles(0);
			ImGui::InputFloat3("##Rotation", &rotation.x, "%.2f");
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				selectedObject->getTransform().SetRotationEulerAngles(rotation, 0, true);
			}
			ImGui::Separator();
		}
		{
			// Scale
			if (colliderType != ColliderType::Capsule) {
				// Only show scale for non-capsule colliders
				ImGui::Text("Scale:");
				DirectX::XMFLOAT3 scale;
				DirectX::XMStoreFloat3(&scale, selectedObject->getTransform().GetScale(0));
				ImGui::InputFloat3("##Scale", &scale.x, "%.2f");
				if (ImGui::IsItemDeactivatedAfterEdit()) {
					selectedObject->getTransform().SetScale(scale, 0, true);
				}
				ImGui::Separator();
			}
		}
		{
			// Physics Material
			ImGui::Text("Physics Material:");
			PhysicsMaterial material = selectedObject->getPhysicsMaterial();
			ImGui::DragFloat("Friction", &material.friction, 0.01f, 0.0f, 1.0f);
			ImGui::DragFloat("Restitution", &material.restitution, 0.01f, 0.0f, 1.0f);
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				selectedObject->setPhysicsMaterial(material);
			}
			ImGui::Separator();
		}
		{
			// Collider Size
			{
				ImGui::Text("Collider Type: %s", colliderType == ColliderType::Sphere ? "Sphere" :
					colliderType == ColliderType::Box ? "Box" : "Capsule");
				if (colliderType == ColliderType::Sphere) {
					float radius = static_cast<SphereCollider*>(selectedObject->getCollider())->getRadius();
					if (ImGui::DragFloat("Sphere Radius", &radius, 0.01f, 0.01f, 10.0f)) {
						static_cast<SphereCollider*>(selectedObject->getCollider())->setRadius(radius);
					}
				}
				else if (colliderType == ColliderType::Box) {
					DirectX::XMFLOAT3 boxHalfSize;
					DirectX::XMStoreFloat3(&boxHalfSize, static_cast<BoxCollider*>(selectedObject->getCollider())->getHalfSize());
					if (ImGui::InputFloat3("Box Half Size", &boxHalfSize.x, "%.2f")) {
						static_cast<BoxCollider*>(selectedObject->getCollider())->setHalfSize(DirectX::XMLoadFloat3(&boxHalfSize));
					}
				}
				ImGui::Separator();
			}
		}
		{
			// Velocity
			ImGui::Text("Velocity:");
			DirectX::XMFLOAT3 velocity;
			DirectX::XMStoreFloat3(&velocity, selectedObject->getVelocity(1));
			ImGui::InputFloat3("##Velocity", &velocity.x, "%.2f");
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				selectedObject->setVelocity(DirectX::XMLoadFloat3(&velocity), 1);
			}
			ImGui::Separator();
		}
		{
			// Angular Velocity
			ImGui::Text("Angular Velocity:");
			DirectX::XMFLOAT3 angularVelocity;
			DirectX::XMStoreFloat3(&angularVelocity, selectedObject->getAngularVelocity(1));
			ImGui::InputFloat3("##AngularVelocity", &angularVelocity.x, "%.2f");
			if (ImGui::IsItemDeactivatedAfterEdit()) {
				selectedObject->setAngularVelocity(DirectX::XMLoadFloat3(&angularVelocity), 1);
			}
			ImGui::Separator();
		}

	}

	ImGui::Separator();
	ImGui::End();
}
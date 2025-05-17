#include "SpheresScenario.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include "GlobalData.h"
#include "CollisionSystem.h"

#include <algorithm>
#include <random>
#include <Application.h>
#include <CommandQueue.h>

void SpheresScenario::onLoadInternal(CommandList& commandList)
{
	std::normal_distribution<float> d{ 0.0, 1.0 };
	PhysicsMaterial material = {
		0.02f,
		0.02,
		1
	};
	CollisionSystem collisionSystem;
	for (int i = 0; i < 50; i++)
	{
		int randNum = rand() % 2;
		std::shared_ptr<PhysicsObject> obj = nullptr;
		float x = 0, y = 0, z = 0;
		obj = std::make_shared<PhysicsObject>(static_cast<UINT>(i), MeshType::Sphere, GlobalData::g_sphereMesh, GlobalData::g_customTexture);
		auto sphereCollider = std::make_shared<SphereCollider>(.5f);
		obj->setCollider(sphereCollider);
		obj->setPhysicsMaterial(material);
		float radius = std::clamp(d(m_randomEngine), m_minimumRadius, m_maximumRadius);
		obj->getTransform().SetScale({ radius, radius, radius, 1 }, 0, true);

		obj->setStatic(randNum % 2);

		int cnt = 50;
		while (cnt-- > 0) {
			float x = std::clamp(d(m_randomEngine), -m_bounds[0], m_bounds[0]);
			float y = std::clamp(d(m_randomEngine), -m_bounds[1], m_bounds[1]);
			float z = std::clamp(d(m_randomEngine), -m_bounds[2], m_bounds[2]);
			obj->getTransform().SetPosition({ x, y, z, 1 }, 0, true);

			bool found = false;
			for (auto& o : m_physicsObjects) {
				if (collisionSystem.checkCollision(obj.get(), o.get()) != std::nullopt) {
					found = true;
					break;
				}
			}
			if (!found)
				break;
		}
		if (cnt <= 0) {
			OutputDebugStringA("Failed to find a position for the object\n");
			continue;
		}
		obj->onLoad();
		m_physicsObjects.push_back(obj);
	}
}

void SpheresScenario::drawImGui() {
	ImGui::Begin("Scenario");

	// Bounds control
	ImGui::DragFloat3("Bounds", m_bounds.data(), 0.1f, 0.0f, 100.0f);

	// Minimum and maximum radius
	ImGui::Text("Minimum Radius: %.2f", m_minimumRadius);
	ImGui::Text("Maximum Radius: %.2f", m_maximumRadius);

	if (ImGui::Button("Generate Spheres")) {
		// Clear existing objects from index 6 and later
		for (size_t i = 6; i < m_physicsObjects.size(); ++i) {
			m_physicsObjects[i]->onUnload();
		}
		m_physicsObjects.erase(m_physicsObjects.begin() + 6, m_physicsObjects.end());

		auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
		auto commandList = commandQueue->GetCommandList();

		// Generate new spheres
		onLoadInternal(*commandList);

		auto fenceValue = commandQueue->ExecuteCommandList(commandList);
		commandQueue->WaitForFenceValue(fenceValue);
	}

	ImGui::End();
}
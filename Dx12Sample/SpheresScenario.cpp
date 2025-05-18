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
	std::uniform_real_distribution<float> dx{ -m_bounds[0], m_bounds[0] };
	std::uniform_real_distribution<float> dy{ -m_bounds[1], m_bounds[1] };
	std::uniform_real_distribution<float> dz{ -m_bounds[2], m_bounds[2] };
	std::uniform_real_distribution<float> dr{ m_minimumDiameter, m_maximumDiameter };

	PhysicsMaterial material = {
		0.02f,
		0.02,
		1
	};
	CollisionSystem collisionSystem;
	for (int i = 0; i < m_spheresCount; i++)
	{
		std::shared_ptr<PhysicsObject> obj = nullptr;
		obj = std::make_shared<PhysicsObject>(static_cast<UINT>(i), MeshType::Sphere, GlobalData::g_sphereMesh, GlobalData::g_customTexture);
		auto sphereCollider = std::make_shared<SphereCollider>(.5f);

		obj->setCollider(sphereCollider);
		obj->setPhysicsMaterial(material);

		float diameter = std::clamp(dr(m_randomEngine), m_minimumDiameter, m_maximumDiameter);
		obj->getTransform().SetScale({ diameter, diameter, diameter, 1 }, 0, true);

		int cnt = 10;
		float x = 0, y = 0, z = 0;
		while (cnt-- > 0) {
			x = std::clamp(dx(m_randomEngine), -m_bounds[0] + diameter / 2.0f, m_bounds[0] - diameter / 2.0f);
			y = std::clamp(dy(m_randomEngine), -m_bounds[1] + diameter / 2.0f, m_bounds[1] - diameter / 2.0f);
			z = std::clamp(dz(m_randomEngine), -m_bounds[2] + diameter / 2.0f, m_bounds[2] - diameter / 2.0f);
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
			continue;
		}
		std::bernoulli_distribution makeStatic(0.5f);
		obj->setStatic(makeStatic(m_randomEngine));

		obj->onLoad();
		m_physicsObjects.push_back(obj);
	}
}

void SpheresScenario::drawImGui() {
	ImGui::Begin("Scenario");

	// Bounds control
	ImGui::DragFloat3("Bounds", m_bounds.data(), 0.1f, 0.0f, 100.0f);

	// Minimum and maximum radius control
	ImGui::DragFloat("Minimum Radius", &m_minimumDiameter, 0.1f, 0.0f, 10.0f);
	ImGui::DragFloat("Maximum Radius", &m_maximumDiameter, 0.1f, 0.0f, 10.0f);

	// Sphere Count control
	ImGui::DragInt("Count", &m_spheresCount, 1, 0, 10000);

	if (ImGui::Button("Generate Spheres")) {

		// Clear existing objects from index 6 and later
		for (size_t i = 6; i < m_physicsObjects.size(); ++i) {
			m_physicsObjects[i]->onUnload();
		}
		m_physicsObjects.erase(m_physicsObjects.begin() + 6, m_physicsObjects.end());



		auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
		auto commandList = commandQueue->GetCommandList();

		createWalls(*commandList);

		// Generate new spheres
		onLoadInternal(*commandList);

		auto fenceValue = commandQueue->ExecuteCommandList(commandList);
		commandQueue->WaitForFenceValue(fenceValue);
	}

	ImGui::End();
}
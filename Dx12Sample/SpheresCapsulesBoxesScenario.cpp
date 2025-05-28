#include "SpheresCapsulesBoxesScenario.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include "CollisionSystem.h"

#include "CapsuleCollider.h"
#include "Application.h"
#include "CommandQueue.h"

#include <algorithm>
#include <random>

void SpheresCapsulesBoxesScenario::onLoadInternal(CommandList& commandList)
{
	std::uniform_real_distribution<float> dx{ -m_bounds[0], m_bounds[0] };
	std::uniform_real_distribution<float> dy{ -m_bounds[1], m_bounds[1] };
	std::uniform_real_distribution<float> dz{ -m_bounds[2], m_bounds[2] };
	std::uniform_real_distribution<float> dr{ m_minimumDiameter, m_maximumDiameter };

	CollisionSystem collisionSystem;
	std::bernoulli_distribution makeStatic(m_staticSpheresRatio);
	for (int i = 0; i < m_spheresCount; i++)
	{
		bool stat = makeStatic(m_randomEngine);
		std::shared_ptr<PhysicsObject> obj = nullptr;
		obj = std::make_shared<PhysicsObject>(MeshType::Sphere, GlobalData::g_sphereMesh.get(), stat ? GlobalData::g_staticTexture.get() : GlobalData::g_customTexture.get());
		auto sphereCollider = std::make_shared<SphereCollider>(.5f);

		float randomFriction = std::uniform_real_distribution<float>(m_minimumFriction, m_maximumFriction)(m_randomEngine);
		float randomRestitution = std::uniform_real_distribution<float>(m_minimumRestitution, m_maximumRestitution)(m_randomEngine);

		PhysicsMaterial material = {
			randomFriction,
			randomRestitution
		};

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

		obj->setStatic(stat);
		obj->onLoad();
		m_physicsObjects.push_back(obj);
	}

	std::uniform_real_distribution<float> dh{ m_minimumCapsuleHeight, m_maximumCapsuleHeight };
	std::uniform_real_distribution<float> drc{ m_minimumCapsuleRadius, m_maximumCapsuleRadius };

	for (int i = 0; i < m_capsulesCount; i++)
	{
		std::shared_ptr<PhysicsObject> obj = nullptr;

		float height = std::clamp(dh(m_randomEngine), m_minimumCapsuleHeight, m_maximumCapsuleHeight);
		float radius = std::clamp(drc(m_randomEngine), m_minimumCapsuleRadius, m_maximumCapsuleRadius);

		auto capsuleMesh = GlobalData::createCapsuleMesh(commandList, radius, height, 16);

		obj = std::make_shared<PhysicsObject>(MeshType::Capsule, capsuleMesh.get(), GlobalData::g_staticTexture.get());
		auto capsuleCollider = std::make_shared<CapsuleCollider>(radius, height);
		float randomFriction = std::uniform_real_distribution<float>(m_minimumFriction, m_maximumFriction)(m_randomEngine);
		float randomRestitution = std::uniform_real_distribution<float>(m_minimumRestitution, m_maximumRestitution)(m_randomEngine);
		PhysicsMaterial material = {
			randomFriction,
			randomRestitution
		};
		obj->setCollider(capsuleCollider);
		obj->setPhysicsMaterial(material);
		int cnt = 10;
		float x = 0, y = 0, z = 0;
		float xRot, yRot, zRot;
		while (cnt-- > 0) {
			x = std::clamp(dx(m_randomEngine), -m_bounds[0] + radius / 2.0f, m_bounds[0] - radius / 2.0f);
			y = std::clamp(dy(m_randomEngine), -m_bounds[1] + height / 2.0f, m_bounds[1] - height / 2.0f);
			z = std::clamp(dz(m_randomEngine), -m_bounds[2] + radius / 2.0f, m_bounds[2] - radius / 2.0f);

			xRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);
			yRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);
			zRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);

			obj->getTransform().SetPosition({ x, y, z, 1 }, 0, true);
			obj->getTransform().SetRotationEulerAngles({ xRot, yRot, zRot }, 0, true);
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
		obj->setStatic(true);
		obj->onLoad();
		m_physicsObjects.push_back(obj);
	}

	std::uniform_real_distribution<float> dbs{ m_minimumBoxSize, m_maximumBoxSize };
	std::uniform_real_distribution<float> dbr{ m_minimumBoxSize, m_maximumBoxSize };

	for (int i = 0; i < m_boxesCount; i++)
	{
		std::shared_ptr<PhysicsObject> obj = nullptr;
		auto boxMesh = GlobalData::g_boxMesh;
		obj = std::make_shared<PhysicsObject>(MeshType::Box, boxMesh.get(), GlobalData::g_staticTexture.get());
		auto boxCollider = std::make_shared<BoxCollider>(DirectX::XMVectorSet(0.5f, 0.5f, 0.5f, 1));
		float randomFriction = std::uniform_real_distribution<float>(m_minimumFriction, m_maximumFriction)(m_randomEngine);
		float randomRestitution = std::uniform_real_distribution<float>(m_minimumRestitution, m_maximumRestitution)(m_randomEngine);
		PhysicsMaterial material = {
			randomFriction,
			randomRestitution
		};

		float sizeX = std::clamp(dbs(m_randomEngine), m_minimumBoxSize, m_maximumBoxSize);
		float sizeY = std::clamp(dbs(m_randomEngine), m_minimumBoxSize, m_maximumBoxSize);
		float sizeZ = std::clamp(dbr(m_randomEngine), m_minimumBoxSize, m_maximumBoxSize);
		obj->getTransform().SetScale({ sizeX, sizeY, sizeZ, 1 }, 0, true);

		obj->setCollider(boxCollider);
		obj->setPhysicsMaterial(material);
		int cnt = 10;
		float x = 0, y = 0, z = 0;
		float xRot, yRot, zRot;
		while (cnt-- > 0) {
			x = std::clamp(dx(m_randomEngine), -m_bounds[0] + sizeX / 2.0f, m_bounds[0] - sizeX / 2.0f);
			y = std::clamp(dy(m_randomEngine), -m_bounds[1] + sizeY / 2.0f, m_bounds[1] - sizeY / 2.0f);
			z = std::clamp(dz(m_randomEngine), -m_bounds[2] + sizeZ / 2.0f, m_bounds[2] - sizeZ / 2.0f);

			xRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);
			yRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);
			zRot = std::uniform_real_distribution<float>(0.0f, 360.0f)(m_randomEngine);

			obj->getTransform().SetPosition({ x, y, z, 1 }, 0, true);
			obj->getTransform().SetRotationEulerAngles({ xRot, yRot, zRot }, 0, true);
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
		obj->setStatic(true);
		obj->onLoad();
		m_physicsObjects.push_back(obj);
	}

}

void SpheresCapsulesBoxesScenario::drawImGui() {
	ImGui::Begin("Scenario");

	if (ImGui::CollapsingHeader("Spawning")) {
		ImGui::DragFloat3("Bounds", m_bounds.data(), 0.1f, 0.0f, 100.0f);

		ImGui::Text("Sphere Settings");
		ImGui::DragFloat("Minimum Radius", &m_minimumDiameter, 0.1f, 0.0f, 10.0f);
		ImGui::DragFloat("Maximum Radius", &m_maximumDiameter, 0.1f, 0.0f, 10.0f);
		if (ImGui::DragFloat("Static Spheres Ratio", &m_staticSpheresRatio, 0.01f, 0.0f, 1.0f)) {
			m_staticSpheresRatio = std::clamp(m_staticSpheresRatio, 0.0f, 1.0f);
		}
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Capsule Settings");
		ImGui::DragFloat("Minimum Capsule Height", &m_minimumCapsuleHeight, 0.01f, 0.0f, 10.0f);
		ImGui::DragFloat("Maximum Capsule Height", &m_maximumCapsuleHeight, 0.01f, 0.0f, 10.0f);

		ImGui::DragFloat("Minimum Capsule Radius", &m_minimumCapsuleRadius, 0.01f, 0.0f, 10.0f);
		ImGui::DragFloat("Maximum Capsule Radius", &m_maximumCapsuleRadius, 0.01f, 0.0f, 10.0f);
		ImGui::Separator();
		ImGui::Spacing();


		ImGui::DragInt("Spheres Count", &m_spheresCount, 1, 0, 10000);
		ImGui::DragInt("Capsules Count", &m_capsulesCount, 1, 0, 10000);
		ImGui::Separator();
		ImGui::Spacing();
	}

	ImGui::Spacing();
	if (ImGui::CollapsingHeader("Physics")) {
		ImGui::DragFloat("Minimum Friction", &m_minimumFriction, 0.01f, 0.0f, 1.0f);
		ImGui::DragFloat("Maximum Friction", &m_maximumFriction, 0.01f, 0.0f, 1.0f);

		ImGui::DragFloat("Minimum Restitution", &m_minimumRestitution, 0.01f, 0.0f, 1.0f);
		ImGui::DragFloat("Maximum Restitution", &m_maximumRestitution, 0.01f, 0.0f, 1.0f);
	}

	if (ImGui::Button("Generate")) {
		setReady(false);

		for (size_t i = 6; i < m_physicsObjects.size(); ++i) {
			m_physicsObjects[i]->onUnload();
		}
		m_physicsObjects.erase(m_physicsObjects.begin() + 6, m_physicsObjects.end());

		auto commandQueue = Application::Get().GetCommandQueue(D3D12_COMMAND_LIST_TYPE_COPY);
		auto commandList = commandQueue->GetCommandList();

		createWalls(*commandList);

		onLoadInternal(*commandList);

		auto fenceValue = commandQueue->ExecuteCommandList(commandList);
		commandQueue->WaitForFenceValue(fenceValue);
	}

	ImGui::End();
}
#include "ScenarioB.h"
#include "BoxCollider.h"
#include "SphereCollider.h"
#include <algorithm>
#include <random>

void ScenarioB::onLoadInternal(CommandList& commandList)
{
	std::normal_distribution<float> d{ 0.0, 3.0 };
	PhysicsMaterial material = {
		0.5f, // static friction
		0.5,
		0.7
	};
	for (int i = 0; i < 50; i++)
	{
		int randNum = 0;// rand() % 2;
		if (randNum) {
			// Create a static box
			auto box = std::make_shared<PhysicsObject>(static_cast<UINT>(4000), MeshType::Box, GlobalData::g_boxMesh, GlobalData::g_defaultTexture);
			auto sphereCollider = std::make_shared<BoxCollider>(DirectX::XMVectorReplicate(.5f));
			box->setCollider(sphereCollider);
			box->setPhysicsMaterial(material);
			float x = std::clamp(d(m_randomEngine), -m_bounds[0], m_bounds[0]);
			float y = std::clamp(d(m_randomEngine), -m_bounds[1], m_bounds[1]);
			float z = std::clamp(d(m_randomEngine), -m_bounds[2], m_bounds[2]);
			box->onLoad();
			box->setStatic(true);
			box->getTransform().SetPosition({ x, y, z, 1 }, 0, true);
			m_physicsObjects.push_back(box);
		}
		else {
			// Create a sphere
			auto particle = std::make_shared<PhysicsObject>(static_cast<UINT>(4000), MeshType::Sphere, GlobalData::g_sphereMesh, GlobalData::g_customTexture);
			auto sphereCollider = std::make_shared<SphereCollider>(.5f);
			particle->setCollider(sphereCollider);
			particle->setPhysicsMaterial(material);
			float x = std::clamp(d(m_randomEngine), -9.0f, 9.0f);
			float y = std::clamp(d(m_randomEngine), -9.0f, 9.0f);
			float z = std::clamp(d(m_randomEngine), -9.0f, 9.0f);
			particle->onLoad();
			particle->getTransform().SetPosition({ x, y, z, 1 }, 0, true);
			m_physicsObjects.push_back(particle);
			//particle->setVelocity({ 5, 0, 0, 0 }, 0);
			//particle->setVelocity({ 5, 0, 0, 0 }, 1);
		}
	}
}

void ScenarioB::drawImGui() {

}
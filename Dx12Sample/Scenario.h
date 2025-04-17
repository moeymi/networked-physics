#pragma once
#include <memory>
#include <vector>
#include "PhysicsObject.h"
#include "Texture.h"
#include "PhysicsEngine.h"

class Scenario
{
protected:
	std::vector<std::shared_ptr<PhysicsObject>> m_physicsObjects;
	PhysicsEngine m_physicsEngine;

public:
	Scenario() = default;
	virtual ~Scenario() = default;

	void virtual onLoad(CommandList& commandList) = 0;
	void virtual onUpdate(const float& dt) = 0;
	void virtual onUnload(CommandList& commandList) = 0;
	void virtual onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) = 0;

	std::vector<std::shared_ptr<PhysicsObject>>& getPhysicsObjects() { return m_physicsObjects; }
};

class ConcreteScenarioA : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr<Mesh> m_BoxMesh;
	std::shared_ptr<Mesh> m_PlaneMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	ConcreteScenarioA() = default;
	virtual ~ConcreteScenarioA() override = default;

	void onLoad(CommandList& commandList) override;
	void onUpdate(const float& dt) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};

class ConcreteScenarioB : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr<Mesh> m_PlaneMesh;
	std::shared_ptr<Mesh> m_BoxMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	ConcreteScenarioB() = default;
	virtual ~ConcreteScenarioB() override = default;

	void onLoad(CommandList& commandList) override;
	void onUpdate(const float& dt) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
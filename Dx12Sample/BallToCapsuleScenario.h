#pragma once

#include "pch.h"
#include "Scenario.h"

class BallToCapsuleScenario : public Scenario
{
	std::shared_ptr<Mesh> m_CapsuleMesh;
	std::shared_ptr<PhysicsObject> m_capsule;
	std::shared_ptr<PhysicsObject> m_particle;
	std::shared_ptr<PhysicsObject> m_box;

public:
	BallToCapsuleScenario() = default;
	virtual ~BallToCapsuleScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void drawImGui() override;
};
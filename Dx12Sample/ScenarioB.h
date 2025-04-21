#pragma once
#include "Scenario.h"

class ScenarioB : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr<Mesh> m_BoxMesh;
	std::shared_ptr<Mesh> m_PlaneMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	ScenarioB() = default;
	virtual ~ScenarioB() override = default;

	void onLoad(CommandList& commandList) override;
	void onUpdate(const float& dt) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
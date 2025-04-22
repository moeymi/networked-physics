#pragma once
#include "Scenario.h"

class ScenarioA : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr<Mesh> m_BoxMesh;
	std::shared_ptr<Mesh> m_PlaneMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	ScenarioA() = default;
	virtual ~ScenarioA() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
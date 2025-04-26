#pragma once
#include "pch.h"
#include "Scenario.h"

class ThreeByThreeCubeScenario : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	ThreeByThreeCubeScenario() = default;
	virtual ~ThreeByThreeCubeScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
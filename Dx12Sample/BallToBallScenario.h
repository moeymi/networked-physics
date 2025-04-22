#pragma once
#include "Scenario.h"

class BallToBallScenario : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	BallToBallScenario() = default;
	virtual ~BallToBallScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
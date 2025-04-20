#pragma once
#include "Scenario.h"

class BallToCapsuleScenario : public Scenario
{
	std::shared_ptr<Mesh> m_SphereMesh;
	std::shared_ptr<Mesh> m_CapsuleMesh;
	std::shared_ptr <Texture> m_customTexture;
	std::shared_ptr <Texture> m_defaultTexture;

public:
	BallToCapsuleScenario() = default;
	virtual ~BallToCapsuleScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUpdate(const float& dt) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
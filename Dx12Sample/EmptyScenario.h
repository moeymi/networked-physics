#pragma once
#include "Scenario.h"
class EmptyScenario : public Scenario
{
public:
	EmptyScenario(std::vector<std::shared_ptr<PhysicsObject>>&&);
	virtual ~EmptyScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void drawImGui() override;
};


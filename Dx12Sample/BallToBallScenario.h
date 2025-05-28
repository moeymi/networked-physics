#pragma once
#include "pch.h"
#include "Scenario.h"

class BallToBallScenario : public Scenario
{
public:
	BallToBallScenario() = default;
	virtual ~BallToBallScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void drawImGui() override;
};
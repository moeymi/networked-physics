#pragma once

#include "pch.h"
#include "Scenario.h"

class BallToWallScenario : public Scenario
{
public:
	BallToWallScenario() = default;
	virtual ~BallToWallScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void virtual drawImGui() override;
};
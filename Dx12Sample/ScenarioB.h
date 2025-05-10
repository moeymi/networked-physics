#pragma once
#include "pch.h"
#include "Scenario.h"

class ScenarioB : public Scenario
{
public:
	ScenarioB() = default;
	virtual ~ScenarioB() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
};
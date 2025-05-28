#pragma once
#include "pch.h"
#include "Scenario.h"

class SandboxScenario : public Scenario
{
public:
	SandboxScenario() = default;
	virtual ~SandboxScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void drawImGui() override;
};


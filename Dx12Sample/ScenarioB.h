#pragma once
#include "pch.h"
#include "RoomScenario.h"

class ScenarioB : public RoomScenario
{
public:
	ScenarioB() = default;
	virtual ~ScenarioB() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
#pragma once
#include "pch.h"
#include "RoomScenario.h"

class SpheresScenario : public RoomScenario
{
private:
	float m_minimumRadius = 0.25f;
	float m_maximumRadius = 0.5f;
public:
	SpheresScenario() = default;
	virtual ~SpheresScenario() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
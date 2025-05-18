#pragma once
#include "pch.h"
#include "RoomScenario.h"

class SpheresScenario : public RoomScenario
{
private:
	float m_minimumDiameter = 0.25f;
	float m_maximumDiameter = 0.25f;
	int m_spheresCount = 2;
public:
	SpheresScenario() = default;
	virtual ~SpheresScenario() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
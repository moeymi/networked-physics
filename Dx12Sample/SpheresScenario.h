#pragma once
#include "pch.h"
#include "RoomScenario.h"

class SpheresScenario : public RoomScenario
{
private:
	float m_minimumDiameter = 0.35f;
	float m_maximumDiameter = 0.55f;
	int m_spheresCount = 50;

	float m_staticSpheresRatio = 0.2f;

	float m_minimumFriction = 0.2f;
	float m_maximumFriction = 0.5f;
	float m_minimumRestitution = 0.4f;
	float m_maximumRestitution = 0.8f;

public:
	SpheresScenario() = default;
	virtual ~SpheresScenario() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
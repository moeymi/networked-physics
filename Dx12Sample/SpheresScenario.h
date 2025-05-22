#pragma once
#include "pch.h"
#include "RoomScenario.h"

class SpheresScenario : public RoomScenario
{
private:
	float m_minimumDiameter = 0.25f;
	float m_maximumDiameter = 0.25f;
	int m_spheresCount = 50;

	float m_staticSpheresRatio = 0.5f;

	float m_friction = 0.5f;
	float m_angularFriction = 0.5f;
	float m_restitution = 1.0f;

public:
	SpheresScenario() = default;
	virtual ~SpheresScenario() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
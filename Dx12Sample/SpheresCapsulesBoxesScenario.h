#pragma once
#include "pch.h"
#include "RoomScenario.h"

class SpheresCapsulesBoxesScenario : public RoomScenario
{
private:
	float m_minimumDiameter = 0.35f;
	float m_maximumDiameter = 0.55f;

	float m_minimumCapsuleHeight = 0.5f;
	float m_maximumCapsuleHeight = 0.8f;

	float m_minimumCapsuleRadius = 0.1f;
	float m_maximumCapsuleRadius = 0.3f;

	float m_minimumBoxSize = 0.4f;
	float m_maximumBoxSize = 0.9f;

	int m_spheresCount = 50;
	int m_capsulesCount = 50;
	int m_boxesCount = 50;


	float m_staticSpheresRatio = 0.2f;

	float m_minimumFriction = 0.2f;
	float m_maximumFriction = 0.5f;
	float m_minimumRestitution = 0.4f;
	float m_maximumRestitution = 0.8f;

public:
	SpheresCapsulesBoxesScenario() = default;
	virtual ~SpheresCapsulesBoxesScenario() override = default;

	void onLoadInternal(CommandList& commandList) override;
	void drawImGui() override;
};
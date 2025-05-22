#pragma once
#include "pch.h"
#include "Scenario.h"

class RoomScenario : public Scenario
{
protected: 
	std::array<float, 3> m_bounds;
	std::vector<std::shared_ptr<PhysicsObject>> m_wallsObjects;

	void createWalls(CommandList& commandList);

public:
	RoomScenario() : m_bounds({ 5, 5, 5 }) {};
	virtual ~RoomScenario() override = default;

	void onLoad(CommandList& commandList) override;
	void onUnload(CommandList& commandList) override;
	void onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) override;
	void virtual onLoadInternal(CommandList& commandList) = 0;
};
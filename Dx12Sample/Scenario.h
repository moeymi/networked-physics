#pragma once
#include "pch.h"
#include <memory>
#include <vector>
#include <random>
#include "imgui.h"
#include "Texture.h"
#include "PhysicsObject.h"
#include "GlobalData.h"

class Scenario
{
protected:
	std::vector<std::shared_ptr<PhysicsObject>> m_physicsObjects;
	static std::default_random_engine m_randomEngine;

public:
	Scenario() = default;
	virtual ~Scenario() = default;

	void virtual onLoad(CommandList& commandList) = 0;
	void virtual onUnload(CommandList& commandList) = 0;
	void virtual onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) = 0;
	void virtual drawImGui() = 0;

	std::vector<std::shared_ptr<PhysicsObject>>& getPhysicsObjects() { return m_physicsObjects; }
};
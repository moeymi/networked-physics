#include "EmptyScenario.h"

EmptyScenario::EmptyScenario(std::vector<std::shared_ptr<PhysicsObject>>&& objects) : Scenario() {
	m_physicsObjects = std::move(objects);
}

void EmptyScenario::onLoad(CommandList& commandList) {
	for (const auto& physicsObject : m_physicsObjects) {
		physicsObject->onLoad();
	}
}

void EmptyScenario::onUnload(CommandList& commandList) {
	for (const auto& physicsObject : m_physicsObjects) {
		physicsObject->onUnload();
	}
	m_physicsObjects.clear();
}

void EmptyScenario::onRender(CommandList& commandList, const DirectX::XMMATRIX& viewMatrix, const DirectX::XMMATRIX& viewProjectionMatrix) {
	for (const auto& physicsObject : m_physicsObjects) {
		physicsObject->onRender(commandList, viewMatrix, viewProjectionMatrix);
	}
}

void EmptyScenario::drawImGui() {

}
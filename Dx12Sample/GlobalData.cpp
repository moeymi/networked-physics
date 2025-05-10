#include "pch.h"
#include "GlobalData.h"

namespace GlobalData {
	uint16_t g_clientId = 0;
	uint16_t g_listenPort = 54000;
	std::string g_clientName = "PhysicsSimulation";
	DirectX::XMFLOAT4 g_clientColor = { 1.0f, 1.0f, 1.0f, 1.0f };

	std::shared_ptr<Texture> g_customTexture = nullptr;
	std::shared_ptr<Texture> g_defaultTexture = nullptr;

	std::shared_ptr<Mesh> g_sphereMesh = nullptr;
	std::shared_ptr<Mesh> g_boxMesh = nullptr;
	std::shared_ptr<Mesh> g_planeMesh = nullptr;
}
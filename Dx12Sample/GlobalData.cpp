#include "pch.h"
#include "GlobalData.h"

namespace GlobalData {
	uint16_t g_clientId = 0;
	uint16_t g_listenPort = 54000;
	USHORT g_broadcastPort = 6001;
	std::string g_clientName = "PhysicsSimulation";
	DirectX::XMFLOAT4 g_clientColor = { 1.0f, 0.0f, 0.0f, 1.0f };

	double g_renderingFPS = 0.0;
	double g_physicsFPS = 0.0;
	double g_networkFPS = 0.0;

	std::shared_ptr<Texture> g_customTexture = nullptr;
	std::shared_ptr<Texture> g_defaultTexture = nullptr;

	std::shared_ptr<Mesh> g_sphereMesh = nullptr;
	std::shared_ptr<Mesh> g_boxMesh = nullptr;
	std::shared_ptr<Mesh> g_planeMesh = nullptr;

	double getTimestamp()
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto epoch = now.time_since_epoch();
		return std::chrono::duration<double>(epoch).count();
	}

}
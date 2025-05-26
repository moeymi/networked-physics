#include "pch.h"
#include "GlobalData.h"

#include <functional>


namespace GlobalData {
	uint16_t g_clientId = 0;
	uint16_t g_listenPort = 6001;
	USHORT g_broadcastPort = 6005;
	std::string g_clientName = "Client A";
	DirectX::XMFLOAT4 g_clientColor = { 1.0f, 0.0f, 0.0f, 1.0f };

	double g_renderingFPS = 0.0;

	int g_physicsFreq = 120.0; // Hz
	double g_physicsDt = 0.0;

	int g_networkFreq = 60; // Hz
	double g_networkDt = 0.0;

	int g_tick = 0;

	std::shared_ptr<Texture> g_customTexture = nullptr;
	std::shared_ptr<Texture> g_staticTexture = nullptr;
	std::shared_ptr<Texture> g_defaultTexture = nullptr;

	std::shared_ptr<Mesh> g_sphereMesh = nullptr;
	std::shared_ptr<Mesh> g_boxMesh = nullptr;
	std::shared_ptr<Mesh> g_planeMesh = nullptr;
	std::unordered_map<std::tuple<float, float>, std::shared_ptr<Mesh>, TupleHash> g_capsuleMeshes;

	double getTimestamp()
	{
		auto now = std::chrono::high_resolution_clock::now();
		auto epoch = now.time_since_epoch();
		return std::chrono::duration<double>(epoch).count();
	}

	std::shared_ptr<Mesh> createCapsuleMesh(CommandList& commandList, float radius, float height, int tessellation, bool rhcoords)
	{
		auto it = g_capsuleMeshes.find({ radius, height });
		if (it != g_capsuleMeshes.end()) {
			return it->second;
		}
		std::shared_ptr<Mesh> sharedMesh = Mesh::CreateCapsule(commandList, radius * 2, height, tessellation, rhcoords);
		g_capsuleMeshes[{ radius, height }] = sharedMesh;
		return sharedMesh;
	}
}
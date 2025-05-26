#pragma once
#include <string>
#include <Mesh.h>
#include <Texture.h>
#include "Collider.h"

struct TupleHash {
	std::size_t operator()(const std::tuple<float, float>& key) const {
		auto [x, y] = key;
		std::size_t h1 = std::hash<float>{}(x);
		std::size_t h2 = std::hash<float>{}(y);
		return h1 ^ (h2 << 1); // Combine hashes
	}
};

namespace GlobalData {
	extern uint16_t g_clientId;
	extern uint16_t g_listenPort;
	extern USHORT g_broadcastPort;
	extern std::string g_clientName;
	extern DirectX::XMFLOAT4 g_clientColor;

	extern double g_renderingFPS;

	extern int g_physicsFreq;
	extern double g_physicsDt;

	extern int g_networkFreq;
	extern double g_networkDt;

	extern int g_tick;

	extern std::shared_ptr <Texture> g_customTexture;
	extern std::shared_ptr <Texture> g_defaultTexture;

	extern std::shared_ptr<Mesh> g_sphereMesh;
	extern std::shared_ptr <Mesh> g_boxMesh;
	extern std::shared_ptr <Mesh> g_planeMesh;
	extern std::unordered_map<std::tuple<float, float>, std::shared_ptr<Mesh>, TupleHash> g_capsuleMeshes;

	double getTimestamp();

	std::shared_ptr<Mesh> createCapsuleMesh(CommandList& commandList, float radius, float height, int tessellation, bool rhcoords = false);
}
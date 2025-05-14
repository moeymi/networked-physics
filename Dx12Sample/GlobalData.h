#pragma once
#include <string>
#include <Mesh.h>
#include <Texture.h>
#include "Collider.h"

namespace GlobalData {
	extern uint16_t g_clientId;
	extern uint16_t g_listenPort;
	extern USHORT g_broadcastPort;
	extern std::string g_clientName;
	extern DirectX::XMFLOAT4 g_clientColor;

	extern double g_renderingFPS;
	extern double g_physicsFPS;
	extern double g_networkFPS;

	extern double g_simulationTime;

	extern std::shared_ptr <Texture> g_customTexture;
	extern std::shared_ptr <Texture> g_defaultTexture;

	extern std::shared_ptr<Mesh> g_sphereMesh;
	extern std::shared_ptr <Mesh> g_boxMesh;
	extern std::shared_ptr <Mesh> g_planeMesh;

	double getTimestamp();
}
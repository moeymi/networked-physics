#pragma once
#include "pch.h"
#include <optional>
#include <functional>
#include "PhysicsObject.h"
namespace CollisionHandlers {
    std::optional<CollisionManifold> SphereVsSphere(PhysicsObject* a, PhysicsObject* b, const bool& flip = false);
	std::optional<CollisionManifold> SphereVsBox(PhysicsObject* a, PhysicsObject* b, const bool& flip = false);
	std::optional<CollisionManifold> BoxVsBox(PhysicsObject* a, PhysicsObject* b, const bool& flip = false);
	std::optional<CollisionManifold> SphereVsCapsule(PhysicsObject* a, PhysicsObject* b, const bool& flip = false);
}
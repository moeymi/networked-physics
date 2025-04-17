#include "SphereCollider.h"
#include "BoxCollider.h"
#include <optional>
#include <functional>
#include "PhysicsObject.h"

namespace CollisionHandlers {
    std::optional<CollisionManifold> SphereVsSphere(PhysicsObject* a, PhysicsObject* b);
	std::optional<CollisionManifold> SphereVsBox(PhysicsObject* a, PhysicsObject* b);
	std::optional<CollisionManifold> BoxVsBox(PhysicsObject* a, PhysicsObject* b);
}
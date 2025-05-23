#pragma once
#include "pch.h"
#include <memory>
#include <unordered_map>
#include <DirectXMath.h>
#include <functional>
#include "PhysicsObject.h"
#include "SphereCollider.h"
#include "CollisionHandlers.h"
class CollisionSystem {
private:
    using CollisionHandler = std::function<std::optional<CollisionManifold>(PhysicsObject*, PhysicsObject*, const bool&)>;

    CollisionHandler m_handlers[static_cast<size_t>(ColliderType::Count)][static_cast<size_t>(ColliderType::Count)];

public:
    void registerHandler(ColliderType a, ColliderType b, CollisionHandler handler);
    std::optional<CollisionManifold> checkCollision(PhysicsObject* a, PhysicsObject* b);
    void initializeCollisionSystem();

    CollisionSystem();
};

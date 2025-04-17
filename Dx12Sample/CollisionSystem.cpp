#include "CollisionSystem.h"

CollisionSystem::CollisionSystem()
{
	// Initialize the collision system with default handlers
	initializeCollisionSystem();
}

void CollisionSystem::registerHandler(ColliderType a, ColliderType b, CollisionHandler handler)
{
    m_handlers[static_cast<size_t>(a)][static_cast<size_t>(b)] = handler;
    m_handlers[static_cast<size_t>(b)][static_cast<size_t>(a)] = [handler](PhysicsObject* objB, PhysicsObject* objA) {
		return handler(objA, objB); // Swap parameters
	};
}

std::optional<CollisionManifold> CollisionSystem::checkCollision(PhysicsObject* a, PhysicsObject* b)
{
	return m_handlers[static_cast<size_t>(a->getCollider()->getType())][static_cast<size_t>(b->getCollider()->getType())](a, b);
}

void CollisionSystem::initializeCollisionSystem()
{
	registerHandler(ColliderType::Sphere, ColliderType::Sphere, CollisionHandlers::SphereVsSphere);
	registerHandler(ColliderType::Sphere, ColliderType::Box, CollisionHandlers::SphereVsBox);
	registerHandler(ColliderType::Box, ColliderType::Box, CollisionHandlers::BoxVsBox);
}
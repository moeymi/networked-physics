#pragma once
#include "CollisionSystem.h"
#include <shared_mutex>

class PhysicsEngine {
private:
    std::vector<std::shared_ptr<PhysicsObject>> m_bodies;
    CollisionSystem m_collisionSystem;

public:
    void onUpdate(float deltaTime);
    void addBody(std::shared_ptr<PhysicsObject> body);

private:
    void detectAndResolveCollisions();

    std::vector<std::pair<PhysicsObject*, PhysicsObject*>> broadPhase();

    void resolveCollisionVelocity(const CollisionManifold& manifold);
	void resolveCollisionPosition(const CollisionManifold& contact);
	DirectX::XMVECTOR computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal);

    void applyFriction(const ContactPoint& contact,
        PhysicsObject* a, PhysicsObject* b,
        const DirectX::XMVECTOR& rA,
        const DirectX::XMVECTOR& rB,
        const float& normalImpulse,
        const DirectX::XMVECTOR& preRelVel);
	void positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b);
};
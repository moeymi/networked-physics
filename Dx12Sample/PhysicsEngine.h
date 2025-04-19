#pragma once
#include "CollisionSystem.h"
#include <shared_mutex>

class PhysicsEngine {
private:
    std::vector<std::shared_ptr<PhysicsObject>> m_bodies;
    CollisionSystem m_collisionSystem;

    static constexpr int m_velocityIterations = 8;
    static constexpr int m_positionIterations = 4;
    static constexpr float m_kRestitutionThreshold = 1.0f;  // only bounce if closing speed > this

    static constexpr float m_kPenetrationSlop = 0.001f;  // 1 mm of allowed overlap
    static constexpr float m_kBaumgarte = 0.2f;    // positional stabilization factor

public:
    void onUpdate(float deltaTime);
    void addBody(std::shared_ptr<PhysicsObject> body);

private:
    void detectAndResolveCollisions(const float& dt);

    std::vector<std::pair<PhysicsObject*, PhysicsObject*>> broadPhase();

    void prestepCollisionManifolds(std::vector<CollisionManifold>& collisionManifolds, const float& dt);
    void resolveCollisionVelocity(CollisionManifold& manifold);

	DirectX::XMVECTOR computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal);
	void positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b);
};
#pragma once
#include "pch.h"
#include "CollisionSystem.h"
#include "ThreadedSystem.h"
#include <shared_mutex>

class PhysicsEngine : public ThreadedSystem {
private:
    std::vector<std::shared_ptr<PhysicsObject>> m_bodies;
    CollisionSystem m_collisionSystem;

    std::map <std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold> m_contactManifolds;

    static constexpr int m_velocityIterations = 8;
    static constexpr int m_positionIterations = 4;
    static constexpr float m_kRestitutionThreshold = .5f;  // only bounce if closing speed > this

    static constexpr float m_kPenetrationSlop = 0.001f;  // 1 mm of allowed overlap
    static constexpr float m_kBaumgarte = 0.2f;    // positional stabilization factor

	static float m_gravity;
	static bool m_gravityEnabled;

public:
	PhysicsEngine() = default;
    void addBody(std::shared_ptr<PhysicsObject> body);
    void clearBodies();

    void toggleGravity(const bool& toggle);
    void setGravity(const float& gravity);

    float getGravity() const;
    bool isGravityEnabled() const;

private:
    virtual void onUpdate(float deltaTime) override;

    std::vector<std::pair<PhysicsObject*, PhysicsObject*>> broadPhase();

    void detectAndResolveCollisions(const float& dt);
    void prestepCollisionManifolds(std::map<std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold>& contactManifolds, const float& dt);
    void resolveCollisionVelocity(CollisionManifold& manifold, const int& iteration);
    void positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b);
    void matchAndTransferImpulses(CollisionManifold& newManifold, const CollisionManifold& oldManifold);

    DirectX::XMVECTOR computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal);

    // Helper to create a canonical key for a pair of objects
    std::pair<PhysicsObject*, PhysicsObject*> makePairKey(PhysicsObject* objA, PhysicsObject* objB);

};
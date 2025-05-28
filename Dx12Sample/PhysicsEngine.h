#pragma once
#include "pch.h"
#include "CollisionSystem.h"
#include "ThreadedSystem.h"
#include "NetworkedObject.h"
#include "RingBufferSPSC.h"
#include "SpatialHashGrid.h"
#include "ThreadPool.h"

#include <shared_mutex>

class PhysicsEngine : public ThreadedSystem {
private:
    static std::unordered_map<uint16_t, std::shared_ptr<NetworkedObject>> m_ownedNetworkedObjects;
	static std::unordered_map<uint16_t, std::shared_ptr<NetworkedObject>> m_nonOwnedNetworkedObjects;
    static std::vector<std::shared_ptr<PhysicsObject>> m_ownedBodies;
	static std::vector<std::shared_ptr<PhysicsObject>> m_nonOwnedBodies;
	static std::vector<PhysicsObject*> m_bodies;
    static SpatialHashGrid m_spatialGrid;
    static ThreadPool s_pool;

    static CollisionSystem m_collisionSystem;

    std::map <PairKey, CollisionManifold> m_contactManifolds;

    static constexpr int m_velocityIterations = 8;
    static constexpr int m_positionIterations = 4;
    static constexpr float m_kRestitutionThreshold = 1.0f;
	static constexpr int m_kDelayTicks = 2;

    static constexpr float m_kPenetrationSlop = 0.001f;
    static constexpr float m_kBaumgarte = 0.2f;
    static constexpr float m_kTangentEpsSq = 1e-8f;
	static constexpr std::size_t m_kChunkSize = 64;

	static float m_gravity;

	static float m_simulationDeltaTime;

	static std::mutex m_ghostModeMutex;
    static std::atomic<bool> m_ghostMode;
	static float m_simTimeAccumulator;
	static float m_nextTickTime;

	RingBufferSPSC<Snapshot, 512>* m_outgoingBuffer;
	RingBufferSPSC<Snapshot, 512>* m_incomingBuffer;

public:
    PhysicsEngine(RingBufferSPSC<Snapshot, 512>* outgoingBuf,
        RingBufferSPSC<Snapshot, 512>* incomingBuf);

    void addOwnedBody(const std::shared_ptr<PhysicsObject>& body);
	void addBody(const std::shared_ptr<NetworkedObject>& object);
    void clearBodies();

    static void setGravity(const float& gravity);
    static float getGravity();

    static bool ghostModeEnabled();
    static void setGhostMode(bool enabled);

    void setSimulationDeltaTime(const float& deltaTime);
    float getSimulationDeltaTime() const;

private:
    virtual void onUpdate(float deltaTime) override;
	virtual void onStop() noexcept override;
    virtual void onStart() noexcept override;

    std::vector<std::pair<PhysicsObject*, PhysicsObject*>> broadPhase();

    void detectAndResolveCollisions(const float& dt);
    void prestepCollisionManifolds(std::map<PairKey, CollisionManifold>& contactManifolds, const float& dt);
    void resolveCollisionVelocity(CollisionManifold& manifold, const int& iteration);
    void positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b);

    DirectX::XMVECTOR computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal);

    PairKey makePairKey(PhysicsObject* objA, PhysicsObject* objB);

    bool canModify(NetworkedObject* object) const;

};
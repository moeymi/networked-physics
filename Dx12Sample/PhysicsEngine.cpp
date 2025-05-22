#include "PhysicsEngine.h"
#include "GlobalData.h"
#include <algorithm>
#include <string>
#include <omp.h>

float PhysicsEngine::m_gravity = 9.81f;
bool PhysicsEngine::m_gravityEnabled = true;
float PhysicsEngine::m_simulationDeltaTime;
std::vector<std::shared_ptr<PhysicsObject>> PhysicsEngine::m_bodies;
CollisionSystem PhysicsEngine::m_collisionSystem;

void PhysicsEngine::onUpdate(float) {
    for (const auto& body : m_bodies) {
        if (!body->isStatic()) {
            body->onUpdate(m_simulationDeltaTime);
        }
    }

    detectAndResolveCollisions(m_simulationDeltaTime);
    for (const auto& body : m_bodies) {
        body->swapStates();
    }
}

void PhysicsEngine::addBody(std::shared_ptr<PhysicsObject> body) {
    if (m_running) {
		throw std::runtime_error("Cannot add bodies while the physics engine is running.");
    }
	m_bodies.push_back(body);
	if (m_gravityEnabled) {
		body->applyConstantForce({ 0.0f, -m_gravity * body->getMass(), 0.0f, 0.0f });
	}
}

void PhysicsEngine::clearBodies() {
	if (m_running) {
		throw std::runtime_error("Cannot clear bodies while the physics engine is running.");
	}
	m_bodies.clear();
	m_contactManifolds.clear();
}

void PhysicsEngine::setGravity(const float& gravity) {
	m_gravity = gravity;
	if (!m_gravityEnabled) return;
	for (auto body : m_bodies) {
        body->resetConstantForces();
		body->applyConstantForce({ 0.0f, -m_gravity * body->getMass(), 0.0f, 0.0f });
	}
}

float PhysicsEngine::getGravity() {
	return m_gravity;
}

bool PhysicsEngine::isGravityEnabled() const {
	return m_gravityEnabled;
}

void PhysicsEngine::setSimulationDeltaTime(const float& deltaTime) {
	m_simulationDeltaTime = deltaTime;
}

float PhysicsEngine::getSimulationDeltaTime() const {
	return m_simulationDeltaTime;
}

void PhysicsEngine::detectAndResolveCollisions(const float& deltaTime) {
    auto candidates = broadPhase();

    std::map<std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold> currentFrameManifolds;

    //#pragma omp parallel for
    for (int i = 0; i < candidates.size(); i++) {
        auto pair = candidates[i];
        PhysicsObject* objA = pair.first;
        PhysicsObject* objB = pair.second;

        if (objA->isStatic() && objB->isStatic()) continue;

        if (auto newManifoldOpt = m_collisionSystem.checkCollision(objA, objB)) {
            CollisionManifold newManifold = *newManifoldOpt;
            auto pairKey = makePairKey(objA, objB);

            auto it = m_contactManifolds.find(pairKey);

            if (it != m_contactManifolds.end()) {
                CollisionManifold& oldManifold = it->second;
                matchAndTransferImpulses(newManifold, oldManifold);
            }
            currentFrameManifolds[pairKey] = std::move(newManifold);
        }
    }
    m_contactManifolds = std::move(currentFrameManifolds);

    prestepCollisionManifolds(m_contactManifolds, deltaTime);

    for (int iter = 0; iter < m_velocityIterations; ++iter) {
        for (int i = 0; i < m_contactManifolds.size(); i++) {
			auto pair = m_contactManifolds.begin();
			std::advance(pair, i);
            auto& manifold = pair->second;
            if (manifold.contacts.empty()) continue;
            resolveCollisionVelocity(manifold, iter);
        }
    }

    for (int iter = 0; iter < m_positionIterations; ++iter) {
		for (auto& [key, manifold] : m_contactManifolds) {
            if (manifold.contacts.empty()) continue;
            for (auto& contact : manifold.contacts) {
                positionalCorrection(contact, manifold.objectA, manifold.objectB);
            }
        }
    }
}

std::vector<std::pair<PhysicsObject*, PhysicsObject*>> PhysicsEngine::broadPhase() {
    std::vector<std::pair<PhysicsObject*, PhysicsObject*>> pairs;
    for (size_t i = 0; i < m_bodies.size(); ++i) {
        for (size_t j = i + 1; j < m_bodies.size(); ++j) {
            auto colliderA = m_bodies[i]->getCollider();
            auto colliderB = m_bodies[j]->getCollider();
            if (colliderA && colliderB) {
                pairs.emplace_back(m_bodies[i].get(), m_bodies[j].get());
            }
        }
    }
    return pairs;
}

void PhysicsEngine::prestepCollisionManifolds(std::map<std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold>& contactManifolds, const float& dt) {
    using namespace DirectX;

    for (auto& pair : contactManifolds) {
        auto& m = pair.second;

        PhysicsObject* A = m.objectA;
        PhysicsObject* B = m.objectB;

        if (m.contacts.empty() || (A->isStatic() && B->isStatic())) continue;

        float invMassA = A->isStatic() ? 0.0f : 1.0f / A->getMass();
        float invMassB = B->isStatic() ? 0.0f : 1.0f / B->getMass();
        XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor(0);
        XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor(0);

        XMVECTOR comA = A->getTransform().GetPosition(0);
        XMVECTOR comB = B->getTransform().GetPosition(0);

        for (auto& c : m.contacts) {
            XMVECTOR rA = XMVectorSubtract(c.position, comA);
            XMVECTOR rB = XMVectorSubtract(c.position, comB);
            XMVECTOR n = c.normal;

            XMVECTOR raCrossN = XMVector3Cross(rA, n);
            XMVECTOR Ia_raCrossN = XMVector3Transform(raCrossN, invInertiaA);
            XMVECTOR crossTermA = XMVector3Cross(Ia_raCrossN, rA);
            float angularA = XMVectorGetX(XMVector3Dot(crossTermA, n));

            XMVECTOR rbCrossN = XMVector3Cross(rB, n);
            XMVECTOR Ib_rbCrossN = XMVector3Transform(rbCrossN, invInertiaB);
            XMVECTOR crossTermB = XMVector3Cross(Ib_rbCrossN, rB);
            float angularB = XMVectorGetX(XMVector3Dot(crossTermB, n));

            float invMassSum = invMassA + invMassB + angularA + angularB;
            c.normalMass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;

            float Ia_n = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(n, invInertiaA), n));
            float Ib_n = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(n, invInertiaB), n));

            float denom = Ia_n + Ib_n;
            c.angularMass = (denom > 0.0f) ? 1.0f / denom : 0.0f;

            XMVECTOR velA = XMVectorAdd(A->getVelocity(0), XMVector3Cross(A->getAngularVelocity(0), rA));
            XMVECTOR velB = XMVectorAdd(B->getVelocity(0),
                XMVector3Cross(B->getAngularVelocity(0), rB));
            XMVECTOR relVel = XMVectorSubtract(velB, velA);

            XMVECTOR t = computeTangent(relVel, n);
            c.tangent = t;

            XMVECTOR raCrossT = XMVector3Cross(rA, t);
            XMVECTOR rbCrossT = XMVector3Cross(rB, t);

            XMVECTOR Ia_raCrossT = XMVector3Transform(raCrossT, invInertiaA);
            XMVECTOR crossTA = XMVector3Cross(Ia_raCrossT, rA);
            float angularTA = XMVectorGetX(XMVector3Dot(crossTA, t));


			XMVECTOR Ib_rbCrossT = XMVector3Transform(rbCrossT, invInertiaB);
			XMVECTOR crossTB = XMVector3Cross(Ib_rbCrossT, rB);
			float angularTB = XMVectorGetX(XMVector3Dot(crossTB, t));

            float invMassT = invMassA + invMassB + angularTA + angularTB;
            c.tangentMass = invMassT > 0.0f ? 1.0f / invMassT : 0.0f;

            float penetrationErr = max(c.penetration - m_kPenetrationSlop, 0.0f);
            float baumgarteBias = (m_kBaumgarte / dt) * penetrationErr;

            float vn = XMVectorGetX(XMVector3Dot(relVel, n));        // closing speed
            float restitutionBias = 0.0f;

            const PhysicsMaterial& matA = A->getPhysicsMaterial();
            const PhysicsMaterial& matB = B->getPhysicsMaterial();
            float combinedRestitution = 0.5f * std::abs(matA.restitution + matB.restitution);

            if (vn < -m_kRestitutionThreshold)              // *fast* incoming impact
            {
                // apply restitution only
                restitutionBias = -combinedRestitution * vn;
                c.velocityBias = restitutionBias;
            }
            else                                             // slow / resting contact
            {
                // apply Baumgarte only
                c.velocityBias = baumgarteBias;
            }

            // Optional safety clamp so bias can’t exceed 0.2 m/s
            c.velocityBias = min(c.velocityBias, 0.20f / dt);
        }
    }
}

void PhysicsEngine::resolveCollisionVelocity(
    CollisionManifold& manifold,
    const int& iteration)
{
    using namespace DirectX;

    PhysicsObject* A = manifold.objectA;
    PhysicsObject* B = manifold.objectB;
    if (A->isStatic() && B->isStatic()) return;

    const PhysicsMaterial& matA = A->getPhysicsMaterial();
    const PhysicsMaterial& matB = B->getPhysicsMaterial();

    const float combinedFriction = std::sqrt(matA.friction * matB.friction);
    const float combinedAngularFriction = std::sqrt(matA.angularFriction * matB.angularFriction);

    const XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor(0);
    const XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor(0);

    const XMVECTOR comA = A->getTransform().GetPosition(0);
    const XMVECTOR comB = B->getTransform().GetPosition(0);

    for (auto& c : manifold.contacts)
    {
        XMVECTOR rA = c.position - comA;
        XMVECTOR rB = c.position - comB;
        XMVECTOR n = c.normal;

        XMVECTOR vA = A->getVelocity(iteration != 0);
        XMVECTOR wA = A->getAngularVelocity(iteration != 0);
        XMVECTOR vB = B->getVelocity(iteration != 0);
        XMVECTOR wB = B->getAngularVelocity(iteration != 0);

        XMVECTOR velA_contact = vA + XMVector3Cross(wA, rA);
        XMVECTOR velB_contact = vB + XMVector3Cross(wB, rB);
        XMVECTOR relVelLinear = velB_contact - velA_contact;
        XMVECTOR relVelAngular = wB - wA;

        float relVelNormal = XMVectorGetX(XMVector3Dot(relVelLinear, n));

        float dVN = -relVelNormal + c.velocityBias;   // bias is constant
        if (dVN < 0.0f) dVN = 0.0f;

        float lambdaN = dVN * c.normalMass;

        float oldImpN = c.accumulatedNormalImpulse;
        float newImpN = max(oldImpN + lambdaN, 0.0f);
        float actualN = newImpN - oldImpN;
        c.accumulatedNormalImpulse = newImpN;

        XMVECTOR Pn = n * actualN;

        if (!A->isStatic()) A->applyImpulseAtPosition(-Pn, c.position);
        if (!B->isStatic()) B->applyImpulseAtPosition(Pn, c.position);

        vA = A->getVelocity(iteration != 0);
        wA = A->getAngularVelocity(iteration != 0);
        vB = B->getVelocity(iteration != 0);
        wB = B->getAngularVelocity(iteration != 0);

        velA_contact = vA + XMVector3Cross(wA, rA);
        velB_contact = vB + XMVector3Cross(wB, rB);
        relVelLinear = velB_contact - velA_contact;

        XMVECTOR t = c.tangent;
        float relVelTangent = XMVectorGetX(XMVector3Dot(relVelLinear, t));

        float lambdaT = -relVelTangent * c.tangentMass;
        float maxT = combinedFriction * c.accumulatedNormalImpulse;

        float oldAccumT = c.accumulatedFrictionImpulse;
        c.accumulatedFrictionImpulse =
            std::clamp(oldAccumT + lambdaT, -maxT, maxT);
        float actualLambdaT = c.accumulatedFrictionImpulse - oldAccumT;

        XMVECTOR Pt = t * actualLambdaT;
        if (!A->isStatic()) A->applyImpulseAtPosition(-Pt, c.position);
        if (!B->isStatic()) B->applyImpulseAtPosition(Pt, c.position);

        if (c.angularMass > 1e-9f)
        {
            relVelAngular = B->getAngularVelocity(iteration != 0) -
                A->getAngularVelocity(iteration != 0);

            float relOmegaN = XMVectorGetX(XMVector3Dot(relVelAngular, n));
            float lambdaA = -relOmegaN * c.angularMass;

            float maxA = combinedAngularFriction * c.accumulatedNormalImpulse;

            float oldAccumA = c.accumulatedAngularFrictionImpulse;
            c.accumulatedAngularFrictionImpulse =
                std::clamp(oldAccumA + lambdaA, -maxA, maxA);
            float actualLambdaA =
                c.accumulatedAngularFrictionImpulse - oldAccumA;

            XMVECTOR Pa = n * actualLambdaA;
            if (!A->isStatic()) A->applyAngularImpulse(-Pa);
            if (!B->isStatic()) B->applyAngularImpulse(Pa);
        }
    }
}

DirectX::XMVECTOR PhysicsEngine::computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal) {
    using namespace DirectX;

    XMVECTOR t = XMVectorSubtract(relVel,
        XMVectorScale(normal, XMVectorGetX(XMVector3Dot(relVel, normal))));

    if (XMVectorGetX(XMVector3LengthSq(t)) < kTangentEpsSq)
    {
        float nx = fabsf(XMVectorGetX(normal));
        float ny = fabsf(XMVectorGetY(normal));
        float nz = fabsf(XMVectorGetZ(normal));

        XMVECTOR axis = (nx < 0.57735f) ? XMVectorSet(1, 0, 0, 0) :
            (ny < 0.57735f) ? XMVectorSet(0, 1, 0, 0) :
            XMVectorSet(0, 0, 1, 0);

        t = XMVector3Cross(normal, axis);
    }

    return XMVector3Normalize(t);
}

void PhysicsEngine::positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b) {
    using namespace DirectX;

    if (a->isStatic() && b->isStatic()) return;

    const float percent = .2f;
    const float slop = 0.001f;

    const float penetrationDepth = contact.penetration - slop;
    if (penetrationDepth <= 0.0f) return;

    const float invMassA = a->isStatic() ? 0.0f : 1.0f / a->getMass();
    const float invMassB = b->isStatic() ? 0.0f : 1.0f / b->getMass();
    const float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    const XMVECTOR correction = XMVectorScale(
        contact.normal,
        (penetrationDepth * percent) / totalInvMass
    );

    if (!a->isStatic()) {
        a->getTransform().Translate(XMVectorScale(correction, -invMassA), 1);
    }
    if (!b->isStatic()) {
        b->getTransform().Translate(XMVectorScale(correction, invMassB), 1);
    }
}

void PhysicsEngine::matchAndTransferImpulses(CollisionManifold& newManifold, const CollisionManifold& oldManifold) {
    using namespace DirectX;
    const float matchDistanceThresholdSq = 0.01f * 0.01f; // Example threshold

    for (auto& newContact : newManifold.contacts) {
        float closestDistSq = 1e10f;
        const ContactPoint* bestMatch = nullptr;

        for (const auto& oldContact : oldManifold.contacts) {
            // Match based on world-space position proximity (simplistic)
            float distSq = XMVectorGetX(XMVector3LengthSq(XMVectorSubtract(newContact.position, oldContact.position)));
            if (distSq < closestDistSq && distSq < matchDistanceThresholdSq) {
                closestDistSq = distSq;
                bestMatch = &oldContact;
            }
        }

        if (bestMatch) {
            // Transfer impulses from the matched old contact
            newContact.accumulatedNormalImpulse = bestMatch->accumulatedNormalImpulse;
            newContact.accumulatedFrictionImpulse = bestMatch->accumulatedFrictionImpulse;
            newContact.accumulatedAngularFrictionImpulse = bestMatch->accumulatedAngularFrictionImpulse; // NEW
        }
    }
}

std::pair<PhysicsObject*, PhysicsObject*> PhysicsEngine::makePairKey(PhysicsObject* objA, PhysicsObject* objB) {
    return (objA < objB) ? std::make_pair(objA, objB) : std::make_pair(objB, objA);
}

void PhysicsEngine::onStart() {
	if (m_simulationDeltaTime <= 0.0f) {
        m_simulationDeltaTime = m_fixedTimeStep;
	}
}

void PhysicsEngine::onStop() {}
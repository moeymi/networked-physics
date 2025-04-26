#include "PhysicsEngine.h"
#include <algorithm>
#include <string>
#include <omp.h>

float PhysicsEngine::m_gravity = 9.81f;
bool PhysicsEngine::m_gravityEnabled = true;

void PhysicsEngine::onUpdate(float deltaTime) {
	for (auto body : m_bodies) {
        body->swapStates();
	}

    #pragma omp parallel for
    for (int i = 0; i < m_bodies.size(); i++) {
        auto body = m_bodies[i].get();
        if (!body->isStatic()) {
            body->onUpdate(deltaTime);
        }
    }

    detectAndResolveCollisions(deltaTime);
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

void PhysicsEngine::setGravity(const float& gravity) {
	m_gravity = gravity;
	if (!m_gravityEnabled) return;
	for (auto body : m_bodies) {
        body->resetConstantForces();
		body->applyConstantForce({ 0.0f, -m_gravity * body->getMass(), 0.0f, 0.0f });
	}
}

float PhysicsEngine::getGravity() const {
	return m_gravity;
}

void PhysicsEngine::toggleGravity(const bool& toggle) {
	if (m_gravityEnabled == toggle) return;
	m_gravityEnabled = toggle;
	if (toggle) {
		setGravity(m_gravity);
	}
	else {
		for (auto& body : m_bodies) {
			body->resetConstantForces();
		}
	}
}

bool PhysicsEngine::isGravityEnabled() const {
	return m_gravityEnabled;
}

void PhysicsEngine::detectAndResolveCollisions(const float& deltaTime) {
    auto candidates = broadPhase();

    std::map<std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold> currentFrameManifolds;

    std::vector<CollisionManifold> collisions(candidates.size());

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
                
                if (!newManifold.contacts.empty() && !oldManifold.contacts.empty()) {
                    newManifold.contacts[0].accumulatedNormalImpulse = oldManifold.contacts[0].accumulatedNormalImpulse;
                    newManifold.contacts[0].accumulatedFrictionImpulse = oldManifold.contacts[0].accumulatedFrictionImpulse;
                    newManifold.contacts[0].accumulatedAngularFrictionImpulse = oldManifold.contacts[0].accumulatedAngularFrictionImpulse;
                }
                
            }
            currentFrameManifolds[pairKey] = std::move(newManifold);
        }
    }
    m_contactManifolds = std::move(currentFrameManifolds);

    prestepCollisionManifolds(m_contactManifolds, deltaTime);

    for (int iter = 0; iter < m_velocityIterations; ++iter) {

        #pragma omp parallel for
        for (int i = 0; i < m_contactManifolds.size(); i++) {
			auto pair = m_contactManifolds.begin();
			std::advance(pair, i);
            auto& manifold = pair->second;
            if (manifold.contacts.empty()) continue;
            resolveCollisionVelocity(manifold, iter);
        }
    }

    for (int i = 0; i < m_positionIterations; ++i) {
        #pragma omp parallel for
		for (int i = 0; i < m_contactManifolds.size(); i++) {
            // Output omp thread id
			auto pair = m_contactManifolds.begin();
			std::advance(pair, i);
            auto& manifold = pair->second;
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
            XMVECTOR rbCrossN = XMVector3Cross(rB, n);
            float angularA = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(raCrossN, invInertiaA), raCrossN));
            float angularB = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(rbCrossN, invInertiaB), rbCrossN));
            float invMassSum = invMassA + invMassB + angularA + angularB;
            c.normalMass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;

            XMVECTOR velA = XMVectorAdd(A->getVelocity(0), XMVector3Cross(A->getAngularVelocity(0), rA));
            XMVECTOR velB = XMVectorAdd(B->getVelocity(0),
                XMVector3Cross(B->getAngularVelocity(0), rB));
            XMVECTOR relVel = XMVectorSubtract(velB, velA);

            XMVECTOR t = computeTangent(relVel, n);
            c.tangent = t;

            XMVECTOR raCrossT = XMVector3Cross(rA, t);
            XMVECTOR rbCrossT = XMVector3Cross(rB, t);
            float angularTA = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(raCrossT, invInertiaA), raCrossT));
            float angularTB = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(rbCrossT, invInertiaB), rbCrossT));
            float invMassT = invMassA + invMassB + angularTA + angularTB;
            c.tangentMass = invMassT > 0.0f ? 1.0f / invMassT : 0.0f;

            float penetrationErr = max(c.penetration - m_kPenetrationSlop, 0.0f);
            c.bias = (m_kBaumgarte / dt) * penetrationErr;
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

    if (A->isStatic() && B->isStatic()) {
        return;
    }

    const PhysicsMaterial& matA = A->getMaterial();
    const PhysicsMaterial& matB = B->getMaterial();

    const float combinedFriction = std::sqrt(matA.friction * matB.friction);
	const float combinedAngularFriction = std::sqrt(matA.angularFriction * matB.angularFriction);
    const float combinedRestitution = abs((matA.restitution + matB.restitution) / 2.0f);

    const float invMassA = A->isStatic() ? 0.0f : 1.0f / A->getMass();
    const float invMassB = B->isStatic() ? 0.0f : 1.0f / B->getMass();
    const XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor(0);
    const XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor(0);

    const XMVECTOR comA = A->getTransform().GetPosition(0);
    const XMVECTOR comB = B->getTransform().GetPosition(0);

    for (auto& c : manifold.contacts) {
        XMVECTOR rA = XMVectorSubtract(c.position, comA);
        XMVECTOR rB = XMVectorSubtract(c.position, comB);

        XMVECTOR vA = A->getVelocity(iteration != 0);
        XMVECTOR wA = A->getAngularVelocity(iteration != 0);
        XMVECTOR vB = B->getVelocity(iteration != 0);
        XMVECTOR wB = B->getAngularVelocity(iteration != 0);

        XMVECTOR velA_contact = XMVectorAdd(vA, XMVector3Cross(wA, rA));
        XMVECTOR velB_contact = XMVectorAdd(vB, XMVector3Cross(wB, rB));
        XMVECTOR relVelLinear = XMVectorSubtract(velB_contact, velA_contact);

        XMVECTOR relVelAngular = XMVectorSubtract(wB, wA);

        XMVECTOR n = c.normal;
        float relVelNormal = XMVectorGetX(XMVector3Dot(relVelLinear, n));

		if (relVelNormal > 0.0f) {
			continue;
		}

        float velocityBias = c.bias;
        float restitutionTerm = 0.0f;
        if (relVelNormal < -1.0f) {
            restitutionTerm = -combinedRestitution * relVelNormal;
        }

        float deltaVelNormalTarget = -relVelNormal + restitutionTerm;
        float lambdaN = deltaVelNormalTarget * c.normalMass;

        float oldAccumulatedNormalImpulse = c.accumulatedNormalImpulse;
        c.accumulatedNormalImpulse = max(oldAccumulatedNormalImpulse + lambdaN, 0.0f);
        float actualLambdaN = c.accumulatedNormalImpulse - oldAccumulatedNormalImpulse;

        XMVECTOR Pn_step = XMVectorScale(n, actualLambdaN);
        if (!A->isStatic()) A->applyImpulseAtPosition(XMVectorNegate(Pn_step), c.position);
        if (!B->isStatic()) B->applyImpulseAtPosition(Pn_step, c.position);

        vA = A->getVelocity(iteration != 0); wA = A->getAngularVelocity(iteration != 0);
        vB = B->getVelocity(iteration != 0); wB = B->getAngularVelocity(iteration != 0);
        velA_contact = XMVectorAdd(vA, XMVector3Cross(wA, rA));
        velB_contact = XMVectorAdd(vB, XMVector3Cross(wB, rB));
        relVelLinear = XMVectorSubtract(velB_contact, velA_contact);
        relVelAngular = XMVectorSubtract(wB, wA);


        // --- Linear Friction Impulse ---
        XMVECTOR t = c.tangent;
        if (c.tangentMass > 1e-9f) {
            float relVelTangent = XMVectorGetX(XMVector3Dot(relVelLinear, t));

            // Calculate impulse magnitude change needed (lambda_t) to stop tangent motion
            float lambdaT = -relVelTangent * c.tangentMass;

            float maxFriction = combinedFriction * c.accumulatedNormalImpulse;
            float oldAccumulatedFrictionImpulse = c.accumulatedFrictionImpulse;
            c.accumulatedFrictionImpulse = std::clamp(oldAccumulatedFrictionImpulse + lambdaT, -maxFriction, maxFriction);
            float actualLambdaT = c.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse;

            // Apply linear friction impulse for this step
            XMVECTOR Pt_step = XMVectorScale(t, actualLambdaT);
            if (!A->isStatic()) A->applyImpulseAtPosition(XMVectorNegate(Pt_step), c.position);
            if (!B->isStatic()) B->applyImpulseAtPosition(Pt_step, c.position);
        }


        if (c.angularMass > 1e-9f) {
            float relOmegaNormal = XMVectorGetX(XMVector3Dot(relVelAngular, n));

            float lambdaA = -relOmegaNormal * c.angularMass;

            float maxAngularFriction = combinedAngularFriction * c.accumulatedNormalImpulse;
            float oldAccumulatedAngularImpulse = c.accumulatedAngularFrictionImpulse;
            c.accumulatedAngularFrictionImpulse = std::clamp(oldAccumulatedAngularImpulse + lambdaA, -maxAngularFriction, maxAngularFriction);
            float actualLambdaA = c.accumulatedAngularFrictionImpulse - oldAccumulatedAngularImpulse;

            XMVECTOR Pa_step = XMVectorScale(n, actualLambdaA);
            if (!A->isStatic()) A->applyAngularImpulse(XMVectorNegate(Pa_step));
            if (!B->isStatic()) B->applyAngularImpulse(Pa_step);
        }
    }
}

DirectX::XMVECTOR PhysicsEngine::computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal) {
    DirectX::XMVECTOR tangent = DirectX::XMVectorSubtract(relVel, DirectX::XMVectorScale(normal, DirectX::XMVectorGetX(DirectX::XMVector3Dot(relVel, normal))));
    if (DirectX::XMVector3NearEqual(tangent, DirectX::XMVectorZero(), DirectX::XMVectorSplatEpsilon())) return tangent;
    return DirectX::XMVector3Normalize(tangent);
}

void PhysicsEngine::positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b) {
    using namespace DirectX;

    if (a->isStatic() && b->isStatic()) return;

    const float percent = .15f;
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
#include "PhysicsEngine.h"
#include <algorithm>
#include <string>
#include <omp.h>

void PhysicsEngine::onUpdate(float deltaTime) {
	for (auto body : m_bodies) {
        body->swapStates();
	}
    for (int i = 0; i < m_bodies.size(); i++) {
        auto body = m_bodies[i].get();
        if (!body->isStatic()) {
            body->onUpdate(deltaTime);
        }
    }

    detectAndResolveCollisions(deltaTime);
}

void PhysicsEngine::addBody(std::shared_ptr<PhysicsObject> body) {
	m_bodies.push_back(body);
}

void PhysicsEngine::detectAndResolveCollisions(const float& deltaTime) {
    auto candidates = broadPhase();

    std::map<std::pair<PhysicsObject*, PhysicsObject*>, CollisionManifold> currentFrameManifolds;

    std::vector<CollisionManifold> collisions(candidates.size());

    // --- NARROW PHASE ---
    for (int i = 0; i < candidates.size(); i++) {
        auto pair = candidates[i];
        PhysicsObject* objA = pair.first;
        PhysicsObject* objB = pair.second;

        // Skip pairs of static objects early if desired
        // if (objA->isStatic() && objB->isStatic()) continue;

        if (auto newManifoldOpt = m_collisionSystem.checkCollision(objA, objB)) { // Assuming checkCollision returns std::optional<CollisionManifold>
            CollisionManifold newManifold = *newManifoldOpt; // Get the manifold value
            auto pairKey = makePairKey(objA, objB);

            auto it = m_contactManifolds.find(pairKey);

            if (it != m_contactManifolds.end()) {
                // Manifold existed last frame, attempt to transfer accumulated impulses (Warm Starting)
                CollisionManifold& oldManifold = it->second;
                matchAndTransferImpulses(newManifold, oldManifold); // Use a proper matching function

                // *** Example Simple Transfer (replace with matchAndTransferImpulses) ***
                
                if (!newManifold.contacts.empty() && !oldManifold.contacts.empty()) {
                    // WARNING: This assumes only one contact and it corresponds.
                    // A real implementation needs contact matching based on position/features.
                    newManifold.contacts[0].accumulatedNormalImpulse = oldManifold.contacts[0].accumulatedNormalImpulse;
                    newManifold.contacts[0].accumulatedFrictionImpulse = oldManifold.contacts[0].accumulatedFrictionImpulse;
                    // --> TRANSFER ANGULAR IMPULSE <--
                    newManifold.contacts[0].accumulatedAngularFrictionImpulse = oldManifold.contacts[0].accumulatedAngularFrictionImpulse;
                }
                
            }
            // Store the new manifold (with potentially transferred impulses) for this frame
            currentFrameManifolds[pairKey] = std::move(newManifold); // Move constructed manifold
        }
    }

    // Update the persistent manifolds map for the next frame
    m_contactManifolds = std::move(currentFrameManifolds);

    // --- PRE-STEP ---
    // Compute each contact's effective masses,bias, and warm-start impulses
    // Pass the persistent map to the prestep function
    prestepCollisionManifolds(m_contactManifolds, deltaTime);

    // ITERATIVE VELOCITY SOLVE
    for (int iter = 0; iter < m_velocityIterations; ++iter) {
        // Iterate over the manifolds in the persistent map
        for (auto& pair : m_contactManifolds) {
            auto& manifold = pair.second; // Get the manifold by reference
            if (manifold.contacts.empty()) continue;
            // This now runs one "Gauss–Seidel" pass over all contacts:
            resolveCollisionVelocity(manifold, iter);
        }
    }

    // ITERATIVE POSITION SOLVE (split-impulse style, e.g. 4 passes):
    for (int i = 0; i < m_positionIterations; ++i) {
        // Iterate over the manifolds in the persistent map
        for (auto& pair : m_contactManifolds) {
            auto& manifold = pair.second; // Get the manifold by reference
            if (manifold.contacts.empty()) continue;
            // Positional correction is usually applied per contact point
            for (auto& contact : manifold.contacts) {
                // Pass the individual contact and objects
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

            XMVECTOR Pn = XMVectorScale(n, c.accumulatedNormalImpulse);
            XMVECTOR Pt = XMVectorScale(c.tangent, c.accumulatedFrictionImpulse);
            XMVECTOR P = XMVectorAdd(Pn, Pt);

            //A->applyImpulseAtPosition(XMVectorNegate(P), c.position);
            //B->applyImpulseAtPosition(P, c.position);
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

    // If both objects are static, no velocity resolution needed
    if (A->isStatic() && B->isStatic()) {
        return;
    }

    const PhysicsMaterial& matA = A->getMaterial();
    const PhysicsMaterial& matB = B->getMaterial();

    // Combine material properties (example: average friction, max restitution)
    // Geometric mean is often preferred for friction: sqrt(fA * fB)
    // Max is often preferred for restitution: max(eA, eB)
    const float combinedFriction = std::sqrt(matA.friction * matB.friction);
	const float combinedAngularFriction = std::sqrt(matA.angularFriction * matB.angularFriction);
    const float combinedRestitution = abs((matA.restitution + matB.restitution) / 2.0f);

    const float invMassA = A->isStatic() ? 0.0f : 1.0f / A->getMass();
    const float invMassB = B->isStatic() ? 0.0f : 1.0f / B->getMass();
    const XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor(0);
    const XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor(0);

    const XMVECTOR comA = A->getTransform().GetPosition(0);
    const XMVECTOR comB = B->getTransform().GetPosition(0);

    // Iterate through each contact point in the manifold
    for (auto& c : manifold.contacts) {
        // --- Calculate Relative Velocity at Contact Point ---
        XMVECTOR rA = XMVectorSubtract(c.position, comA);
        XMVECTOR rB = XMVectorSubtract(c.position, comB);

        XMVECTOR vA = A->getVelocity(iteration != 0);
        XMVECTOR wA = A->getAngularVelocity(iteration != 0);
        XMVECTOR vB = B->getVelocity(iteration != 0);
        XMVECTOR wB = B->getAngularVelocity(iteration != 0);

        // Velocity at contact point = linear velocity + angular velocity contribution
        XMVECTOR velA_contact = XMVectorAdd(vA, XMVector3Cross(wA, rA));
        XMVECTOR velB_contact = XMVectorAdd(vB, XMVector3Cross(wB, rB));
        XMVECTOR relVelLinear = XMVectorSubtract(velB_contact, velA_contact);

        // Angular velocity relative between bodies
        XMVECTOR relVelAngular = XMVectorSubtract(wB, wA); // NEW

        // --- Normal Impulse (Non-penetration & Restitution) ---
        XMVECTOR n = c.normal;
        float relVelNormal = XMVectorGetX(XMVector3Dot(relVelLinear, n));

		if (relVelNormal > 0.0f) {
			// Objects are separating, no need to resolve
			continue;
		}

        // Baumgarte bias makes the target velocity slightly positive for penetration
        // Restitution adds velocity based on the incoming speed
        float velocityBias = c.bias; // From pre-step
        float restitutionTerm = 0.0f;
        if (relVelNormal < -1.0f) { // Apply restitution only if velocity is significant enough
            restitutionTerm = -combinedRestitution * relVelNormal;
        }

        // Target velocity change = -(relative_velocity_normal + bias + restitution)
        // Note: Erin Catto's Box2D includes bias here. Some sources add it separately.
        // Let's calculate lambda needed to achieve zero relative velocity + bias + restitution
        float deltaVelNormalTarget = -relVelNormal + restitutionTerm; // Target change in velocity along normal
        float lambdaN = deltaVelNormalTarget * c.normalMass;

        // Clamp accumulated impulse: P_n >= 0
        float oldAccumulatedNormalImpulse = c.accumulatedNormalImpulse;
        c.accumulatedNormalImpulse = max(oldAccumulatedNormalImpulse + lambdaN, 0.0f);
        float actualLambdaN = c.accumulatedNormalImpulse - oldAccumulatedNormalImpulse; // The actual change applied

        // Apply normal impulse immediately (updates velocities for friction calc)
        XMVECTOR Pn_step = XMVectorScale(n, actualLambdaN);
        if (!A->isStatic()) A->applyImpulseAtPosition(XMVectorNegate(Pn_step), c.position);
        if (!B->isStatic()) B->applyImpulseAtPosition(Pn_step, c.position);

        // --- Friction Impulses (Linear and Angular) ---
        // Recalculate relative velocities *after* normal impulse applied
        // (Important for correct friction calculation in sequential impulse)
        vA = A->getVelocity(iteration != 0); wA = A->getAngularVelocity(iteration != 0); // Get updated velocities
        vB = B->getVelocity(iteration != 0); wB = B->getAngularVelocity(iteration != 0);
        velA_contact = XMVectorAdd(vA, XMVector3Cross(wA, rA));
        velB_contact = XMVectorAdd(vB, XMVector3Cross(wB, rB));
        relVelLinear = XMVectorSubtract(velB_contact, velA_contact);
        relVelAngular = XMVectorSubtract(wB, wA); // Update relative angular velocity too


        // --- Linear Friction Impulse ---
        XMVECTOR t = c.tangent; // Tangent computed in prestep based on initial relVel
        // Recompute tangent if necessary, or use pre-computed one.
        // Using the pre-computed one is common.
        // If tangentMass is zero, skip linear friction.
        if (c.tangentMass > 1e-9f) {
            float relVelTangent = XMVectorGetX(XMVector3Dot(relVelLinear, t));

            // Calculate impulse magnitude change needed (lambda_t) to stop tangent motion
            float lambdaT = -relVelTangent * c.tangentMass;

            // Clamp accumulated impulse within friction cone: |P_t| <= mu * P_n
            float maxFriction = combinedFriction * c.accumulatedNormalImpulse; // Use the *new* total normal impulse
            float oldAccumulatedFrictionImpulse = c.accumulatedFrictionImpulse;
            c.accumulatedFrictionImpulse = std::clamp(oldAccumulatedFrictionImpulse + lambdaT, -maxFriction, maxFriction);
            float actualLambdaT = c.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse; // The actual change applied

            // Apply linear friction impulse for this step
            XMVECTOR Pt_step = XMVectorScale(t, actualLambdaT);
            if (!A->isStatic()) A->applyImpulseAtPosition(XMVectorNegate(Pt_step), c.position);
            if (!B->isStatic()) B->applyImpulseAtPosition(Pt_step, c.position);
        }


        // --- Angular (Torsional) Friction Impulse --- (NEW)
        // If angularMass is zero, skip angular friction.
        if (c.angularMass > 1e-9f) {
            // Relative angular velocity projected onto the normal
            float relOmegaNormal = XMVectorGetX(XMVector3Dot(relVelAngular, n));

            // Calculate impulse magnitude change needed (lambda_a) to stop relative rotation around normal
            float lambdaA = -relOmegaNormal * c.angularMass;

            // Clamp accumulated impulse: |P_a| <= mu_angular * P_n
            float maxAngularFriction = combinedAngularFriction * c.accumulatedNormalImpulse; // Use the *new* total normal impulse
            float oldAccumulatedAngularImpulse = c.accumulatedAngularFrictionImpulse;
            c.accumulatedAngularFrictionImpulse = std::clamp(oldAccumulatedAngularImpulse + lambdaA, -maxAngularFriction, maxAngularFriction);
            float actualLambdaA = c.accumulatedAngularFrictionImpulse - oldAccumulatedAngularImpulse; // Actual change

            // Apply angular friction impulse (pure torque) for this step
            XMVECTOR Pa_step = XMVectorScale(n, actualLambdaA);
            if (!A->isStatic()) A->applyAngularImpulse(XMVectorNegate(Pa_step));
            if (!B->isStatic()) B->applyAngularImpulse(Pa_step);
        }
    } // End loop over contact points
}

DirectX::XMVECTOR PhysicsEngine::computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal) {
    DirectX::XMVECTOR tangent = DirectX::XMVectorSubtract(relVel, DirectX::XMVectorScale(normal, DirectX::XMVectorGetX(DirectX::XMVector3Dot(relVel, normal))));
    if (DirectX::XMVector3NearEqual(tangent, DirectX::XMVectorZero(), DirectX::XMVectorSplatEpsilon())) return tangent;
    return DirectX::XMVector3Normalize(tangent);
}

void PhysicsEngine::positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b) {
    using namespace DirectX;

    if (a->isStatic() && b->isStatic()) return;

    const float percent = .3f;
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
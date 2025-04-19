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

    for (int i = 0; i < candidates.size(); i++) {
        auto pair = candidates[i];
        PhysicsObject* objA = pair.first;
        PhysicsObject* objB = pair.second;

        if (auto newManifold = m_collisionSystem.checkCollision(objA, objB)) {
            auto pairKey = makePairKey(objA, objB);

            auto it = m_contactManifolds.find(pairKey);

            if (it != m_contactManifolds.end()) {
                CollisionManifold& oldManifold = it->second;

                if (!newManifold->contacts.empty() && !oldManifold.contacts.empty()) {
                    // Simple case: assume one contact and transfer impulses
                    newManifold->contacts[0].accumulatedNormalImpulse = oldManifold.contacts[0].accumulatedNormalImpulse;
                    newManifold->contacts[0].accumulatedFrictionImpulse = oldManifold.contacts[0].accumulatedFrictionImpulse;
                }
                currentFrameManifolds[pairKey] = *newManifold; // Copy the manifold
            }
            else {
                currentFrameManifolds[pairKey] = *newManifold; // Copy the manifold
            }
        }
    }
    m_contactManifolds = std::move(currentFrameManifolds); // Efficient swap

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
        XMVECTOR relVel = XMVectorSubtract(velB_contact, velA_contact);

        // --- Normal Impulse (Non-penetration & Restitution) ---
        XMVECTOR n = c.normal;
        float relVelNormal = XMVectorGetX(XMVector3Dot(relVel, n));

		if (relVelNormal > 0.0f) {
			// Objects are separating, no need to resolve
			continue;
		}

        // Calculate impulse magnitude change needed (lambda_n)
        // Formula: lambda = - (J V + bias + restitution) / (J M^-1 J^T)
        // Where J M^-1 J^T = 1 / effectiveMass = normalMass
        // J V = relVelNormalfloat 
        float velocityDueToRestitution = -combinedRestitution * relVelNormal;
        float deltaVelocity = velocityDueToRestitution - relVelNormal;
        float lambdaN = (deltaVelocity)*c.normalMass;

        // Clamp accumulated impulse: P_n >= 0 (contacts only push)
        float oldAccumulatedNormalImpulse = c.accumulatedNormalImpulse;
        c.accumulatedNormalImpulse = max(oldAccumulatedNormalImpulse + lambdaN, 0.0f);
        float actualLambdaN = c.accumulatedNormalImpulse - oldAccumulatedNormalImpulse; // The actual change applied

        // --- Friction Impulse ---
        XMVECTOR t = c.tangent; // Tangent computed in prestep
        float relVelTangent = XMVectorGetX(XMVector3Dot(relVel, t));

        // Calculate impulse magnitude change needed (lambda_t)
        // Formula: lambda = - (J V) / (J M^-1 J^T)
        // Where J M^-1 J^T = 1 / effectiveMass = tangentMass
        // J V = relVelTangent
        float lambdaT = -relVelTangent * c.tangentMass;

        // Clamp accumulated impulse within friction cone: |P_t| <= mu * P_n
        float maxFriction = combinedFriction * c.accumulatedNormalImpulse; // Use the *new* total normal impulse
        float oldAccumulatedFrictionImpulse = c.accumulatedFrictionImpulse;
        c.accumulatedFrictionImpulse = std::clamp(oldAccumulatedFrictionImpulse + lambdaT, -maxFriction, maxFriction);
        float actualLambdaT = c.accumulatedFrictionImpulse - oldAccumulatedFrictionImpulse; // The actual change applied

        // --- Apply Impulses ---
        // Calculate total impulse vector applied *in this step*
        XMVECTOR Pn = XMVectorScale(n, actualLambdaN);
        XMVECTOR Pt = XMVectorScale(t, actualLambdaT);
        XMVECTOR P = XMVectorAdd(Pn, Pt); // Total impulse vector for this step

        // Apply impulse to objects (remember impulse on A is -P)
        // Note: The applyImpulseAtPosition function needs to update both
        // linear and angular velocity based on the impulse P, the application
        // point 'pos', the inverse mass, and inverse inertia tensor.
        if (!A->isStatic()) {
            A->applyImpulseAtPosition(XMVectorNegate(P), c.position);
        }
        if (!B->isStatic()) {
            B->applyImpulseAtPosition(P, c.position);
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
std::pair<PhysicsObject*, PhysicsObject*> PhysicsEngine::makePairKey(PhysicsObject* objA, PhysicsObject* objB) {
    return (objA < objB) ? std::make_pair(objA, objB) : std::make_pair(objB, objA);
}
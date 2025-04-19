#include "PhysicsEngine.h"
#include <algorithm>
#include <string>
#include <omp.h>

void PhysicsEngine::onUpdate(float deltaTime) {
    // A. Integrate motion for all dynamic objects
    //#pragma omp parallel for
    for (int i = 0; i < m_bodies.size(); i++) {
        auto body = m_bodies[i].get();
        if (!body->isStatic()) {
            body->swapStates();
            body->onUpdate(deltaTime);
        }
    }

    // C. Detect and resolve collisions
    detectAndResolveCollisions(deltaTime);
}

void PhysicsEngine::addBody(std::shared_ptr<PhysicsObject> body) {
	m_bodies.push_back(body);
}

void PhysicsEngine::detectAndResolveCollisions(const float& deltaTime) {
    // 1. Broad phase (get potential pairs)
    auto candidates = broadPhase();

    // 2. Narrow phase (exact collision checks)
    std::vector<CollisionManifold> collisions(candidates.size());

    //#pragma omp parallel for
    for (int i = 0; i < candidates.size(); i++) {
		auto pair = &candidates[i];
        if (auto manifold = m_collisionSystem.checkCollision(pair->first, pair->second)) {
            collisions[i] = (*manifold);
        }
    }

    // --- PRE-STEP ---
    // Compute each contact's effective masses,bias, and warm-start impulses
    prestepCollisionManifolds(collisions, deltaTime);

    // ITERATIVE VELOCITY SOLVE
    for (int iter = 0; iter < m_velocityIterations; ++iter) {
        for (int j = 0; j < (int)collisions.size(); ++j) {
            auto& manifold = collisions[j];
            if (manifold.contacts.empty()) continue;
            // This now runs one "Gauss–Seidel" pass over all contacts:
            resolveCollisionVelocity(manifold);
        }
    }

    // ITERATIVE POSITION SOLVE (split-impulse style, e.g. 4 passes):
    for (int i = 0; i < m_positionIterations; ++i) {
        for (int j = 0; j < collisions.size(); ++j) {
            auto& manifold = collisions[j];
            if (manifold.contacts.empty()) continue;
            for (auto& contact : manifold.contacts) {
                positionalCorrection(contact, manifold.objectA, manifold.objectB);
            }
        }
    }

    /*

    // (Optional) Warm-start: apply last frame's impulses stored in each manifold
    // so that the solver "remembers" previous corrections and converges faster.

    #pragma omp parallel for
    for (int i = 0; i < collisions.size(); i++) {
		auto& manifold = collisions[i];
		if (manifold.contacts.empty()) continue;
        resolveCollisionVelocity(manifold);
    }

    #pragma omp parallel for
    for (int i = 0; i < collisions.size(); i++) {

        auto& manifold = collisions[i];

        if (manifold.contacts.empty()) continue;
        ContactPoint deepestContact;
        float maxPenetration = 0.0f;

        for (const auto& contact : manifold.contacts) {
            // Track deepest contact
            if (contact.penetration > maxPenetration) {
                maxPenetration = contact.penetration;
                deepestContact = contact;
            }
        }
        positionalCorrection(deepestContact, manifold.objectA, manifold.objectB);
    }
    */
}

std::vector<std::pair<PhysicsObject*, PhysicsObject*>> PhysicsEngine::broadPhase() {
    // Simple approach: Check all pairs (replace with spatial partitioning later)
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

void PhysicsEngine::prestepCollisionManifolds(std::vector<CollisionManifold>& manifolds, const float& dt) {
    using namespace DirectX;

    for (auto& m : manifolds) {
        PhysicsObject* A = m.objectA;
        PhysicsObject* B = m.objectB;

		// Skip empty manifolds
		if (m.contacts.empty()) continue;

        // Skip static–static
        if (A->isStatic() && B->isStatic()) continue;

        // Precompute inverse mass and inertia
        float invMassA = A->isStatic() ? 0.0f : 1.0f / A->getMass();
        float invMassB = B->isStatic() ? 0.0f : 1.0f / B->getMass();
        XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor();
        XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor();

        // COM positions
        XMVECTOR comA = A->getTransform().GetPosition(0);
        XMVECTOR comB = B->getTransform().GetPosition(0);

        for (auto& c : m.contacts) {
            // 1) Build rA, rB
            XMVECTOR rA = XMVectorSubtract(c.position, comA);
            XMVECTOR rB = XMVectorSubtract(c.position, comB);
            XMVECTOR n = c.normal;

            // 2) Effective mass along normal:
            XMVECTOR raCrossN = XMVector3Cross(rA, n);
            XMVECTOR rbCrossN = XMVector3Cross(rB, n);
            float angularA = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(raCrossN, invInertiaA), raCrossN));
            float angularB = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(rbCrossN, invInertiaB), rbCrossN));
            float invMassSum = invMassA + invMassB + angularA + angularB;
            c.normalMass = invMassSum > 0.0f ? 1.0f / invMassSum : 0.0f;

            // 3) Effective mass along tangent:
            //    we pick tangent from current relVel so friction lines up
            XMVECTOR velA = XMVectorAdd(A->getVelocity(),
                XMVector3Cross(A->getAngularVelocity(), rA));
            XMVECTOR velB = XMVectorAdd(B->getVelocity(),
                XMVector3Cross(B->getAngularVelocity(), rB));
            XMVECTOR relVel = XMVectorSubtract(velB, velA);

            // reuse your computeTangent()
            XMVECTOR t = computeTangent(relVel, n);

            XMVECTOR raCrossT = XMVector3Cross(rA, t);
            XMVECTOR rbCrossT = XMVector3Cross(rB, t);
            float angularTA = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(raCrossT, invInertiaA), raCrossT));
            float angularTB = XMVectorGetX(
                XMVector3Dot(XMVector3Transform(rbCrossT, invInertiaB), rbCrossT));
            float invMassT = invMassA + invMassB + angularTA + angularTB;
            c.tangentMass = invMassT > 0.0f ? 1.0f / invMassT : 0.0f;

            // 4) Baumgarte bias to correct penetration over time
            float penetrationErr = max(c.penetration - m_kPenetrationSlop, 0.0f);
            c.bias = (m_kBaumgarte / dt) * penetrationErr;

            // 5) Warm-start: apply last frame's accumulated impulses
            //    P = N * normalImpulse + T * frictionImpulse
            XMVECTOR Pn = XMVectorScale(n, c.accumulatedNormalImpulse);
            XMVECTOR Pt = XMVectorScale(t, c.accumulatedFrictionImpulse);
            XMVECTOR P = XMVectorAdd(Pn, Pt);

            // apply(-P) to A, +P to B
            A->applyImpulseAtPosition(XMVectorNegate(P), c.position);
            B->applyImpulseAtPosition(P, c.position);
        }
    }
}

void PhysicsEngine::resolveCollisionVelocity(CollisionManifold& manifold) {
    using namespace DirectX;
    using namespace DirectX;

    auto* A = manifold.objectA;
    auto* B = manifold.objectB;
    if (A->isStatic() && B->isStatic()) return;

    // Precompute inverse masses & inertias per manifold
    float invMassA = A->isStatic() ? 0.0f : 1.0f / A->getMass();
    float invMassB = B->isStatic() ? 0.0f : 1.0f / B->getMass();
    XMMATRIX invInertiaA = A->getInverseWorldInertiaTensor();
    XMMATRIX invInertiaB = B->getInverseWorldInertiaTensor();

    // Center-of-mass positions
    XMVECTOR comA = A->getTransform().GetPosition(0);
    XMVECTOR comB = B->getTransform().GetPosition(0);

    for (auto& c : manifold.contacts) {
        const XMVECTOR& n = c.normal;

        // 1) Compute rA, rB
        XMVECTOR rA = c.position - comA;
        XMVECTOR rB = c.position - comB;

        // 2) Relative velocity at contact
        XMVECTOR vA = A->getVelocity()
            + XMVector3Cross(A->getAngularVelocity(), rA);
        XMVECTOR vB = B->getVelocity()
            + XMVector3Cross(B->getAngularVelocity(), rB);
        XMVECTOR relVel = vB - vA;

        // 3) NORMAL IMPULSE
        float vn = XMVectorGetX(XMVector3Dot(relVel, n));
        //    a) restitution bounce (only if closing fast enough)
        float e = (A->getMaterial().restitution +
            B->getMaterial().restitution) * 0.5f;
        float bounceTerm = (vn < -m_kRestitutionThreshold) ? (-e * vn) : 0.0f;
        float biasTerm = c.bias; // Position correction (precomputed)
        //    b) total impulse scalar
        float dPn = c.normalMass * (-vn + bounceTerm + biasTerm);

        //    c) clamp & accumulate
        float oldPn = c.accumulatedNormalImpulse;
        float newPn = max(oldPn + dPn, 0.0f);
        dPn = newPn - oldPn;
        c.accumulatedNormalImpulse = newPn;

        //    d) apply normal impulse
        XMVECTOR Pn = n * dPn;
        A->applyImpulseAtPosition(-Pn, c.position);
        B->applyImpulseAtPosition(Pn, c.position);

        // 4) FRICTION IMPULSE
        //    a) recompute relVel *after* normal impulse
        vA = A->getVelocity()
            + XMVector3Cross(A->getAngularVelocity(), rA);
        vB = B->getVelocity()
            + XMVector3Cross(B->getAngularVelocity(), rB);
        relVel = vB - vA;
        //    b) find tangent
        XMVECTOR t = computeTangent(relVel, n);
        //    c) magnitude
        float vt = XMVectorGetX(XMVector3Dot(relVel, t));
        float dPt = -vt * c.tangentMass;
        //    d) clamp by Coulomb cone
        float u = (A->getMaterial().friction +
            B->getMaterial().friction) * 0.5f;
        float maxPt = c.accumulatedNormalImpulse * u;
        float oldPt = c.accumulatedFrictionImpulse;
        float newPt = std::clamp(oldPt + dPt, -maxPt, +maxPt);
        dPt = newPt - oldPt;
        c.accumulatedFrictionImpulse = newPt;
        //    e) apply friction impulse
        XMVECTOR Pt = t * dPt;
        A->applyImpulseAtPosition(-Pt, c.position);
        B->applyImpulseAtPosition(Pt, c.position);
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

    // Configuration
    const float percent = 1.0f;     // Can be changed to less but this means we need more iterations
    const float slop = 0.001f;      // Permitted penetration

    const float penetrationDepth = contact.penetration - slop;
    if (penetrationDepth <= 0.0f) return;

    // Mass calculations
    const float invMassA = a->isStatic() ? 0.0f : 1.0f / a->getMass();
    const float invMassB = b->isStatic() ? 0.0f : 1.0f / b->getMass();
    const float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    // Correction vector
    const XMVECTOR correction = XMVectorScale(
        contact.normal,
        (penetrationDepth * percent) / totalInvMass
    );

    if (!a->isStatic()) {
        a->getTransform().Translate(XMVectorScale(correction, invMassA), 1);
    }
    if (!b->isStatic()) {
        b->getTransform().Translate(XMVectorScale(correction, -invMassB), 1);
    }
}
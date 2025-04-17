#include "PhysicsEngine.h"
#include <algorithm>

void PhysicsEngine::onUpdate(float deltaTime) {
    // A. Integrate motion for all dynamic objects
    for (auto& body : m_bodies) {
        if (!body->isStatic()) {
            body->onUpdate(deltaTime);
        }
    }

    // C. Detect and resolve collisions
    detectAndResolveCollisions();
}

void PhysicsEngine::addBody(std::shared_ptr<PhysicsObject> body) {
	m_bodies.push_back(body);
}

void PhysicsEngine::detectAndResolveCollisions() {
    // 1. Broad phase (get potential pairs)
    auto candidates = broadPhase();

    // 2. Narrow phase (exact collision checks)
    std::vector<CollisionManifold> collisions;
    for (auto& pair : candidates) {
        if (auto manifold = m_collisionSystem.checkCollision(pair.first, pair.second)) {
			for (auto& contact : manifold->contacts) {
				contact.normalImpulse = 0;
                contact.tangentImpulse = 0;
			}
            collisions.push_back(*manifold);
        }
    }

    // (Optional) Warm-start: apply last frame's impulses stored in each manifold
    // so that the solver "remembers" previous corrections and converges faster.

    for (int iter = 0; iter < m_velocityIterations; ++iter) {
        for (auto& manifold : collisions) {
            resolveCollisionVelocity(manifold);
        }
    }

    for (int iter = 0; iter < m_positionIterations; ++iter) {
        for (auto& manifold : collisions) {
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
    }
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

void PhysicsEngine::resolveCollisionPosition(const CollisionManifold& manifold) {
    using namespace DirectX;
    PhysicsObject* a = manifold.objectA;
    PhysicsObject* b = manifold.objectB;

    if (a->isStatic() && b->isStatic()) return;

    ContactPoint deepestContact;
    float maxPenetration = 0.0f;

    for (const auto& contact : manifold.contacts) {
        // Track deepest contact
        if (contact.penetration > maxPenetration) {
            maxPenetration = contact.penetration;
            deepestContact = contact;
        }
    }

    // Second pass: Positional correction once per manifold
    if (maxPenetration > 0.0f) {
        positionalCorrection(deepestContact, a, b);
    }
}

void PhysicsEngine::resolveCollisionVelocity(const CollisionManifold& manifold) {
    using namespace DirectX;
    PhysicsObject* a = manifold.objectA;
    PhysicsObject* b = manifold.objectB;

    if (a->isStatic() && b->isStatic()) return;

    for (const auto& contact : manifold.contacts) {
        // Get center of mass positions
        XMVECTOR comA = a->getTransform().GetPosition();
        XMVECTOR comB = b->getTransform().GetPosition();

        // Calculate vectors from COM to contact point
        XMVECTOR rA = XMVectorSubtract(contact.position, comA);
        XMVECTOR rB = XMVectorSubtract(contact.position, comB);

        // Calculate velocities at contact point including rotation
        XMVECTOR velA = XMVectorAdd(a->getVelocity(),
            XMVector3Cross(a->getAngularVelocity(), rA));
        XMVECTOR velB = XMVectorAdd(b->getVelocity(),
            XMVector3Cross(b->getAngularVelocity(), rB));

        XMVECTOR relVel = XMVectorSubtract(velB, velA);
        float contactVel = XMVectorGetX(XMVector3Dot(relVel, contact.normal));

        // Calculate tangent direction using PRE-impulse relVel
        XMVECTOR tangent = XMVectorSubtract(relVel,
            XMVectorScale(contact.normal, XMVectorGetX(XMVector3Dot(relVel, contact.normal))));
        if (!XMVector3NearEqual(tangent, XMVectorZero(), XMVectorSplatEpsilon())) {
            tangent = XMVector3Normalize(tangent);
        }

        // Calculate restitution
        float restitution = (a->getMaterial().restitution +
            b->getMaterial().restitution) / 2.0f;

        // Calculate effective mass (including rotational inertia)
        float invMassA = a->isStatic() ? 0.0f : 1.0f / a->getMass();
        float invMassB = b->isStatic() ? 0.0f : 1.0f / b->getMass();

        XMVECTOR crossA = XMVector3Cross(rA, contact.normal);
        XMVECTOR crossB = XMVector3Cross(rB, contact.normal);

        XMMATRIX invInertiaA = a->getInverseWorldInertiaTensor();
        XMMATRIX invInertiaB = b->getInverseWorldInertiaTensor();

        float angularEffectA = XMVectorGetX(XMVector3Dot(
            XMVector3Transform(crossA, invInertiaA),
            crossA));

        float angularEffectB = XMVectorGetX(XMVector3Dot(
            XMVector3Transform(crossB, invInertiaB),
            crossB));

        float denominator = invMassA + invMassB + angularEffectA + angularEffectB;
        if (denominator <= 0.0f) continue;

        float impulseScalar = -(1.0f + restitution) * contactVel / denominator;

        // Apply impulses at contact point
        XMVECTOR impulse = XMVectorScale(contact.normal, impulseScalar);
        a->applyImpulseAtPosition(XMVectorScale(impulse, -1.0f), contact.position);
        b->applyImpulseAtPosition(impulse, contact.position);

        // Apply friction (optional)
        applyFriction(contact, a, b, rA, rB, impulseScalar, relVel);
    }
}

DirectX::XMVECTOR PhysicsEngine::computeTangent(DirectX::XMVECTOR relVel, DirectX::XMVECTOR normal) {
    DirectX::XMVECTOR tangent = DirectX::XMVectorSubtract(relVel, DirectX::XMVectorScale(normal, DirectX::XMVectorGetX(DirectX::XMVector3Dot(relVel, normal))));
    if (DirectX::XMVector3NearEqual(tangent, DirectX::XMVectorZero(), DirectX::XMVectorSplatEpsilon())) return tangent;
    return DirectX::XMVector3Normalize(tangent);
}

void PhysicsEngine::applyFriction(const ContactPoint& contact,
    PhysicsObject* a, PhysicsObject* b,
    const DirectX::XMVECTOR& rA,
    const DirectX::XMVECTOR& rB,
    const float& normalImpulse,
    const DirectX::XMVECTOR& preRelVel) { // Pass pre-normal-impulse relVel
    using namespace DirectX;


    // Calculate tangent direction from pre-normal-impulse relVel
    XMVECTOR tangent = computeTangent(preRelVel, contact.normal);
    if (XMVector3Equal(tangent, XMVectorZero())) return;

    // Calculate effective mass for friction (unchanged)
    float invMassA = a->isStatic() ? 0.0f : 1.0f / a->getMass();
    float invMassB = b->isStatic() ? 0.0f : 1.0f / b->getMass();

    XMVECTOR crossA = XMVector3Cross(rA, tangent);
    XMVECTOR crossB = XMVector3Cross(rB, tangent);

    XMMATRIX invInertiaA = a->getInverseWorldInertiaTensor();
    XMMATRIX invInertiaB = b->getInverseWorldInertiaTensor();

    float angularEffectA = XMVectorGetX(XMVector3Dot(
        XMVector3Transform(crossA, invInertiaA),
        crossA));

    float angularEffectB = XMVectorGetX(XMVector3Dot(
        XMVector3Transform(crossB, invInertiaB),
        crossB));

    float denominator = invMassA + invMassB + angularEffectA + angularEffectB;
    if (denominator <= 0.0f) return;

    // Calculate friction impulse using PRE-normal-impulse tangential velocity
    float tangentRelVel = XMVectorGetX(XMVector3Dot(preRelVel, tangent));
    float frictionImpulseMag = -tangentRelVel / denominator;
    float friction = (a->getMaterial().friction + b->getMaterial().friction) / 2.0f;

    float maxFriction = std::abs(normalImpulse) * friction;
    frictionImpulseMag = std::clamp(frictionImpulseMag, -maxFriction, maxFriction);

    // Apply friction impulses
    XMVECTOR frictionImpulse = XMVectorScale(tangent, frictionImpulseMag);
    a->applyImpulseAtPosition(XMVectorScale(frictionImpulse, -1.0f), contact.position);
    b->applyImpulseAtPosition(frictionImpulse, contact.position);
}

void PhysicsEngine::positionalCorrection(const ContactPoint& contact, PhysicsObject* a, PhysicsObject* b) {
    using namespace DirectX;

    if (a->isStatic() && b->isStatic()) return;

    // Configuration
    const float percent = 1.0f; // Can be changed to less but this means we need more iterations
    const float slop = 0.001f;     // Permitted penetration

    const float penetrationDepth = contact.penetration - slop;
    if (penetrationDepth <= 0.0f) return;

    // Mass calculations
    const float invMassA = a->isStatic() ? 0.0f : 1.0f / a->getMass();
    const float invMassB = b->isStatic() ? 0.0f : 1.0f / b->getMass();
    const float totalInvMass = invMassA + invMassB;
    if (totalInvMass <= 0.0f) return;

    // Correction vector (DIRECTION FIXED HERE)
    const XMVECTOR correction = XMVectorScale(
        contact.normal,
        (penetrationDepth * percent) / totalInvMass
    );

    if (!a->isStatic()) {
        a->getTransform().Translate(XMVectorScale(correction, invMassA));
    }
    if (!b->isStatic()) {
        b->getTransform().Translate(XMVectorScale(correction, -invMassB));
    }
}
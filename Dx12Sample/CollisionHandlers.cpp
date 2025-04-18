#include "CollisionHandlers.h"

std::optional<CollisionManifold> CollisionHandlers::SphereVsSphere(PhysicsObject* a, PhysicsObject* b)
 {
     const SphereCollider* sphereA = static_cast<SphereCollider*>(a->getCollider());
     const SphereCollider* sphereB = static_cast<SphereCollider*>(b->getCollider());

     const DirectX::XMVECTOR posA = a->getTransform().GetPosition(1);
     const DirectX::XMVECTOR posB = b->getTransform().GetPosition(1);
     const DirectX::XMVECTOR delta = DirectX::XMVectorSubtract(posA, posB);

     const float distance = DirectX::XMVectorGetX(DirectX::XMVector3Length(delta));
     const float radiusSum = sphereA->getRadius() + sphereB->getRadius();

     if (distance >= radiusSum) return std::nullopt;

     CollisionManifold manifold;
     manifold.objectA = a;
     manifold.objectB = b;

     // Single contact point at midpoint
     manifold.contacts.push_back({
         DirectX::XMVectorAdd(posA, DirectX::XMVectorScale(delta, 0.5f)),
         DirectX::XMVector3Normalize(delta),
         radiusSum - distance
         });

     return manifold;
}

std::optional<CollisionManifold> CollisionHandlers::SphereVsBox(PhysicsObject* sphereObj, PhysicsObject* boxObj) {
    using namespace DirectX;

    const SphereCollider* sphere = static_cast<SphereCollider*>(sphereObj->getCollider());
    const BoxCollider* box = static_cast<BoxCollider*>(boxObj->getCollider());

    const XMVECTOR sphereCenter = sphereObj->getTransform().GetPosition(1);
    const float sphereRadius = sphere->getRadius();
    Transform& boxTransform = boxObj->getTransform();

    // Get closest point on box surface and check containment
    const XMVECTOR closestPoint = box->closestPoint(&boxTransform, sphereCenter);
    const bool isInside = box->containsPoint(&boxTransform, sphereCenter);
    const XMVECTOR delta = XMVectorSubtract(sphereCenter, closestPoint);
    const float distance = XMVectorGetX(XMVector3Length(delta));

    if (distance >= sphereRadius && !isInside) return std::nullopt;

    CollisionManifold manifold;
    manifold.objectA = sphereObj;
    manifold.objectB = boxObj;

    if (isInside) {
        // Calculate penetration using box's face normals
        const std::vector<XMVECTOR> faceNormals = box->getFaceNormals(&boxTransform);
        const XMVECTOR boxCenter = boxTransform.GetPosition(1);
        const XMVECTOR halfSize = box->getHalfSize();

        XMVECTOR maxNormal = XMVectorZero();
        float maxPenetration = -FLT_MAX;

        // Check all face normals (including negative directions)
        for (const XMVECTOR& normal : faceNormals) {
            // Get positive and negative normals for each axis
            for (int sign = -1; sign <= 1; sign += 2) {
                const XMVECTOR dir = XMVectorMultiply(normal, XMVectorReplicate((float)sign));
                const XMVECTOR facePoint = XMVectorAdd(boxCenter,
                    XMVectorMultiply(dir, halfSize));

                const float penetration = sphereRadius +
                    XMVectorGetX(XMVector3Dot(dir, XMVectorSubtract(sphereCenter, facePoint)));

                if (penetration > maxPenetration) {
                    maxPenetration = penetration;
                    maxNormal = dir;
                }
            }
        }

        manifold.contacts.push_back({
            closestPoint,
            XMVector3Normalize(maxNormal),
            maxPenetration
            });
    }
    else {
        // External collision
        const XMVECTOR normal = XMVector3Normalize(delta);
        const float penetration = sphereRadius - distance;

        manifold.contacts.push_back({
            closestPoint,
            normal,
            penetration
        });
    }

    return manifold;
}

std::optional<CollisionManifold> CollisionHandlers::BoxVsBox(PhysicsObject* a, PhysicsObject* b) {
    return std::nullopt;
}
#include "CollisionHandlers.h"
#include "CapsuleCollider.h"
#include <Helpers.h>

std::optional<CollisionManifold> CollisionHandlers::SphereVsSphere(PhysicsObject* a, PhysicsObject* b, const bool& flip)
{
     const SphereCollider* sphereA = static_cast<SphereCollider*>(a->getCollider());
     const SphereCollider* sphereB = static_cast<SphereCollider*>(b->getCollider());

     const DirectX::XMVECTOR posA = a->getTransform().GetPosition(1);
     const DirectX::XMVECTOR posB = b->getTransform().GetPosition(1);
     const DirectX::XMVECTOR delta = DirectX::XMVectorSubtract(posB, posA);

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

     if(flip)
	 {
		 // Swap the objects in the manifold if flip is true
		 std::swap(manifold.objectA, manifold.objectB);
         for (auto& contact : manifold.contacts) {
             contact.normal = DirectX::XMVectorNegate(contact.normal);
         }
	 }
     return manifold;
}

std::optional<CollisionManifold> CollisionHandlers::SphereVsBox(PhysicsObject* sphereObj, PhysicsObject* boxObj, const bool& flip) {
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
		const XMVECTOR normal = XMVector3Normalize(XMVectorSubtract(closestPoint, sphereCenter));
        const float penetration = sphereRadius - distance;

        manifold.contacts.push_back({
            closestPoint, // Contact point might be better as closestPoint - normal * penetration
            normal,
            penetration
        });
    }

    if (flip)
    {
        // Swap the objects in the manifold if flip is true
        std::swap(manifold.objectA, manifold.objectB);
		for (auto& contact : manifold.contacts) {
			contact.normal = XMVectorNegate(contact.normal);
		}
    }
    return manifold;
}

std::optional<CollisionManifold> CollisionHandlers::SphereVsCapsule(PhysicsObject* sphereObj, PhysicsObject* capsuleObj, const bool& flip)
{
    using namespace DirectX;

    // 1. Get Colliders and Transforms
    const SphereCollider* sphereCollider = static_cast<SphereCollider*>(sphereObj->getCollider());
    const CapsuleCollider* capsuleCollider = static_cast<CapsuleCollider*>(capsuleObj->getCollider());

    Transform& sphereTransform = sphereObj->getTransform();
    Transform& capsuleTransform = capsuleObj->getTransform();

    // 2. Get Collision Properties
    const float sphereRadius = sphereCollider->getRadius();
    const float capsuleRadius = capsuleCollider->getRadius();
    const float radiusSum = sphereRadius + capsuleRadius;

    const XMVECTOR sphereCenter = sphereTransform.GetPosition(1); // Assuming state 1

    // 3. Get Capsule World Segment
    XMVECTOR localP1, localP2;
    capsuleCollider->getLocalSegmentEndpoints(localP1, localP2);

    XMMATRIX capsuleWorldMatrix = capsuleTransform.GetWorldMatrix(1); // Assuming state 1
    XMVECTOR worldP1 = Math::LocalToWorld(capsuleWorldMatrix, localP1);
    XMVECTOR worldP2 = Math::LocalToWorld(capsuleWorldMatrix, localP2);

    // 4. Find Closest Point on Capsule Segment to Sphere Center
    XMVECTOR closestPointOnSegment = Math::ClosestPointOnLineSegment(sphereCenter, worldP1, worldP2);

    // 5. Calculate Distance and Check Collision
    XMVECTOR delta = XMVectorSubtract(closestPointOnSegment, sphereCenter);
    XMVECTOR distanceSq = XMVector3LengthSq(delta);

    if (XMVectorGetX(distanceSq) >= (radiusSum * radiusSum))
    {
        return std::nullopt; // No collision
    }

    // 6. Collision Detected - Generate Manifold
    CollisionManifold manifold;
    manifold.objectA = sphereObj;
    manifold.objectB = capsuleObj;

    float distance = sqrtf(XMVectorGetX(distanceSq));
    float penetration = radiusSum - distance;

    // Normal points from capsule towards sphere
    XMVECTOR normal = Math::NormalizeSafe(delta);
    // If distance is near zero, sphere center is on the segment. Choose an arbitrary normal?
    // NormalizeSafe handles this, returning zero. Let's provide a fallback.
    if (XMVectorGetX(XMVector3LengthSq(normal)) < 1e-8f) {
        // Fallback: Use the vector from capsule segment start to sphere center,
        // or just an arbitrary axis like world Y if that also fails.
        normal = Math::NormalizeSafe(XMVectorSubtract(sphereCenter, worldP1));
        if (XMVectorGetX(XMVector3LengthSq(normal)) < 1e-8f) {
            normal = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f); // World Y up
        }
    }


    // Calculate contact point (midpoint between surfaces along the normal)
    // Point on capsule surface = closestPointOnSegment + normal * capsuleRadius
    // Point on sphere surface = sphereCenter - normal * sphereRadius
    XMVECTOR contactPoint = XMVectorAdd(closestPointOnSegment, XMVectorScale(normal, capsuleRadius));
    contactPoint = XMVectorAdd(contactPoint, XMVectorSubtract(sphereCenter, XMVectorScale(normal, sphereRadius)));
    contactPoint = XMVectorScale(contactPoint, 0.5f);

    // Alternative simpler contact point: Point on capsule surface
    // XMVECTOR contactPoint = XMVectorAdd(closestPointOnSegment, XMVectorScale(normal, capsuleRadius));

    manifold.contacts.push_back({
        contactPoint,
        normal,
        penetration
        });

    // 7. Handle Flip
    if (flip)
    {
        std::swap(manifold.objectA, manifold.objectB);
        for (auto& contact : manifold.contacts) {
            contact.normal = XMVectorNegate(contact.normal);
        }
    }

    return manifold;
}

std::optional<CollisionManifold> CollisionHandlers::BoxVsBox(PhysicsObject* a, PhysicsObject* b, const bool& flip) {
	OutputDebugStringW(L"BoxVsBox collision detection not implemented yet.\n");
    return std::nullopt;
}
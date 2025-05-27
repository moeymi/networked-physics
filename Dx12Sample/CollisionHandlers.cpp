#include "CollisionHandlers.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
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
     DirectX::XMFLOAT3 scaleA;
	 DirectX::XMFLOAT3 scaleB;

	 DirectX::XMStoreFloat3(&scaleA, a->getTransform().GetScale(0));
	 DirectX::XMStoreFloat3(&scaleB, b->getTransform().GetScale(0));
     const float radiusSum = sphereA->getRadius() * max(max(scaleA.x, scaleA.y), scaleA.z) + sphereB->getRadius() * max(max(scaleB.x, scaleB.y), scaleB.z);

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

//	 Log::Info() << "SphereVsSphere: Collision detected with distance: " << distance << " at " <<
//		 DirectX::XMVectorGetX(manifold.contacts[0].position) << ", " <<
//		 DirectX::XMVectorGetY(manifold.contacts[0].position) << ", " <<
//		 DirectX::XMVectorGetZ(manifold.contacts[0].position) << " with normal: " <<
//		 DirectX::XMVectorGetX(manifold.contacts[0].normal) << ", " <<
//		 DirectX::XMVectorGetY(manifold.contacts[0].normal) << ", " <<
//		 DirectX::XMVectorGetZ(manifold.contacts[0].normal) << " and penetration: " << manifold.contacts[0].penetration << std::endl;


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

    DirectX::XMFLOAT3 sphereScale;
    DirectX::XMStoreFloat3(&sphereScale, sphereObj->getTransform().GetScale(0));

    const float sphereRadius = sphere->getRadius() * max(max(sphereScale.x, sphereScale.y), sphereScale.z);
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
        const auto faceNormals = box->getFaceNormals(&boxTransform);
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

//	Log::Info() << "SphereVsBox: Collision detected with distance: " << distance << " at {" <<
//		XMVectorGetX(manifold.contacts[0].position) << ", " <<
//		XMVectorGetY(manifold.contacts[0].position) << ", " <<
//		XMVectorGetZ(manifold.contacts[0].position) << "} with normal: {" <<
//		XMVectorGetX(manifold.contacts[0].normal) << ", " <<
//		XMVectorGetY(manifold.contacts[0].normal) << ", " <<
//		XMVectorGetZ(manifold.contacts[0].normal) << "} and penetration: " << manifold.contacts[0].penetration << std::endl;

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
    DirectX::XMFLOAT3 sphereScale;
    DirectX::XMStoreFloat3(&sphereScale, sphereObj->getTransform().GetScale(0));

    const float sphereRadius = sphereCollider->getRadius() * max(max(sphereScale.x, sphereScale.y), sphereScale.z);
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


//    Log::Info() << "SphereVsCasule: Collision detected with distance: " << distance << " at " <<
//        XMVectorGetX(contactPoint) << ", " <<
//        XMVectorGetY(contactPoint) << ", " <<
//        XMVectorGetZ(contactPoint) << " with normal: " <<
//        XMVectorGetX(normal) << ", " <<
//        XMVectorGetY(normal) << ", " <<
//        XMVectorGetZ(normal) << " and penetration: " << penetration << std::endl;


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

std::optional<CollisionManifold> CollisionHandlers::CapsuleVsCapsule(PhysicsObject* aObj, PhysicsObject* bObj, const bool& flip) {
    using namespace DirectX;

    const CapsuleCollider* capA = static_cast<CapsuleCollider*>(aObj->getCollider());
    const CapsuleCollider* capB = static_cast<CapsuleCollider*>(bObj->getCollider());

    Transform& tA = aObj->getTransform();
    Transform& tB = bObj->getTransform();

    // local segment endpoints (in collider space)
    XMVECTOR aLocal0, aLocal1;
    capA->getLocalSegmentEndpoints(aLocal0, aLocal1);

    XMVECTOR bLocal0, bLocal1;
    capB->getLocalSegmentEndpoints(bLocal0, bLocal1);

    // world segment endpoints
    XMMATRIX mA = tA.GetWorldMatrix(1);
    XMMATRIX mB = tB.GetWorldMatrix(1);

    XMVECTOR a0 = Math::LocalToWorld(mA, aLocal0);
    XMVECTOR a1 = Math::LocalToWorld(mA, aLocal1);
    XMVECTOR b0 = Math::LocalToWorld(mB, bLocal0);
    XMVECTOR b1 = Math::LocalToWorld(mB, bLocal1);

    const float rA = capA->getRadius();   // (scale already baked into segment)
    const float rB = capB->getRadius();
    const float rSum = rA + rB;

    // (Dan Sunday's algorithm, adapted for XMVECTOR)
    auto ClosestPtsSegSeg = [](XMVECTOR p1, XMVECTOR q1,
        XMVECTOR p2, XMVECTOR q2,
        XMVECTOR& c1, XMVECTOR& c2)
        {
            const XMVECTOR d1 = XMVectorSubtract(q1, p1);  // S1 = p1 + s * d1
            const XMVECTOR d2 = XMVectorSubtract(q2, p2);  // S2 = p2 + t * d2
            const XMVECTOR r = XMVectorSubtract(p1, p2);
            const float    a = XMVectorGetX(XMVector3Dot(d1, d1)); // |d1|^2
            const float    e = XMVectorGetX(XMVector3Dot(d2, d2)); // |d2|^2
            const float    f = XMVectorGetX(XMVector3Dot(d2, r));

            float s, t;

            if (a <= 1e-6f && e <= 1e-6f) {          // both segments degenerate
                s = t = 0.0f;
                c1 = p1;
                c2 = p2;
            }
            else if (a <= 1e-6f) {                   // first degenerate
                s = 0.0f;
                t = std::clamp(f / e, 0.0f, 1.0f);
            }
            else {
                const float c = XMVectorGetX(XMVector3Dot(d1, r));
                if (e <= 1e-6f) {                    // second degenerate
                    t = 0.0f;
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                }
                else {
                    const float b = XMVectorGetX(XMVector3Dot(d1, d2));
                    const float denom = a * e - b * b;

                    s = (denom != 0.0f) ? std::clamp((b * f - c * e) / denom, 0.0f, 1.0f) : 0.0f;
                    float tNom = b * s + f;

                    if (tNom < 0.0f) {
                        t = 0.0f;
                        s = std::clamp(-c / a, 0.0f, 1.0f);
                    }
                    else if (tNom > e) {
                        t = 1.0f;
                        s = std::clamp((b - c) / a, 0.0f, 1.0f);
                    }
                    else {
                        t = tNom / e;
                    }
                }
            }

            c1 = XMVectorAdd(p1, XMVectorScale(d1, s));
            c2 = XMVectorAdd(p2, XMVectorScale(d2, t));
        };

    XMVECTOR pA, pB;
    ClosestPtsSegSeg(a0, a1, b0, b1, pA, pB);

    XMVECTOR delta = XMVectorSubtract(pB, pA);
    float     distSq = XMVectorGetX(XMVector3LengthSq(delta));

    if (distSq >= rSum * rSum)
        return std::nullopt;

    CollisionManifold m;
    m.objectA = aObj;
    m.objectB = bObj;

    float distance = std::sqrt(distSq);
    float penetration = rSum - distance;

    XMVECTOR normal = (distance > 1e-6f)
        ? XMVectorScale(delta, 1.0f / distance)   // delta / |delta|
        : XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);    // fallback axis

    XMVECTOR contact = XMVectorScale(XMVectorAdd(pA, pB), 0.5f);  // midpoint

    m.contacts.push_back({ contact, normal, penetration });


//    Log::Info() << "CapsuleVsCapsule: Collision detected with distance: " << distance << " at " <<
//        XMVectorGetX(contact) << ", " <<
//        XMVectorGetY(contact) << ", " <<
//        XMVectorGetZ(contact) << " with normal: " <<
//        XMVectorGetX(normal) << ", " <<
//        XMVectorGetY(normal) << ", " <<
//        XMVectorGetZ(normal) << " and penetration: " << penetration << std::endl;


    if (flip)
    {
        std::swap(m.objectA, m.objectB);
        for (auto& c : m.contacts)
            c.normal = XMVectorNegate(c.normal);
    }

    return m;
}

std::optional<CollisionManifold> CollisionHandlers::BoxVsCapsule(PhysicsObject* boxObj, PhysicsObject* capsuleObj, const bool& flip) {
    using namespace DirectX;

    // ------------------------------------------------------------------
    // 1. Colliders & transforms
    // ------------------------------------------------------------------
    const CapsuleCollider* cap = static_cast<CapsuleCollider*>(capsuleObj->getCollider());
    const BoxCollider* box = static_cast<BoxCollider*>(boxObj->getCollider());

    Transform& capT = capsuleObj->getTransform();
    Transform& boxT = boxObj->getTransform();

    // radius enlarged by non-uniform X/Z scale
    auto scaledRadius = [](const CapsuleCollider* c, const Transform& t)
        {
            XMFLOAT3 s; XMStoreFloat3(&s, t.GetScale(0));
            return c->getRadius() * max(std::abs(s.x), std::abs(s.z));
        };
    const float r = scaledRadius(cap, capT);

    // ------------------------------------------------------------------
    // 2. Capsule centre-line, contracted by ±r
    // ------------------------------------------------------------------
    XMVECTOR localTip0, localTip1;
    cap->getLocalSegmentEndpoints(localTip0, localTip1);

    XMMATRIX wCap = capT.GetWorldMatrix(1);
    XMVECTOR tip0 = Math::LocalToWorld(wCap, localTip0);
    XMVECTOR tip1 = Math::LocalToWorld(wCap, localTip1);

    XMVECTOR axis = XMVectorSubtract(tip1, tip0);
    float    lenSq = XMVectorGetX(XMVector3LengthSq(axis));
    if (lenSq < 1e-8f) return std::nullopt;               // degenerate capsule

    XMVECTOR axisNorm = XMVectorScale(axis, 1.0f / std::sqrt(lenSq));
    XMVECTOR cylP0 = XMVectorAdd(tip0, XMVectorScale(axisNorm, r)); // tip0 + r
    XMVECTOR cylP1 = XMVectorAdd(tip1, XMVectorScale(axisNorm, -r)); // tip1 - r
    XMVECTOR cylDir = XMVectorSubtract(cylP1, cylP0);
    float    cylLenSq = XMVectorGetX(XMVector3LengthSq(cylDir));

    // ------------------------------------------------------------------
    // 3. Pick sphere centre = closest point on contracted axis to box
    // ------------------------------------------------------------------
    XMVECTOR boxCenter = boxT.GetPosition(1);

    float t = 0.0f;
    if (cylLenSq > 1e-8f)
    {
        t = XMVectorGetX(XMVector3Dot(XMVectorSubtract(boxCenter, cylP0), cylDir)) / cylLenSq;
        t = std::clamp(t, 0.0f, 1.0f);
    }
    XMVECTOR sphereCenter = XMVectorAdd(cylP0, XMVectorScale(cylDir, t));

    // ------------------------------------------------------------------
    // 4. Sphere–vs–box test (reuse existing helpers)
    // ------------------------------------------------------------------
    const XMVECTOR closestPt = box->closestPoint(&boxT, sphereCenter);
    const bool     inside = box->containsPoint(&boxT, sphereCenter);

    XMVECTOR delta = XMVectorSubtract(closestPt, sphereCenter);
    const float dist = XMVectorGetX(XMVector3Length(delta));

    if (dist >= r && !inside) return std::nullopt;

    CollisionManifold m;
    m.objectA = capsuleObj;
    m.objectB = boxObj;

    // ------------------------------------------------------------------
    // 5. Build contact info
    // ------------------------------------------------------------------
    if (inside)
    {
        const auto faceNormals = box->getFaceNormals(&boxT);
        const XMVECTOR half = box->getHalfSize();
        XMVECTOR bestN = XMVectorZero();   float bestPen = -FLT_MAX;

        for (const XMVECTOR& nAxis : faceNormals)
        {
            for (int s = -1; s <= 1; s += 2)
            {
                XMVECTOR n = XMVectorScale(nAxis, float(s));
                XMVECTOR fPt = XMVectorAdd(boxCenter, XMVectorMultiply(n, half));

                float pen = r + XMVectorGetX(
                    XMVector3Dot(n, XMVectorSubtract(sphereCenter, fPt)));
                if (pen > bestPen) { bestPen = pen; bestN = n; }
            }
        }
        m.contacts.push_back({ closestPt,
                               XMVector3Normalize(bestN),
                               bestPen });
    }
    else
    {
        XMVECTOR n = (dist > 1e-6f) ? XMVectorScale(delta, 1.0f / dist)
            : XMVectorSet(0, 1, 0, 0);
        float    penetration = r - dist;

        m.contacts.push_back({ closestPt, n, penetration });
    }


//	Log::Info() << "BoxVsCasule: Collision detected with distance: " << dist << " at " <<
//		XMVectorGetX(m.contacts[0].position) << ", " <<
//		XMVectorGetY(m.contacts[0].position) << ", " <<
//		XMVectorGetZ(m.contacts[0].position) << " with normal: " <<
//		XMVectorGetX(m.contacts[0].normal) << ", " <<
//		XMVectorGetY(m.contacts[0].normal) << ", " <<
//		XMVectorGetZ(m.contacts[0].normal) << " and penetration: " << m.contacts[0].penetration << std::endl;


    // ------------------------------------------------------------------
    // 6. Flip?
    // ------------------------------------------------------------------
    if (flip)
    {
        std::swap(m.objectA, m.objectB);
        for (auto& c : m.contacts)
            c.normal = XMVectorNegate(c.normal);
    }
    return m;
}
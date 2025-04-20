#include "CapsuleCollider.h"
#include <Helpers.h>

CapsuleCollider::CapsuleCollider(const float& radius, const float& cylinderHeight)
    : m_radius(radius), m_cylinderHeight(cylinderHeight)
{
    // Ensure valid dimensions
    if (m_radius < 0) m_radius = 0;
    if (m_cylinderHeight < 0) m_cylinderHeight = 0;
}

ColliderType CapsuleCollider::getType() const
{
    return ColliderType::Capsule;
}

void CapsuleCollider::getLocalSegmentEndpoints(DirectX::XMVECTOR& out_p1, DirectX::XMVECTOR& out_p2) const
{
    using namespace DirectX;
    float halfCylinderHeight = m_cylinderHeight * 0.5f;
    // Capsule along local Y-axis
    out_p1 = XMVectorSet(0.0f, -halfCylinderHeight, 0.0f, 0.0f);
    out_p2 = XMVectorSet(0.0f, halfCylinderHeight, 0.0f, 0.0f);
}


AABB CapsuleCollider::getWorldAABB(Transform* transform) const
{
    using namespace DirectX;

    // Get the local-space endpoints of the capsule's central segment
    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    // Transform the endpoints to world space
    XMMATRIX worldMatrix = transform->GetWorldMatrix(1); // Assuming state 1 is the current state
    XMVECTOR worldP1 = Math::LocalToWorld(worldMatrix, localP1);
    XMVECTOR worldP2 = Math::LocalToWorld(worldMatrix, localP2);

    // The AABB of the capsule in world space is the AABB of the world segment
    // plus the radius extent in all directions.
    // A simplified approach is to take the AABB of the two points and expand by radius.
    // A more accurate approach considers the orientation and radius, but expanding
    // the segment AABB by the radius is a good approximation for AABB broadphase.

    XMVECTOR minPoint = XMVectorMin(worldP1, worldP2);
    XMVECTOR maxPoint = XMVectorMax(worldP1, worldP2);

    XMVECTOR radiusExtent = XMVectorReplicate(m_radius);

    return AABB(
        XMVectorSubtract(minPoint, radiusExtent),
        XMVectorAdd(maxPoint, radiusExtent)
    );

    // Note: A more precise AABB calculation involves considering the rotated
    // extent of the capsule. However, expanding the segment AABB by the radius
    // is often sufficient for broadphase checks and simpler to implement.
}


DirectX::XMMATRIX CapsuleCollider::getInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInertiaTensor) {
        // Formula for solid capsule inertia tensor along its local axes (here assumed Y-axis)
        // Capsule is a cylinder of height h and radius r, plus two hemispheres of radius r.
        // Total mass M = mass.
        float r = m_radius;
        float h = m_cylinderHeight; // Height of cylindrical part
        float H = h * 0.5f; // Half-height of cylindrical part

        // Calculate mass distribution assuming uniform density
        float cylinderVolume = XM_PI * r * r * h;
        float hemisphereVolume = (2.0f / 3.0f) * XM_PI * r * r * r; // Volume of one hemisphere
        float totalVolume = cylinderVolume + 2.0f * hemisphereVolume; // Volume of cylinder + 2 hemispheres

        // Avoid division by zero or very small numbers
        if (totalVolume < 1e-6f) {
            m_inertiaTensor = XMMatrixIdentity(); // Or scaling(0) depending on desired behavior
            m_calculatedInertiaTensor = true;
            return m_inertiaTensor;
        }

        float massPerVolume = mass / totalVolume;
        float massCylinder = massPerVolume * cylinderVolume;
        float massHemisphere = massPerVolume * hemisphereVolume; // Mass of one hemisphere

        // Inertia Tensor components (assuming local Y-axis is the capsule axis)
        // Iyy (along the capsule's axis)
        float Iyy_cylinder = 0.5f * massCylinder * r * r;
        // Inertia of a hemisphere about an axis through its center, parallel to flat base: (2/5)*m*r^2
        // Use parallel axis theorem to shift to capsule center (distance H)
        float Iyy_hemisphere = (2.0f / 5.0f) * massHemisphere * r * r + massHemisphere * H * H;
        float Iyy = Iyy_cylinder + 2.0f * Iyy_hemisphere; // Two hemispheres

        // Ixx and Izz (perpendicular to the capsule's axis)
        float Ixx_cylinder = (1.0f / 12.0f) * massCylinder * (3.0f * r * r + h * h); // Use full cylinder height h
        // Inertia of a hemisphere about an axis through its center, perpendicular to flat base: (2/5)*m*r^2
        // Use parallel axis theorem to shift to capsule center (distance H)
        float Ixx_hemisphere = (2.0f / 5.0f) * massHemisphere * r * r + massHemisphere * H * H;
        float Ixx = Ixx_cylinder + 2.0f * Ixx_hemisphere; // Two hemispheres
        float Izz = Ixx; // Symmetric

        m_inertiaTensor = XMMatrixScaling(Ixx, Iyy, Izz);
        m_calculatedInertiaTensor = true;
    }
    return m_inertiaTensor;
}

DirectX::XMMATRIX CapsuleCollider::getInverseInertiaTensor(float mass) {
    using namespace DirectX;
    if (!m_calculatedInverseInertiaTensor) {
        m_inverseInertiaTensor = getInertiaTensor(mass); // Ensure inertia tensor is calculated
        XMVECTOR determinant;
        m_inverseInertiaTensor = XMMatrixInverse(&determinant, m_inverseInertiaTensor);

        // Handle non-invertible matrix (e.g., zero mass)
        if (XMVectorGetX(XMVectorAbs(determinant)) < 1e-6f) {
            // Matrix is singular or close to singular.
            // This might happen for zero mass or non-positive definite tensor.
            // Return identity or zero matrix depending on how you handle this case.
            // Identity represents infinite inertia (static-like behavior for rotations).
            m_inverseInertiaTensor = XMMatrixIdentity(); // Or XMMatrixScaling(0,0,0)
        }

        m_calculatedInverseInertiaTensor = true;
    }
    return m_inverseInertiaTensor;
}

DirectX::XMVECTOR CapsuleCollider::support(Transform* transform, const DirectX::XMVECTOR& direction) const {
    using namespace DirectX;

    // Transform direction to local space (rotation only)
    // Inverse world rotation is the transpose of the world rotation matrix if it's pure rotation
    XMMATRIX worldRotation = XMMatrixRotationQuaternion(transform->GetRotationQuaternion(1)); // Assuming state 1
    XMMATRIX invWorldRotation = XMMatrixTranspose(worldRotation); // For pure rotation matrix
    XMVECTOR localDir = XMVector3TransformNormal(direction, invWorldRotation);

    // Normalize the local direction (important for support mapping)
    localDir = XMVector3Normalize(localDir);

    // Get local segment endpoints
    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    // The support point in local space is the endpoint of the segment
    // that is furthest in the direction of localDir, plus the radius
    // distance along localDir.

    float dot1 = XMVectorGetX(XMVector3Dot(localP1, localDir));
    float dot2 = XMVectorGetX(XMVector3Dot(localP2, localDir));

    XMVECTOR localSupportPoint;
    if (dot1 > dot2) {
        localSupportPoint = localP1;
    }
    else {
        localSupportPoint = localP2;
    }

    // Extend by radius along the local direction
    localSupportPoint = XMVectorAdd(localSupportPoint, XMVectorScale(localDir, m_radius));

    // Transform the local support point back to world space
    return Math::LocalToWorld(transform->GetWorldMatrix(1), localSupportPoint);
}

DirectX::XMVECTOR CapsuleCollider::closestPoint(Transform* transform, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    // Transform the world point to local space
    XMMATRIX worldMatrix = transform->GetWorldMatrix(1); // Assuming state 1
    XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, worldMatrix);
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorldMatrix);

    // Get local segment endpoints
    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    // Find the closest point on the local segment to the local point
    // Need a helper function like Math::ClosestPointOnLineSegment(point, segment_start, segment_end)
    XMVECTOR closestPointOnSegment = Math::ClosestPointOnLineSegment(localPoint, localP1, localP2);

    // The closest point on the capsule surface is the closest point on the segment
    // plus the radius distance outwards towards the local point.
    XMVECTOR vectorToPoint = XMVectorSubtract(localPoint, closestPointOnSegment);
    XMVECTOR directionToPoint = Math::NormalizeSafe(vectorToPoint); // Normalize safely in case vectorToPoint is zero

    XMVECTOR localClosestPoint = XMVectorAdd(closestPointOnSegment, XMVectorScale(directionToPoint, m_radius));

    // Transform the local closest point back to world space
    return Math::LocalToWorld(worldMatrix, localClosestPoint);
}

bool CapsuleCollider::containsPoint(Transform* transform, const DirectX::XMVECTOR& point) const {
    using namespace DirectX;

    // Transform the world point to local space
    XMMATRIX worldMatrix = transform->GetWorldMatrix(1); // Assuming state 1
    XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, worldMatrix);
    XMVECTOR localPoint = XMVector3TransformCoord(point, invWorldMatrix);

    // Get local segment endpoints
    XMVECTOR localP1, localP2;
    getLocalSegmentEndpoints(localP1, localP2);

    // Find the squared distance from the local point to the local segment
    // Need a helper function like Math::SqrDistancePointSegment(point, segment_start, segment_end)
    float sqrDistance = Math::SqrDistancePointSegment(localPoint, localP1, localP2);

    // The point is inside the capsule if the squared distance to the segment
    // is less than or equal to the squared radius.
    return sqrDistance <= (m_radius * m_radius);
}
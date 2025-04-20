#include "DX12LibPCH.h"
#include "Mesh.h"

#include "Application.h"


using namespace DirectX;
using namespace Microsoft::WRL;

const D3D12_INPUT_ELEMENT_DESC VertexPositionNormalTexture::InputElements[] =
{
    { "POSITION",   0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
    { "NORMAL",     0, DXGI_FORMAT_R32G32B32_FLOAT, 0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
    { "TEXCOORD",   0, DXGI_FORMAT_R32G32_FLOAT,    0, D3D12_APPEND_ALIGNED_ELEMENT, D3D12_INPUT_CLASSIFICATION_PER_VERTEX_DATA, 0 },
};

Mesh::Mesh()
    : m_IndexCount(0)
{}

Mesh::~Mesh()
{
    // Allocated resources will be cleaned automatically when the pointers go out of scope.
}

void Mesh::Draw(CommandList& commandList)
{
    commandList.SetPrimitiveTopology(D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    commandList.SetVertexBuffer(0, m_VertexBuffer);
    commandList.SetIndexBuffer(m_IndexBuffer);
    commandList.DrawIndexed(m_IndexCount);
}

std::unique_ptr<Mesh> Mesh::CreateSphere(CommandList& commandList, float diameter, size_t tessellation, bool rhcoords)
{
    VertexCollection vertices;
    IndexCollection indices;

    if (tessellation < 3)
        throw std::out_of_range("tessellation parameter out of range");

    float radius = diameter / 2.0f;
    size_t verticalSegments = tessellation;
    size_t horizontalSegments = tessellation * 2;

    // Create rings of vertices at progressively higher latitudes.
    for (size_t i = 0; i <= verticalSegments; i++)
    {
        float v = 1 - (float)i / verticalSegments;

        float latitude = (i * XM_PI / verticalSegments) - XM_PIDIV2;
        float dy, dxz;

        XMScalarSinCos(&dy, &dxz, latitude);

        // Create a single ring of vertices at this latitude.
        for (size_t j = 0; j <= horizontalSegments; j++)
        {
            float u = (float)j / horizontalSegments;

            float longitude = j * XM_2PI / horizontalSegments;
            float dx, dz;

            XMScalarSinCos(&dx, &dz, longitude);

            dx *= dxz;
            dz *= dxz;

            XMVECTOR normal = XMVectorSet(dx, dy, dz, 0);
            XMVECTOR textureCoordinate = XMVectorSet(u, v, 0, 0);

            vertices.push_back(VertexPositionNormalTexture(normal * radius, normal, textureCoordinate));
        }
    }

    // Fill the index buffer with triangles joining each pair of latitude rings.
    size_t stride = horizontalSegments + 1;

    for (size_t i = 0; i < verticalSegments; i++)
    {
        for (size_t j = 0; j <= horizontalSegments; j++)
        {
            size_t nextI = i + 1;
            size_t nextJ = (j + 1) % stride;

            indices.push_back(static_cast<uint16_t>( i * stride + j ));
            indices.push_back(static_cast<uint16_t>(nextI * stride + j));
            indices.push_back(static_cast<uint16_t>(i * stride + nextJ));

            indices.push_back(static_cast<uint16_t>(i * stride + nextJ));
            indices.push_back(static_cast<uint16_t>(nextI * stride + j));
            indices.push_back(static_cast<uint16_t>(nextI * stride + nextJ));
        }
    }

    // Create the primitive object.
    std::unique_ptr<Mesh> mesh(new Mesh());

    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}

std::unique_ptr<Mesh> Mesh::CreateCube(CommandList& commandList, float size, bool rhcoords)
{
    // A cube has six faces, each one pointing in a different direction.
    const int FaceCount = 6;

    static const XMVECTORF32 faceNormals[FaceCount] =
    {
        { 0,  0,  1 },
        { 0,  0, -1 },
        { 1,  0,  0 },
        { -1,  0,  0 },
        { 0,  1,  0 },
        { 0, -1,  0 },
    };

    static const XMVECTORF32 textureCoordinates[4] =
    {
        { 1, 0 },
        { 1, 1 },
        { 0, 1 },
        { 0, 0 },
    };

    VertexCollection vertices;
    IndexCollection indices;

    size /= 2;

    // Create each face in turn.
    for (int i = 0; i < FaceCount; i++)
    {
        XMVECTOR normal = faceNormals[i];

        // Get two vectors perpendicular both to the face normal and to each other.
        XMVECTOR basis = (i >= 4) ? g_XMIdentityR2 : g_XMIdentityR1;

        XMVECTOR side1 = XMVector3Cross(normal, basis);
        XMVECTOR side2 = XMVector3Cross(normal, side1);

        // Six indices (two triangles) per face.
        size_t vbase = vertices.size();
        indices.push_back(static_cast<uint16_t>(vbase + 0));
        indices.push_back(static_cast<uint16_t>(vbase + 1));
        indices.push_back(static_cast<uint16_t>(vbase + 2));

        indices.push_back(static_cast<uint16_t>(vbase + 0));
        indices.push_back(static_cast<uint16_t>(vbase + 2));
        indices.push_back(static_cast<uint16_t>(vbase + 3));

        // Four vertices per face.
        vertices.push_back(VertexPositionNormalTexture((normal - side1 - side2) * size, normal, textureCoordinates[0]));
        vertices.push_back(VertexPositionNormalTexture((normal - side1 + side2) * size, normal, textureCoordinates[1]));
        vertices.push_back(VertexPositionNormalTexture((normal + side1 + side2) * size, normal, textureCoordinates[2]));
        vertices.push_back(VertexPositionNormalTexture((normal + side1 - side2) * size, normal, textureCoordinates[3]));
    }

    // Create the primitive object.
    std::unique_ptr<Mesh> mesh(new Mesh());

    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}

// Helper computes a point on a unit circle, aligned to the x/z plane and centered on the origin.
static inline XMVECTOR GetCircleVector(size_t i, size_t tessellation)
{
    float angle = i * XM_2PI / tessellation;
    float dx, dz;

    XMScalarSinCos(&dx, &dz, angle);

    XMVECTORF32 v = { dx, 0, dz, 0 };
    return v;
}

static inline XMVECTOR GetCircleTangent(size_t i, size_t tessellation)
{
    float angle = (i * XM_2PI / tessellation) + XM_PIDIV2;
    float dx, dz;

    XMScalarSinCos(&dx, &dz, angle);

    XMVECTORF32 v = { dx, 0, dz, 0 };
    return v;
}

// Helper creates a triangle fan to close the end of a cylinder / cone
static void CreateCylinderCap(VertexCollection& vertices, IndexCollection& indices, size_t tessellation, float height, float radius, bool isTop)
{
    // Create cap indices.
    for (size_t i = 0; i < tessellation - 2; i++)
    {
        size_t i1 = (i + 1) % tessellation;
        size_t i2 = (i + 2) % tessellation;

        if (isTop)
        {
            std::swap(i1, i2);
        }

        size_t vbase = vertices.size();
        indices.push_back(static_cast<uint16_t>(vbase));
        indices.push_back(static_cast<uint16_t>(vbase + i1));
        indices.push_back(static_cast<uint16_t>(vbase + i2));
    }

    // Which end of the cylinder is this?
    XMVECTOR normal = g_XMIdentityR1;
    XMVECTOR textureScale = g_XMNegativeOneHalf;

    if (!isTop)
    {
        normal = -normal;
        textureScale *= g_XMNegateX;
    }

    // Create cap vertices.
    for (size_t i = 0; i < tessellation; i++)
    {
        XMVECTOR circleVector = GetCircleVector(i, tessellation);

        XMVECTOR position = (circleVector * radius) + (normal * height);

        XMVECTOR textureCoordinate = XMVectorMultiplyAdd(XMVectorSwizzle<0, 2, 3, 3>(circleVector), textureScale, g_XMOneHalf);

        vertices.push_back(VertexPositionNormalTexture(position, normal, textureCoordinate));
    }
}

std::unique_ptr<Mesh> Mesh::CreateCylinder(CommandList& commandList, float diameter, float height, size_t tessellation, bool rhcoords)
{
    VertexCollection vertices;
    IndexCollection indices;

    if (tessellation < 3)
        throw std::out_of_range("tessellation parameter out of range");

    const float radius = diameter / 2.0f;
    const float halfHeight = height / 2.0f;

    // Generate side vertices (two rings: bottom and top)
    for (size_t i = 0; i < tessellation; ++i)
    {
        const float theta = static_cast<float>(i) * XM_2PI / tessellation;
        const float cosTheta = cos(theta);
        const float sinTheta = sin(theta);

        // Bottom vertex
        XMVECTOR bottomPosition = XMVectorSet(cosTheta * radius, -halfHeight, sinTheta * radius, 0.0f);
        XMVECTOR bottomNormal = XMVectorSet(cosTheta, 0.0f, sinTheta, 0.0f);
        XMVECTOR bottomTexCoord = XMVectorSet(static_cast<float>(i) / tessellation, 0.0f, 0.0f, 0.0f);
        vertices.push_back(VertexPositionNormalTexture(bottomPosition, bottomNormal, bottomTexCoord));

        // Top vertex
        XMVECTOR topPosition = XMVectorSet(cosTheta * radius, halfHeight, sinTheta * radius, 0.0f);
        XMVECTOR topNormal = XMVectorSet(cosTheta, 0.0f, sinTheta, 0.0f);
        XMVECTOR topTexCoord = XMVectorSet(static_cast<float>(i) / tessellation, 1.0f, 0.0f, 0.0f);
        vertices.push_back(VertexPositionNormalTexture(topPosition, topNormal, topTexCoord));
    }

    // Generate side indices
    for (size_t i = 0; i < tessellation; ++i)
    {
        const size_t nextI = (i + 1) % tessellation;

        const uint16_t bottomIndex = static_cast<uint16_t>(2 * i);
        const uint16_t topIndex = static_cast<uint16_t>(2 * i + 1);
        const uint16_t nextBottomIndex = static_cast<uint16_t>(2 * nextI);
        const uint16_t nextTopIndex = static_cast<uint16_t>(2 * nextI + 1);

        // Quad triangle 1 (bottom, nextBottom, top)
        indices.push_back(bottomIndex);
        indices.push_back(nextBottomIndex);
        indices.push_back(topIndex);

        // Quad triangle 2 (top, nextBottom, nextTop)
        indices.push_back(topIndex);
        indices.push_back(nextBottomIndex);
        indices.push_back(nextTopIndex);
    }

    // Create bottom cap
    {
        // Center vertex
        XMVECTOR centerPosition = XMVectorSet(0.0f, -halfHeight, 0.0f, 0.0f);
        XMVECTOR centerNormal = XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
        XMVECTOR centerTexCoord = XMVectorSet(0.5f, 0.5f, 0.0f, 0.0f);
        vertices.push_back(VertexPositionNormalTexture(centerPosition, centerNormal, centerTexCoord));
        const uint16_t centerIndex = static_cast<uint16_t>(vertices.size() - 1);

        // Perimeter vertices
        for (size_t i = 0; i < tessellation; ++i)
        {
            const float theta = static_cast<float>(i) * XM_2PI / tessellation;
            const float cosTheta = cos(theta);
            const float sinTheta = sin(theta);

            XMVECTOR position = XMVectorSet(cosTheta * radius, -halfHeight, sinTheta * radius, 0.0f);
            XMVECTOR texCoord = XMVectorSet(cosTheta * 0.5f + 0.5f, sinTheta * 0.5f + 0.5f, 0.0f, 0.0f);
            vertices.push_back(VertexPositionNormalTexture(position, centerNormal, texCoord));
        }

        // Cap indices
        for (size_t i = 0; i < tessellation; ++i)
        {
            const uint16_t i1 = static_cast<uint16_t>(centerIndex + 1 + i);
            const uint16_t i2 = static_cast<uint16_t>(centerIndex + 1 + ((i + 1) % tessellation));

            indices.push_back(centerIndex);
            indices.push_back(i2);
            indices.push_back(i1);
        }
    }

    // Create top cap
    {
        // Center vertex
        XMVECTOR centerPosition = XMVectorSet(0.0f, halfHeight, 0.0f, 0.0f);
        XMVECTOR centerNormal = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
        XMVECTOR centerTexCoord = XMVectorSet(0.5f, 0.5f, 0.0f, 0.0f);
        vertices.push_back(VertexPositionNormalTexture(centerPosition, centerNormal, centerTexCoord));
        const uint16_t centerIndex = static_cast<uint16_t>(vertices.size() - 1);

        // Perimeter vertices
        for (size_t i = 0; i < tessellation; ++i)
        {
            const float theta = static_cast<float>(i) * XM_2PI / tessellation;
            const float cosTheta = cos(theta);
            const float sinTheta = sin(theta);

            XMVECTOR position = XMVectorSet(cosTheta * radius, halfHeight, sinTheta * radius, 0.0f);
            XMVECTOR texCoord = XMVectorSet(cosTheta * 0.5f + 0.5f, sinTheta * 0.5f + 0.5f, 0.0f, 0.0f);
            vertices.push_back(VertexPositionNormalTexture(position, centerNormal, texCoord));
        }

        // Cap indices (corrected winding order for top)
        for (size_t i = 0; i < tessellation; ++i)
        {
            const uint16_t i1 = static_cast<uint16_t>(centerIndex + 1 + i);
            const uint16_t i2 = static_cast<uint16_t>(centerIndex + 1 + ((i + 1) % tessellation));

            indices.push_back(centerIndex);
            indices.push_back(i1);
            indices.push_back(i2);
        }
    }

    // Create the mesh
    std::unique_ptr<Mesh> mesh(new Mesh());
    mesh->Initialize(commandList, vertices, indices, rhcoords);
    return mesh;
}

std::unique_ptr<Mesh> Mesh::CreateCapsule(CommandList & commandList, float diameter, float height, size_t tessellation, bool rhcoords)
{
    VertexCollection vertices;
    IndexCollection indices;

    if (tessellation < 3)
        throw std::out_of_range("tessellation parameter must be at least 3");

    float radius = diameter / 2.0f;
    float cylinderHeight = std::max(0.0f, height - diameter);
    float halfCylinderHeight = cylinderHeight / 2.0f;

    size_t verticalSegments = tessellation;
    size_t horizontalSegments = tessellation * 2;
    if (verticalSegments % 2 != 0) verticalSegments++;
    size_t numHemisphereRings = verticalSegments / 2;

    uint16_t currentVertex = 0;
    const uint16_t stride = (uint16_t)(horizontalSegments + 1);

    // === Vertex Generation (Identical to previous version) ===

    // --- Top Hemisphere Vertices ---
    XMVECTOR topCapCenter = XMVectorSet(0.0f, halfCylinderHeight, 0.0f, 0.0f);
    XMVECTOR topPoleNormal = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
    XMVECTOR topPolePos = XMVectorAdd(topCapCenter, XMVectorScale(topPoleNormal, radius));
    vertices.push_back(VertexPositionNormalTexture(topPolePos, topPoleNormal, XMVectorSet(0.5f, 0.0f, 0.0f, 0.0f))); // V=0
    currentVertex++;
    uint16_t topPoleIndex = 0;

    for (size_t i = 1; i < numHemisphereRings; ++i) {
        float latitude = XM_PIDIV2 - (float)i * XM_PI / verticalSegments;
        float dy, dxz; XMScalarSinCos(&dy, &dxz, latitude);
        float v = (float)i / verticalSegments;
        for (size_t j = 0; j <= horizontalSegments; ++j) {
            float longitude = (float)j * XM_2PI / horizontalSegments;
            float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
            dx *= dxz; dz *= dxz;
            XMVECTOR normal = XMVectorSet(dx, dy, dz, 0.0f);
            XMVECTOR position = XMVectorAdd(topCapCenter, XMVectorScale(normal, radius));
            float u = (float)j / horizontalSegments;
            vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, v, 0.0f, 0.0f)));
            currentVertex++;
        }
    }
    float baseLatitude = 0.0f; float base_dy, base_dxz; XMScalarSinCos(&base_dy, &base_dxz, baseLatitude);
    float base_v = (float)numHemisphereRings / verticalSegments; // Should be 0.5f
    for (size_t j = 0; j <= horizontalSegments; ++j) {
        float longitude = (float)j * XM_2PI / horizontalSegments;
        float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
        dx *= base_dxz; dz *= base_dxz;
        XMVECTOR normal = XMVectorSet(dx, base_dy, dz, 0.0f);
        XMVECTOR position = XMVectorAdd(topCapCenter, XMVectorScale(normal, radius));
        float u = (float)j / horizontalSegments;
        vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, base_v, 0.0f, 0.0f)));
        currentVertex++;
    }
    uint16_t topCapBaseIndexStart = currentVertex - stride;

    // --- Cylinder Vertices ---
    uint16_t cylinderTopRingStart = 0;
    uint16_t cylinderBottomRingStart = 0;
    if (cylinderHeight > 0.0f) {
        cylinderTopRingStart = currentVertex;
        float cylinderTopV = base_v;
        for (size_t j = 0; j <= horizontalSegments; ++j) {
            float longitude = (float)j * XM_2PI / horizontalSegments;
            float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
            XMVECTOR normal = XMVectorSet(dx, 0.0f, dz, 0.0f);
            XMVECTOR position = XMVectorSet(dx * radius, halfCylinderHeight, dz * radius, 0.0f);
            float u = (float)j / horizontalSegments;
            vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, cylinderTopV, 0.0f, 0.0f)));
            currentVertex++;
        }
        cylinderBottomRingStart = currentVertex;
        float cylinderBottomV = base_v; // Keep V consistent at seam for simplicity
        for (size_t j = 0; j <= horizontalSegments; ++j) {
            float longitude = (float)j * XM_2PI / horizontalSegments;
            float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
            XMVECTOR normal = XMVectorSet(dx, 0.0f, dz, 0.0f);
            XMVECTOR position = XMVectorSet(dx * radius, -halfCylinderHeight, dz * radius, 0.0f);
            float u = (float)j / horizontalSegments;
            vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, cylinderBottomV, 0.0f, 0.0f)));
            currentVertex++;
        }
    }

    // --- Bottom Hemisphere Vertices ---
    XMVECTOR bottomCapCenter = XMVectorSet(0.0f, -halfCylinderHeight, 0.0f, 0.0f);
    uint16_t bottomCapBaseIndexStart = currentVertex;
    float bottom_base_v = base_v; // Starts at 0.5f
    for (size_t j = 0; j <= horizontalSegments; ++j) {
        float longitude = (float)j * XM_2PI / horizontalSegments;
        float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
        XMVECTOR normal = XMVectorSet(dx, 0.0f, dz, 0.0f);
        XMVECTOR position = XMVectorAdd(bottomCapCenter, XMVectorScale(normal, radius));
        float u = (float)j / horizontalSegments;
        vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, bottom_base_v, 0.0f, 0.0f)));
        currentVertex++;
    }
    for (size_t i = 1; i < numHemisphereRings; ++i) {
        float latitude = -(float)i * XM_PI / verticalSegments;
        float dy, dxz; XMScalarSinCos(&dy, &dxz, latitude);
        float v = (float)(numHemisphereRings + i) / verticalSegments; // V from 0.5 towards 1.0
        for (size_t j = 0; j <= horizontalSegments; ++j) {
            float longitude = (float)j * XM_2PI / horizontalSegments;
            float dx, dz; XMScalarSinCos(&dz, &dx, longitude);
            dx *= dxz; dz *= dxz;
            XMVECTOR normal = XMVectorSet(dx, dy, dz, 0.0f);
            XMVECTOR position = XMVectorAdd(bottomCapCenter, XMVectorScale(normal, radius));
            float u = (float)j / horizontalSegments;
            vertices.push_back(VertexPositionNormalTexture(position, normal, XMVectorSet(u, v, 0.0f, 0.0f)));
            currentVertex++;
        }
    }
    XMVECTOR bottomPoleNormal = XMVectorSet(0.0f, -1.0f, 0.0f, 0.0f);
    XMVECTOR bottomPolePos = XMVectorAdd(bottomCapCenter, XMVectorScale(bottomPoleNormal, radius));
    vertices.push_back(VertexPositionNormalTexture(bottomPolePos, bottomPoleNormal, XMVectorSet(0.5f, 1.0f, 0.0f, 0.0f))); // V=1
    uint16_t bottomPoleIndex = currentVertex;
    currentVertex++;

    // === Generate Indices (CORRECTED Top Hemisphere Winding) ===

    // --- Top Hemisphere Indices ---
    // Top cap triangle fan (Corrected Winding: pole, V(j), V(j+1))
    uint16_t firstRingStart = 1; // Index of first vertex in the first ring below the pole
    for (size_t j = 0; j < horizontalSegments; ++j)
    {
        indices.push_back(topPoleIndex);
        indices.push_back(static_cast<uint16_t>(firstRingStart + j));     // Corrected
        indices.push_back(static_cast<uint16_t>(firstRingStart + j + 1)); // Corrected
    }

    // Top hemisphere body quads (Corrected Winding)
    uint16_t ringStart = firstRingStart;
    for (size_t i = 0; i < numHemisphereRings - 1; ++i) // Up to the ring before the base
    {
        for (size_t j = 0; j < horizontalSegments; ++j)
        {
            uint16_t nextRingStart = ringStart + stride;

            // Triangle 1: (i,j), (i+1,j), (i,j+1)
            indices.push_back(ringStart + j);           // Corrected
            indices.push_back(nextRingStart + j);       // Corrected
            indices.push_back(ringStart + j + 1);       // Corrected

            // Triangle 2: (i,j+1), (i+1,j), (i+1,j+1)
            indices.push_back(ringStart + j + 1);       // Corrected
            indices.push_back(nextRingStart + j);       // Corrected
            indices.push_back(nextRingStart + j + 1);   // Corrected
        }
        ringStart += stride;
    }
    // Connect last ring to the hemisphere base ring (Corrected Winding)
    for (size_t j = 0; j < horizontalSegments; ++j)
    {
        // Triangle 1: (last_ring, j), (base_ring, j), (last_ring, j+1)
        indices.push_back(ringStart + j);               // Corrected
        indices.push_back(topCapBaseIndexStart + j);    // Corrected
        indices.push_back(ringStart + j + 1);           // Corrected

        // Triangle 2: (last_ring, j+1), (base_ring, j), (base_ring, j+1)
        indices.push_back(ringStart + j + 1);           // Corrected
        indices.push_back(topCapBaseIndexStart + j);    // Corrected
        indices.push_back(topCapBaseIndexStart + j + 1);// Corrected
    }


    // --- Cylinder Body Indices (Unchanged) ---
    if (cylinderHeight > 0.0f)
    {
        for (size_t j = 0; j < horizontalSegments; ++j)
        {
            uint16_t top1 = cylinderTopRingStart + (uint16_t)j;
            uint16_t top2 = cylinderTopRingStart + (uint16_t)j + 1;
            uint16_t bottom1 = cylinderBottomRingStart + (uint16_t)j;
            uint16_t bottom2 = cylinderBottomRingStart + (uint16_t)j + 1;

            // Winding: TopLeft, BottomLeft, TopRight
            indices.push_back(top1);
            indices.push_back(bottom1);
            indices.push_back(top2);

            // Winding: TopRight, BottomLeft, BottomRight
            indices.push_back(top2);
            indices.push_back(bottom1);
            indices.push_back(bottom2);
        }
    }

    // --- Bottom Hemisphere Indices (Unchanged - Assumed correct) ---
    // Connect bottom hemisphere base ring to the next ring down
    ringStart = bottomCapBaseIndexStart; // Start of the base ring vertices for bottom cap
    uint16_t nextRingStart = ringStart + stride;
    for (size_t j = 0; j < horizontalSegments; ++j)
    {
        // Triangle 1: (base, j), (next, j), (base, j+1)
        indices.push_back(ringStart + j);
        indices.push_back(nextRingStart + j);
        indices.push_back(ringStart + j + 1);

        // Triangle 2: (base, j+1), (next, j), (next, j+1)
        indices.push_back(ringStart + j + 1);
        indices.push_back(nextRingStart + j);
        indices.push_back(nextRingStart + j + 1);
    }
    ringStart += stride;


    // Bottom hemisphere body quads (rings below base down to near pole)
    for (size_t i = 0; i < numHemisphereRings - 1; ++i)
    {
        nextRingStart = ringStart + stride;
        for (size_t j = 0; j < horizontalSegments; ++j)
        {
            // Triangle 1: (i, j), (i+1, j), (i, j+1)
            indices.push_back(ringStart + j);
            indices.push_back(nextRingStart + j);
            indices.push_back(ringStart + j + 1);

            // Triangle 2: (i, j+1), (i+1, j), (i+1, j+1)
            indices.push_back(ringStart + j + 1);
            indices.push_back(nextRingStart + j);
            indices.push_back(nextRingStart + j + 1);
        }
        ringStart += stride;
    }

    // Bottom cap triangle fan (connects last ring to bottom pole)
    // Winding: V(j), pole, V(j+1)
    for (size_t j = 0; j < horizontalSegments; ++j)
    {
        indices.push_back(ringStart + j);
        indices.push_back(bottomPoleIndex);
        indices.push_back(ringStart + j + 1);
    }


    // --- Finalize ---
    std::unique_ptr<Mesh> mesh(new Mesh());
    // Initialize will call ReverseWinding if rhcoords is false
    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}

std::unique_ptr<Mesh> Mesh::CreateCone(CommandList& commandList, float diameter, float height, size_t tessellation, bool rhcoords)
{
    VertexCollection vertices;
    IndexCollection indices;

    if (tessellation < 3)
        throw std::out_of_range("tessellation parameter out of range");

    height /= 2;

    XMVECTOR topOffset = g_XMIdentityR1 * height;

    float radius = diameter / 2;
    size_t stride = tessellation + 1;

    // Create a ring of triangles around the outside of the cone.
    for (size_t i = 0; i <= tessellation; i++)
    {
        XMVECTOR circlevec = GetCircleVector(i, tessellation);

        XMVECTOR sideOffset = circlevec * radius;

        float u = (float)i / tessellation;

        XMVECTOR textureCoordinate = XMLoadFloat(&u);

        XMVECTOR pt = sideOffset - topOffset;

        XMVECTOR normal = XMVector3Cross(GetCircleTangent(i, tessellation), topOffset - pt);
        normal = XMVector3Normalize(normal);

        // Duplicate the top vertex for distinct normals
        vertices.push_back(VertexPositionNormalTexture(topOffset, normal, g_XMZero));
        vertices.push_back(VertexPositionNormalTexture(pt, normal, textureCoordinate + g_XMIdentityR1));

        indices.push_back(static_cast<uint16_t>(i * 2));
        indices.push_back(static_cast<uint16_t>((i * 2 + 3) % (stride * 2)));
        indices.push_back(static_cast<uint16_t>((i * 2 + 1) % (stride * 2)));
    }

    // Create flat triangle fan caps to seal the bottom.
    CreateCylinderCap(vertices, indices, tessellation, height, radius, false);

    // Create the primitive object.
    std::unique_ptr<Mesh> mesh(new Mesh());

    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}

std::unique_ptr<Mesh> Mesh::CreateTorus(CommandList& commandList, float diameter, float thickness, size_t tessellation, bool rhcoords)
{
    VertexCollection vertices;
    IndexCollection indices;

    if (tessellation < 3)
        throw std::out_of_range("tessellation parameter out of range");

    size_t stride = tessellation + 1;

    // First we loop around the main ring of the torus.
    for (size_t i = 0; i <= tessellation; i++)
    {
        float u = (float)i / tessellation;

        float outerAngle = i * XM_2PI / tessellation - XM_PIDIV2;

        // Create a transform matrix that will align geometry to
        // slice perpendicularly though the current ring position.
        XMMATRIX transform = XMMatrixTranslation(diameter / 2, 0, 0) * XMMatrixRotationY(outerAngle);

        // Now we loop along the other axis, around the side of the tube.
        for (size_t j = 0; j <= tessellation; j++)
        {
            float v = 1 - (float)j / tessellation;

            float innerAngle = j * XM_2PI / tessellation + XM_PI;
            float dx, dy;

            XMScalarSinCos(&dy, &dx, innerAngle);

            // Create a vertex.
            XMVECTOR normal = XMVectorSet(dx, dy, 0, 0);
            XMVECTOR position = normal * thickness / 2;
            XMVECTOR textureCoordinate = XMVectorSet(u, v, 0, 0);

            position = XMVector3Transform(position, transform);
            normal = XMVector3TransformNormal(normal, transform);

            vertices.push_back(VertexPositionNormalTexture(position, normal, textureCoordinate));

            // And create indices for two triangles.
            size_t nextI = (i + 1) % stride;
            size_t nextJ = (j + 1) % stride;

            indices.push_back(static_cast<uint16_t>(i * stride + j));
            indices.push_back(static_cast<uint16_t>(i * stride + nextJ));
            indices.push_back(static_cast<uint16_t>(nextI * stride + j));

            indices.push_back(static_cast<uint16_t>(i * stride + nextJ));
            indices.push_back(static_cast<uint16_t>(nextI * stride + nextJ));
            indices.push_back(static_cast<uint16_t>(nextI * stride + j));
        }
    }

    // Create the primitive object.
    std::unique_ptr<Mesh> mesh(new Mesh());

    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}

std::unique_ptr<Mesh> Mesh::CreatePlane(CommandList& commandList, float width, float height, bool rhcoords)
{
    VertexCollection vertices = 
    {
        { XMFLOAT3(-0.5f * width, 0.0f,  0.5f * height), XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT2(0.0f, 0.0f) }, // 0
        { XMFLOAT3( 0.5f * width, 0.0f,  0.5f * height), XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT2(1.0f, 0.0f) }, // 1
        { XMFLOAT3( 0.5f * width, 0.0f, -0.5f * height), XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT2(1.0f, 1.0f) }, // 2
        { XMFLOAT3(-0.5f * width, 0.0f, -0.5f * height), XMFLOAT3(0.0f, 1.0f, 0.0f), XMFLOAT2(0.0f, 1.0f) }  // 3
    };
    
    IndexCollection indices = 
    {
        0, 3, 1, 1, 3, 2
    };

    std::unique_ptr<Mesh> mesh(new Mesh());

    mesh->Initialize(commandList, vertices, indices, rhcoords);

    return mesh;
}


// Helper for flipping winding of geometric primitives for LH vs. RH coords
static void ReverseWinding(IndexCollection& indices, VertexCollection& vertices)
{
    assert((indices.size() % 3) == 0);
    for (auto it = indices.begin(); it != indices.end(); it += 3)
    {
        std::swap(*it, *(it + 2));
    }

    for (auto it = vertices.begin(); it != vertices.end(); ++it)
    {
        it->textureCoordinate.x = (1.f - it->textureCoordinate.x);
    }
}

void Mesh::Initialize(CommandList& commandList, VertexCollection& vertices, IndexCollection& indices, bool rhcoords)
{
    if (vertices.size() >= USHRT_MAX)
        throw std::exception("Too many vertices for 16-bit index buffer");

    if (!rhcoords)
        ReverseWinding(indices, vertices);

    commandList.CopyVertexBuffer(m_VertexBuffer, vertices);
    commandList.CopyIndexBuffer(m_IndexBuffer, indices);

    m_IndexCount = static_cast<UINT>(indices.size());
}

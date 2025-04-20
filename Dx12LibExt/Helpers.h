#pragma once

/*
 *  Copyright(c) 2017 Jeremiah van Oosten
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files(the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions :
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *  IN THE SOFTWARE.
 */

/**
 *  @file Helpers.h
 *  @date August 28, 2017
 *  @author Jeremiah van Oosten
 *
 *  @brief Helper functions.
 */

#include <cstdint>
#include <functional>
#include <iostream>
#include <sstream>
#include <algorithm>
#include "Transform.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h> // For HRESULT
#include <d3d12.h>

// From DXSampleHelper.h 
// Source: https://github.com/Microsoft/DirectX-Graphics-Samples
inline void ThrowIfFailed(HRESULT hr)
{
    if (FAILED(hr))
    {
        std::ostringstream sout;
        sout << "HRESULT of " << std::hex << hr << " failed." << std::endl;
        OutputDebugStringA(sout.str().c_str());
        throw std::exception();
    }
}

#ifdef _DEBUG
#define FULL_SHADER_PATH(filename) (std::wstring(L"../x64/Debug/") + std::wstring(filename)).c_str()
#else
#define FULL_SHADER_PATH(filename) (std::wstring(L"../x64/Release/") + std::wstring(filename)).c_str()
#endif // _DEBUG

// Hashers for view descriptions.
namespace std
{
    // Source: https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
    template <typename T>
    inline void hash_combine(std::size_t& seed, const T& v)
    {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    template<>
    struct hash<D3D12_SHADER_RESOURCE_VIEW_DESC>
    {
        std::size_t operator()(const D3D12_SHADER_RESOURCE_VIEW_DESC& srvDesc) const noexcept
        {
            std::size_t seed = 0;

            hash_combine(seed, srvDesc.Format);
            hash_combine(seed, srvDesc.ViewDimension);
            hash_combine(seed, srvDesc.Shader4ComponentMapping);

            switch (srvDesc.ViewDimension)
            {
            case D3D12_SRV_DIMENSION_BUFFER:
                hash_combine(seed, srvDesc.Buffer.FirstElement);
                hash_combine(seed, srvDesc.Buffer.NumElements);
                hash_combine(seed, srvDesc.Buffer.StructureByteStride);
                hash_combine(seed, srvDesc.Buffer.Flags);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE1D:
                hash_combine(seed, srvDesc.Texture1D.MostDetailedMip);
                hash_combine(seed, srvDesc.Texture1D.MipLevels);
                hash_combine(seed, srvDesc.Texture1D.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE1DARRAY:
                hash_combine(seed, srvDesc.Texture1DArray.MostDetailedMip);
                hash_combine(seed, srvDesc.Texture1DArray.MipLevels);
                hash_combine(seed, srvDesc.Texture1DArray.FirstArraySlice);
                hash_combine(seed, srvDesc.Texture1DArray.ArraySize);
                hash_combine(seed, srvDesc.Texture1DArray.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE2D:
                hash_combine(seed, srvDesc.Texture2D.MostDetailedMip);
                hash_combine(seed, srvDesc.Texture2D.MipLevels);
                hash_combine(seed, srvDesc.Texture2D.PlaneSlice);
                hash_combine(seed, srvDesc.Texture2D.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE2DARRAY:
                hash_combine(seed, srvDesc.Texture2DArray.MostDetailedMip);
                hash_combine(seed, srvDesc.Texture2DArray.MipLevels);
                hash_combine(seed, srvDesc.Texture2DArray.FirstArraySlice);
                hash_combine(seed, srvDesc.Texture2DArray.ArraySize);
                hash_combine(seed, srvDesc.Texture2DArray.PlaneSlice);
                hash_combine(seed, srvDesc.Texture2DArray.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE2DMS:
//                hash_combine(seed, srvDesc.Texture2DMS.UnusedField_NothingToDefine);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE2DMSARRAY:
                hash_combine(seed, srvDesc.Texture2DMSArray.FirstArraySlice);
                hash_combine(seed, srvDesc.Texture2DMSArray.ArraySize);
                break;
            case D3D12_SRV_DIMENSION_TEXTURE3D:
                hash_combine(seed, srvDesc.Texture3D.MostDetailedMip);
                hash_combine(seed, srvDesc.Texture3D.MipLevels);
                hash_combine(seed, srvDesc.Texture3D.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURECUBE:
                hash_combine(seed, srvDesc.TextureCube.MostDetailedMip);
                hash_combine(seed, srvDesc.TextureCube.MipLevels);
                hash_combine(seed, srvDesc.TextureCube.ResourceMinLODClamp);
                break;
            case D3D12_SRV_DIMENSION_TEXTURECUBEARRAY:
                hash_combine(seed, srvDesc.TextureCubeArray.MostDetailedMip);
                hash_combine(seed, srvDesc.TextureCubeArray.MipLevels);
                hash_combine(seed, srvDesc.TextureCubeArray.First2DArrayFace);
                hash_combine(seed, srvDesc.TextureCubeArray.NumCubes);
                hash_combine(seed, srvDesc.TextureCubeArray.ResourceMinLODClamp);
                break;
            // TODO: Update Visual Studio?
            //case D3D12_SRV_DIMENSION_RAYTRACING_ACCELERATION_STRUCTURE:
            //    hash_combine(seed, srvDesc.RaytracingAccelerationStructure.Location);
            //    break;
            }

            return seed;
        }
    };

    template<>
    struct hash<D3D12_UNORDERED_ACCESS_VIEW_DESC>
    {
        std::size_t operator()(const D3D12_UNORDERED_ACCESS_VIEW_DESC& uavDesc) const noexcept
        {
            std::size_t seed = 0;

            hash_combine(seed, uavDesc.Format);
            hash_combine(seed, uavDesc.ViewDimension);

            switch (uavDesc.ViewDimension)
            {
            case D3D12_UAV_DIMENSION_BUFFER:
                hash_combine(seed, uavDesc.Buffer.FirstElement);
                hash_combine(seed, uavDesc.Buffer.NumElements);
                hash_combine(seed, uavDesc.Buffer.StructureByteStride);
                hash_combine(seed, uavDesc.Buffer.CounterOffsetInBytes);
                hash_combine(seed, uavDesc.Buffer.Flags);
                break;
            case D3D12_UAV_DIMENSION_TEXTURE1D:
                hash_combine(seed, uavDesc.Texture1D.MipSlice);
                break;
            case D3D12_UAV_DIMENSION_TEXTURE1DARRAY:
                hash_combine(seed, uavDesc.Texture1DArray.MipSlice);
                hash_combine(seed, uavDesc.Texture1DArray.FirstArraySlice);
                hash_combine(seed, uavDesc.Texture1DArray.ArraySize);
                break;
            case D3D12_UAV_DIMENSION_TEXTURE2D:
                hash_combine(seed, uavDesc.Texture2D.MipSlice);
                hash_combine(seed, uavDesc.Texture2D.PlaneSlice);
                break;
            case D3D12_UAV_DIMENSION_TEXTURE2DARRAY:
                hash_combine(seed, uavDesc.Texture2DArray.MipSlice);
                hash_combine(seed, uavDesc.Texture2DArray.FirstArraySlice);
                hash_combine(seed, uavDesc.Texture2DArray.ArraySize);
                hash_combine(seed, uavDesc.Texture2DArray.PlaneSlice);
                break;
            case D3D12_UAV_DIMENSION_TEXTURE3D:
                hash_combine(seed, uavDesc.Texture3D.MipSlice);
                hash_combine(seed, uavDesc.Texture3D.FirstWSlice);
                hash_combine(seed, uavDesc.Texture3D.WSize);
                break;
            }

            return seed;
        }
    };
}

namespace Math
{
    constexpr float PI = 3.1415926535897932384626433832795f;
    constexpr float _2PI = 2.0f * PI;
    // Convert radians to degrees.
    constexpr float Degrees(const float radians)
    {
        return radians * (180.0f / PI);
    }

    // Convert degrees to radians.
    constexpr float Radians(const float degrees)
    {
        return degrees * (PI / 180.0f);
    }

    template<typename T>
    inline T Deadzone(T val, T deadzone)
    {
        if (std::abs(val) < deadzone)
        {
            return T(0);
        }

        return val;
    }

    // Normalize a value in the range [min - max]
    template<typename T, typename U>
    inline T NormalizeRange(U x, U min, U max)
    {
        return T(x - min) / T(max - min);
    }

    // Shift and bias a value into another range.
    template<typename T, typename U>
    inline T ShiftBias(U x, U shift, U bias)
    {
        return T(x * bias) + T(shift);
    }

    /***************************************************************************
    * These functions were taken from the MiniEngine.
    * Source code available here:
    * https://github.com/Microsoft/DirectX-Graphics-Samples/blob/master/MiniEngine/Core/Math/Common.h
    * Retrieved: January 13, 2016
    **************************************************************************/
    template <typename T>
    inline T AlignUpWithMask(T value, size_t mask)
    {
        return (T)(((size_t)value + mask) & ~mask);
    }

    template <typename T>
    inline T AlignDownWithMask(T value, size_t mask)
    {
        return (T)((size_t)value & ~mask);
    }

    template <typename T>
    inline T AlignUp(T value, size_t alignment)
    {
        return AlignUpWithMask(value, alignment - 1);
    }

    template <typename T>
    inline T AlignDown(T value, size_t alignment)
    {
        return AlignDownWithMask(value, alignment - 1);
    }

    template <typename T>
    inline bool IsAligned(T value, size_t alignment)
    {
        return 0 == ((size_t)value & (alignment - 1));
    }

    template <typename T>
    inline T DivideByMultiple(T value, size_t alignment)
    {
        return (T)((value + alignment - 1) / alignment);
    }
    /***************************************************************************/

    /**
    * Round up to the next highest power of 2.
    * @source: http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    * @retrieved: January 16, 2016
    */
    inline uint32_t NextHighestPow2(uint32_t v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;

        return v;
    }

    /**
    * Round up to the next highest power of 2.
    * @source: http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
    * @retrieved: January 16, 2016
    */
    inline uint64_t NextHighestPow2(uint64_t v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v |= v >> 32;
        v++;

        return v;
    }

    [[nodiscard]] inline DirectX::XMFLOAT3 QuaternionToEuler(const DirectX::XMVECTOR& quat) {
        using namespace DirectX;

        XMFLOAT4 q = { XMVectorGetX(quat), XMVectorGetY(quat), XMVectorGetZ(quat), XMVectorGetW(quat) };
        float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
        float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
        float roll = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f * (q.w * q.y - q.z * q.x);
        float pitch;
        if (fabs(sinp) >= 1)
            pitch = copysignf(XM_PI / 2.0f, sinp); // use 90 degrees if out of range
        else
            pitch = asinf(sinp);

        float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
        float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
        float yaw = atan2f(siny_cosp, cosy_cosp);

        return { roll, pitch, yaw };
    }

    [[nodiscard]] inline std::vector<DirectX::XMVECTOR> QuaternionToOrientation(const DirectX::XMVECTOR& quat) {
        using namespace DirectX;

        std::vector<XMVECTOR> orientation;
        orientation.resize(3);

        XMMATRIX rotationMatrix = XMMatrixRotationQuaternion(quat);

        orientation[0] = rotationMatrix.r[0]; // Local X-axis
        orientation[1] = rotationMatrix.r[1]; // Local Y-axis
        orientation[2] = rotationMatrix.r[2]; // Local Z-axis

        return orientation;
    }

    inline DirectX::XMVECTOR LocalToWorld(const DirectX::XMMATRIX& mat, const DirectX::XMVECTOR& localPoint) {
        using namespace DirectX;

        return XMVector3Transform(localPoint, mat);
    }

    inline DirectX::XMVECTOR WorldToLocal(const DirectX::XMMATRIX& mat, const DirectX::XMVECTOR& worldPoint) {
		using namespace DirectX;

		XMMATRIX invWorldMatrix = XMMatrixInverse(nullptr, mat);
		return XMVector3Transform(worldPoint, invWorldMatrix);
	}

    inline DirectX::XMVECTOR ClosestPointOnLineSegment(const DirectX::XMVECTOR& a, const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2) {
        using namespace DirectX;
        XMVECTOR seg = XMVectorSubtract(p2, p1);
        float sqrLength = XMVectorGetX(XMVector3LengthSq(seg));

        if (sqrLength < 1e-6f) { // Segment is a point or very small
            return p1;
        }

        XMVECTOR v = XMVectorSubtract(a, p1);
        float dot = XMVectorGetX(XMVector3Dot(v, seg));
        float t = dot / sqrLength;

        t = std::clamp(t, 0.0f, 1.0f); // Clamp to segment [0, 1]

        return XMVectorAdd(p1, XMVectorScale(seg, t));
    }

    inline float SqrDistancePointSegment(const DirectX::XMVECTOR& a, const DirectX::XMVECTOR& p1, const DirectX::XMVECTOR& p2) {
        using namespace DirectX;
        XMVECTOR closest = ClosestPointOnLineSegment(a, p1, p2);
        XMVECTOR diff = XMVectorSubtract(a, closest);
        return XMVectorGetX(XMVector3LengthSq(diff));
    }

    inline DirectX::XMVECTOR NormalizeSafe(const DirectX::XMVECTOR& vec) {
        using namespace DirectX;
        XMVECTOR lengthSq = XMVector3LengthSq(vec);
        if (XMVectorGetX(lengthSq) < 1e-12f) { // Use a small epsilon
            return XMVectorZero();
        }
        return XMVector3Normalize(vec);
    }

}

#define STR1(x) #x
#define STR(x) STR1(x)
#define WSTR1(x) L##x
#define WSTR(x) WSTR1(x)
#define NAME_D3D12_OBJECT(x) x->SetName( WSTR(__FILE__ "(" STR(__LINE__) "): " L#x) )
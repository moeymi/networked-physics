#pragma once
#include "pch.h"
#include <array>
#include <atomic>

template<typename T, size_t Capacity>
class RingBufferSPSC
{
public:
    bool push(const T& v) noexcept
    {
        const auto h = _head.load(std::memory_order_relaxed);
        const auto n = (h + 1) % Capacity;
        if (n == _tail.load(std::memory_order_acquire))   // full
            return false;
        _data[h] = v;
        _head.store(n, std::memory_order_release);
        return true;
    }

    bool pop(T& out) noexcept
    {
        const auto t = _tail.load(std::memory_order_relaxed);
        if (t == _head.load(std::memory_order_acquire))   // empty
            return false;
        out = _data[t];
        _tail.store((t + 1) % Capacity, std::memory_order_release);
        return true;
    }

private:
    std::array<T, Capacity>        _data;
    std::atomic<size_t>            _head{ 0 };
    std::atomic<size_t>            _tail{ 0 };
};

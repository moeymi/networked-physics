#pragma once
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace ThreadAffinity {
    void SetThreadAffinity(std::thread& thread, int coreId) {
#ifdef _WIN32
        DWORD_PTR mask = 1ull << coreId;
        SetThreadAffinityMask(thread.native_handle(), mask);
#else
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(coreId, &cpuset);
        pthread_setaffinity_np(thread.native_handle(), sizeof(cpu_set_t), &cpuset);
#endif
    }
}
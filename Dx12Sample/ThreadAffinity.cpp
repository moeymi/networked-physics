#include "ThreadAffinity.h"

void ThreadAffinity::SetAffinity(std::thread& thread, int coreId) {
    {
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
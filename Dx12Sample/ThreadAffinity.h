#pragma once
#include "pch.h"
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#endif

namespace ThreadAffinity {
    void SetAffinity(std::thread& thread, int coreId);
}
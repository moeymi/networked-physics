#pragma once
#include "ThreadedSystem.h"
#include <atomic>
#include <thread>
#include <chrono>

ThreadedSystem::~ThreadedSystem() { stop(); }

void ThreadedSystem::setFixedTimeStep(const float& timeStep) { m_fixedTimeStep = timeStep; }

void ThreadedSystem::setAffinity(const int& coreId) { m_coreAffinity = coreId; }
int ThreadedSystem::getAffinity() const { return m_coreAffinity; }

void ThreadedSystem::start() {
    if (!m_running) {
        m_running = true;
        m_thread = std::thread([this]() { run(); });

        if (m_coreAffinity >= 0) {
            ThreadAffinity::SetAffinity(m_thread, m_coreAffinity);
        }
    }
}

void ThreadedSystem::stop() {
    m_running = false;
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void ThreadedSystem::run() {
    using clock = std::chrono::steady_clock;
    auto previousTime = clock::now();
    float accumulatedTime = 0.0f;

    while (m_running) {
        auto currentTime = clock::now();
        float realDelta = std::chrono::duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;
        accumulatedTime += realDelta;

        // Critical spiral of death prevention
        constexpr float maxAccumulation = 0.25f;  // Max 250ms buffer
        if (accumulatedTime > maxAccumulation) {
            accumulatedTime = maxAccumulation;
        }

        // Fixed update loop
        while (accumulatedTime >= m_fixedTimeStep) {
            onUpdate(m_fixedTimeStep);  // Physics simulation
            accumulatedTime -= m_fixedTimeStep;
        }

        // Calculate interpolation factor for rendering
        const float interpolationFactor = accumulatedTime / m_fixedTimeStep;

        // Optional: Sleep to maintain consistent frequency
        const float remaining = m_fixedTimeStep - accumulatedTime;
        if (remaining > 0) {
            std::this_thread::sleep_for(
                std::chrono::duration<float>(remaining)
            );
        }
    }
}
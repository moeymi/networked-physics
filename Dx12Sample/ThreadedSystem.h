#pragma once
#include "ThreadAffinity.h"
#include <atomic>
#include <thread>
#include <chrono>

class ThreadedSystem {
protected:
    std::thread m_thread;
    std::atomic<bool> m_running{ false };
    int m_coreAffinity = -1;
    float m_fixedTimeStep = 0.005f;

    virtual void onUpdate(float deltaTime) = 0;

public:
    virtual ~ThreadedSystem() { stop(); }

    void setAffinity(int coreId) { m_coreAffinity = coreId; }
    int getAffinity() const { return m_coreAffinity; }

    void start() {
        if (!m_running) {
            m_running = true;
            m_thread = std::thread([this]() { run(); });

            if (m_coreAffinity >= 0) {
                ThreadAffinity::SetThreadAffinity(m_thread, m_coreAffinity);
            }
        }
    }

    void stop() {
        m_running = false;
        if (m_thread.joinable()) {
            m_thread.join();
        }
    }

protected:
    void run() {
        using clock = std::chrono::steady_clock;
        auto previousTime = clock::now();
        float accumulatedTime = 0.0f;

        while (m_running) {
            auto currentTime = clock::now();
            float deltaTime = std::chrono::duration<float>(currentTime - previousTime).count();
            previousTime = currentTime;
            accumulatedTime += deltaTime;

            // Cap accumulated time to prevent spiral of death
            if (accumulatedTime > 0.2f) accumulatedTime = 0.2f;

            // Fixed timestep updates
            while (accumulatedTime >= m_fixedTimeStep) {
                onUpdate(m_fixedTimeStep);
                accumulatedTime -= m_fixedTimeStep;
            }

            // Yield remaining time
            std::this_thread::sleep_for(std::chrono::duration<float>(
                m_fixedTimeStep - accumulatedTime
            ));
        }
    }
};
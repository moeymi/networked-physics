#pragma once
#include "ThreadedSystem.h"
#include <atomic>
#include <thread>
#include <chrono>

ThreadedSystem::~ThreadedSystem() { stop(); }


void ThreadedSystem::setAffinity(const int& coreId) { m_coreAffinity = coreId; }
int ThreadedSystem::getAffinity() const { return m_coreAffinity; }

int ThreadedSystem::getFrequency() const { return m_frequency; }
float ThreadedSystem::getDeltaTime() const { return m_realTimeStep; }

bool ThreadedSystem::isRunning() const { return m_running; }

void ThreadedSystem::setFrequency(const int& freq) {
    m_frequency = freq;
    m_fixedTimeStep = 1.0f / static_cast<float>(m_frequency);
}

void ThreadedSystem::start() {
    if (!m_running) {
        m_running = true;
		m_fixedTimeStep = 1.0f / static_cast<float>(m_frequency);
		onStart();
        m_thread = std::thread([this]() { run(); });

        if (m_coreAffinity >= 0) {
            ThreadAffinity::SetAffinity(m_thread, m_coreAffinity);
        }
    }
}

void ThreadedSystem::stop() {
    m_running = false;
	if (m_thread.joinable()) {
		onStop();
	}
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

void ThreadedSystem::run() {
    using clock = std::chrono::steady_clock;
    constexpr int maxUpdatesPerCycle = 5;

    auto previousTime = clock::now();
	auto realPreviousTime = previousTime;
    float accumulatedTime = 0.0f;

    while (m_running) {
        auto currentTime = clock::now();
        float frameTime = std::chrono::duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;

        frameTime = min(frameTime, 0.25f); // clamp to avoid huge catch-ups
        accumulatedTime += frameTime;

        int updateCount = 0;
        while (accumulatedTime >= m_fixedTimeStep && updateCount < maxUpdatesPerCycle) {
            auto realCurrentTime = clock::now();
            m_realTimeStep = std::chrono::duration<float>(realCurrentTime - realPreviousTime).count();

			realPreviousTime = realCurrentTime;

            {
                std::lock_guard<std::mutex> lock(m_listenersMutex);
                for (const auto& listener : m_beforeUpdateListeners) {
                    listener(m_fixedTimeStep);
                }
            }

            onUpdate(m_fixedTimeStep);

            {
                std::lock_guard<std::mutex> lock(m_listenersMutex);
                for (const auto& listener : m_postUpdateListeners) {
                    listener(m_fixedTimeStep);
                }
            }

            accumulatedTime -= m_fixedTimeStep;
            updateCount++;
        }

        if (updateCount == maxUpdatesPerCycle) {
            // If we hit the update cap, we likely dropped frames or are running slow.
            accumulatedTime = 0.0f; // Optionally reset, or log a warning
        }

        // Sleep until the next frame
        float timeToSleep = m_fixedTimeStep - accumulatedTime;
        if (timeToSleep > 0.0f) {
            std::this_thread::sleep_for(std::chrono::duration<float>(0));
        }
    }
}

void ThreadedSystem::addBeforeUpdateListener(std::function<void(float)> listener) {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_beforeUpdateListeners.push_back(listener);
}

void ThreadedSystem::addPostUpdateListener(std::function<void(float)> listener) {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_postUpdateListeners.push_back(listener);
}

void ThreadedSystem::removeBeforeUpdateListeners() {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_beforeUpdateListeners.clear();
}

void ThreadedSystem::removePostUpdateListeners() {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_postUpdateListeners.clear();
}
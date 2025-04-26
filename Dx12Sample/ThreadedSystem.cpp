#pragma once
#include "ThreadedSystem.h"
#include <atomic>
#include <thread>
#include <chrono>

ThreadedSystem::~ThreadedSystem() { stop(); }

void ThreadedSystem::setFixedTimeStep(const float& timeStep) { m_fixedTimeStep = timeStep; }

void ThreadedSystem::setAffinity(const int& coreId) { m_coreAffinity = coreId; }
int ThreadedSystem::getAffinity() const { return m_coreAffinity; }

float ThreadedSystem::getFixedTimeStep() const { return m_fixedTimeStep; }
float ThreadedSystem::getRealTimeStep() const { return m_realTimeStep; }

bool ThreadedSystem::isRunning() const { return m_running; }

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
        auto delta = std::chrono::duration<float>(currentTime - previousTime).count();
        previousTime = currentTime;

        // Accumulate the elapsed real time
        accumulatedTime += delta;

        const float MAX_ACCUMULATED_TIME = 1; // cap at 1 second
        if (accumulatedTime > MAX_ACCUMULATED_TIME) {
            accumulatedTime = MAX_ACCUMULATED_TIME;
        }

        // --- Modified Physics Update Logic ---
        if (m_running && accumulatedTime >= m_fixedTimeStep) {

            m_realTimeStep = delta;
            onUpdate(accumulatedTime);

            {
                std::lock_guard<std::mutex> lock(m_listenersMutex);
                for (const auto& listener : m_updateListeners) {
                    listener(delta);
                }
            }

            accumulatedTime = 0.0f;
        }
    }
}

void ThreadedSystem::addUpdateListener(std::function<void(float)> listener) {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_updateListeners.push_back(listener);
}

void ThreadedSystem::removeUpdateListeners() {
	std::lock_guard<std::mutex> lock(m_listenersMutex);
	m_updateListeners.clear();
}
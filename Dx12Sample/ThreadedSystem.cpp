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
    if (!m_running.load()) {
        m_running = true;
		m_fixedTimeStep = 1.0f / static_cast<float>(m_frequency);
		onStart();
        m_thread = std::thread([this]() { run(); });

        if (m_coreAffinity >= 0) {
            ThreadAffinity::SetAffinity(m_thread, m_coreAffinity);
        }
    }
}

void ThreadedSystem::stop() noexcept {
	if (!m_running.load()) return;

	m_running.store(false, std::memory_order_seq_cst);
    if (m_thread.joinable()) {
        m_thread.join();
    }
    onStop();
}

void ThreadedSystem::run() {
    using clock = std::chrono::steady_clock;
    constexpr int   MAX_UPDATES = 5;
    constexpr float MAX_DELTA = 0.25f;

    auto previous = clock::now();
    auto realPrev = previous;
    double accumulator = 0.0;

    while (m_running.load(std::memory_order_relaxed)) {
        auto now = clock::now();
        double frameDt = std::chrono::duration<double>(now - previous).count();
        previous = now;

        frameDt = min(frameDt, double(MAX_DELTA));
        accumulator += frameDt;

        int updates = 0;
        while (accumulator >= m_fixedTimeStep && updates < MAX_UPDATES) {
            auto realNow = clock::now();
            m_realTimeStep = std::chrono::duration<double>(realNow - realPrev).count();
            realPrev = realNow;

            {
                std::lock_guard<std::mutex> lk(m_listenersMutex);
                for (auto& L : m_beforeUpdateListeners) L(m_fixedTimeStep);
            }
            onUpdate(m_fixedTimeStep);
            {
                std::lock_guard<std::mutex> lk(m_listenersMutex);
                for (auto& L : m_postUpdateListeners) L(m_fixedTimeStep);
            }

            accumulator -= m_fixedTimeStep;
            ++updates;
        }

        if (updates == MAX_UPDATES) {
            accumulator *= 0.5;
        }

        std::this_thread::yield();
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

#pragma once
#include "pch.h"
#include "ThreadAffinity.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <functional>

class ThreadedSystem {
protected:
    std::thread m_thread;
    std::atomic<bool> m_running { false };
    int m_coreAffinity = -1;
    float m_fixedTimeStep = 0.001f;
	float m_realTimeStep = 0.0f;

    std::mutex m_listenersMutex;
    std::vector<std::function<void(float)>> m_updateListeners;

    virtual void onUpdate(float deltaTime) = 0;

public:
    virtual ~ThreadedSystem();

    void setFixedTimeStep(const float& timeStep);

	bool isRunning() const;

    void setAffinity(const int& coreId);
    int getAffinity() const;

    void start();
    void stop();

	float getFixedTimeStep() const;
	float getRealTimeStep() const;

    void addUpdateListener(std::function<void(float)> listener);
    void removeUpdateListeners();

protected:
    void run();
};
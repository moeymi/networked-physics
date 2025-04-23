
#pragma once
#include "ThreadAffinity.h"
#include <atomic>
#include <thread>
#include <chrono>

class ThreadedSystem {
protected:
    std::thread m_thread;
    std::atomic<bool> m_running { false };
    int m_coreAffinity = -1;
    float m_fixedTimeStep = 0.001f;
	float m_realTimeStep = 0.0f;

    virtual void onUpdate(float deltaTime) = 0;

public:
    virtual ~ThreadedSystem();

    void setFixedTimeStep(const float& timeStep);

    void setAffinity(const int& coreId);
    int getAffinity() const;

    void start();
    void stop();

	float getFixedTimeStep() const;
	float getRealTimeStep() const;

protected:
    void run();
};
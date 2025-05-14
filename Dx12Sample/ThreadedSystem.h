
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
    int m_frequency = 60;
	float m_realTimeStep = 0.0f;
    float m_fixedTimeStep;

    std::mutex m_listenersMutex;
    std::vector<std::function<void(float)>> m_postUpdateListeners;
	std::vector<std::function<void(float)>> m_beforeUpdateListeners;

    virtual void onUpdate(float deltaTime) = 0;
	virtual void onStop() = 0;
    virtual void onStart() = 0;

public:
    virtual ~ThreadedSystem();

    int getFrequency() const;
    void setFrequency(const int& timeStep);

	bool isRunning() const;

    void setAffinity(const int& coreId);
    int getAffinity() const;

    void start();
    void stop();

	float getDeltaTime() const;

	void addBeforeUpdateListener(std::function<void(float)> listener);
    void addPostUpdateListener(std::function<void(float)> listener);

    void removeBeforeUpdateListeners();
	void removePostUpdateListeners();

protected:
    virtual void run();
};
#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <set>
#include <stdexcept>

class ThreadPool {
public:
    explicit ThreadPool(std::size_t n = std::thread::hardware_concurrency());
    ~ThreadPool();

    template<class F>
    inline void enqueue(F&& f)
    {
        {
            std::lock_guard lk(_m);
            _queue.emplace(Task{ std::forward<F>(f) });
            _pending.fetch_add(1, std::memory_order_acq_rel);
        }
        _cv.notify_one();
    }

    void wait();
private:
    struct Task { std::function<void()> fn; };

    std::vector<std::thread> _workers;
    std::set<std::thread::id> _worker_ids;
    std::queue<Task>         _queue;
    std::mutex               _m;
    std::condition_variable  _cv;
    std::atomic<std::size_t> _pending{ 0 };
    bool                     _shutdown = false;

    void worker();
};
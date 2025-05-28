#include "ThreadPool.h"
#include "Log.h"



ThreadPool::ThreadPool(std::size_t n)
{
    if (n == 0) {
        n = 1;
        if (std::thread::hardware_concurrency() == 0 && n == std::thread::hardware_concurrency()) {
            Log::Warn() << "ThreadPool: hardware_concurrency() returned 0, using 1 thread." << std::endl;
        }
        else if (n == 0) {
            Log::Info() << "ThreadPool: No threads requested, defaulting to 1 thread." << std::endl;
        }
    }
    std::size_t num_threads = n;
    if (num_threads == 0) {
        num_threads = 1;
    }


    for (std::size_t i = 0; i < num_threads; ++i) {
        _workers.emplace_back([this] { worker(); });
        _worker_ids.insert(_workers.back().get_id());
    }
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard lk(_m);
        _shutdown = true;
    }
    _cv.notify_all();
    for (auto& t : _workers) {
        if (t.joinable()) {
            t.join();
        }
    }
}

void ThreadPool::worker()
{
    for (;;)
    {
        Task task;
        {
            std::unique_lock lk(_m);
            _cv.wait(lk, [this] { return _shutdown || !_queue.empty(); });

            if (_shutdown && _queue.empty()) return;

            task = std::move(_queue.front());
            _queue.pop();
        }

        try {
            task.fn();
        }
        catch (const std::exception& e) {
            Log::Error() << "ThreadPool worker caught an exception: " << e.what() << std::endl;
        }
        catch (...) {
            Log::Error() << "ThreadPool worker caught an unknown exception." << std::endl;
        }

        {
            std::lock_guard lk(_m);
            _pending.fetch_sub(1, std::memory_order_acq_rel);
            if (_pending == 0 && _queue.empty() && !_shutdown) {
                if (_pending == 0 && _queue.empty())
                    _cv.notify_all();
            }
        }
    }
}

void ThreadPool::wait()
{
    if (_worker_ids.count(std::this_thread::get_id())) {
        throw std::logic_error("ThreadPool::wait() cannot be called from a worker thread of the same pool instance; this would cause a deadlock.");
    }

    std::unique_lock lk(_m);
    _cv.wait(lk, [this] { return _pending.load(std::memory_order_acquire) == 0 && _queue.empty(); });
}
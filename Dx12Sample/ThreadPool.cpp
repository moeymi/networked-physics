#include "ThreadPool.h"
#include "Log.h"

ThreadPool::ThreadPool(std::size_t n)
{
    for (std::size_t i = 0; i < n; ++i)
        _workers.emplace_back([this] { worker(); });
}

ThreadPool::~ThreadPool()
{
    {
        std::lock_guard lk(_m);
        _shutdown = true;
    }
    _cv.notify_all();
    for (auto& t : _workers) t.join();
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
            task.fn();                          // run user code
        }
        catch (const std::exception& e) {
            Log::Error() << "[ThreadPool] task threw: " << e.what() << '\n';
        }
        catch (...) {
            Log::Error() << "[ThreadPool] task threw unknown exception\n";
        }

        _pending.fetch_sub(1, std::memory_order_release);
        _cv.notify_all();
    }
}
void ThreadPool::wait()
{
    std::unique_lock lk(_m);
    _cv.wait(lk, [this] { return _pending.load() == 0 && _queue.empty(); });
}
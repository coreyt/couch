#ifndef THREAD_MANAGER_H
#define THREAD_MANAGER_H

#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

class ThreadManager {
public:
    ThreadManager() = default;
    ~ThreadManager();

    // Non-copyable, non-movable
    ThreadManager(const ThreadManager&) = delete;
    ThreadManager& operator=(const ThreadManager&) = delete;

    /// Launch step_fn on a background thread.
    /// Returns false if a step is already running.
    bool start_step(std::function<void()> step_fn);

    /// Returns true if no async step is running.
    bool is_complete() const;

    /// Block until the current async step finishes. No-op if none running.
    void wait();

    /// Sets cancel flag, joins worker with timeout.
    /// Returns true if joined cleanly within timeout_ms.
    bool shutdown(int timeout_ms = 5000);

    /// Check if cancellation was requested (for step functions to check).
    bool cancel_requested() const;

    /// Reset state for reuse after shutdown. Must only be called when no step is running.
    void reset();

private:
    void join_worker();

    std::thread _worker;
    std::atomic<bool> _step_complete{true};
    std::atomic<bool> _cancel_requested{false};
    std::mutex _step_mutex;
};

#endif // THREAD_MANAGER_H

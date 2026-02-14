#include "thread_manager.h"
#include <chrono>

ThreadManager::~ThreadManager() {
    shutdown(5000);
}

bool ThreadManager::start_step(std::function<void()> step_fn) {
    std::lock_guard<std::mutex> lock(_step_mutex);

    if (!_step_complete.load()) {
        return false; // step already in progress
    }

    // Join any previous worker thread
    join_worker();

    _step_complete.store(false);
    _worker = std::thread([this, fn = std::move(step_fn)]() {
        fn();
        _step_complete.store(true);
    });

    return true;
}

bool ThreadManager::is_complete() const {
    return _step_complete.load();
}

void ThreadManager::wait() {
    // Spin-wait briefly, then yield. The step function is expected to be fast
    // (sub-millisecond for typical SOFA ankle steps).
    while (!_step_complete.load()) {
        std::this_thread::yield();
    }

    std::lock_guard<std::mutex> lock(_step_mutex);
    join_worker();
}

bool ThreadManager::shutdown(int timeout_ms) {
    _cancel_requested.store(true);

    if (!_worker.joinable()) {
        return true;
    }

    // Poll for completion with timeout
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    while (!_step_complete.load()) {
        if (std::chrono::steady_clock::now() >= deadline) {
            // Timeout — worker is still running. We can't safely detach
            // because SOFA state would be corrupted, so we must join anyway.
            _worker.join();
            return false;
        }
        std::this_thread::yield();
    }

    join_worker();
    return true;
}

bool ThreadManager::cancel_requested() const {
    return _cancel_requested.load();
}

void ThreadManager::reset() {
    _cancel_requested.store(false);
    _step_complete.store(true);
}

void ThreadManager::join_worker() {
    if (_worker.joinable()) {
        _worker.join();
    }
}

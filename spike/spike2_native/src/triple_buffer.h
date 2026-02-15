#ifndef TRIPLE_BUFFER_H
#define TRIPLE_BUFFER_H

#include <atomic>
#include <cstring>

/// Lock-free triple buffer for single-writer, single-reader scenarios.
/// Writer fills writeBuffer(), calls publish(). Reader calls read().
/// No blocking, no torn reads.
template <typename T>
class TripleBuffer {
public:
    TripleBuffer() {
        std::memset(&_buffers[0], 0, sizeof(T));
        std::memset(&_buffers[1], 0, sizeof(T));
        std::memset(&_buffers[2], 0, sizeof(T));
        // Initial state: writer=0, published=1, reader=2
        _state.store(encode(0, 1, 2, false), std::memory_order_relaxed);
    }

    /// Returns a reference to the write buffer. Caller fills it, then calls publish().
    T& writeBuffer() {
        uint32_t s = _state.load(std::memory_order_relaxed);
        return _buffers[writerIndex(s)];
    }

    /// Publishes the current write buffer. Swaps writer and published indices.
    void publish() {
        uint32_t old_state, new_state;
        do {
            old_state = _state.load(std::memory_order_acquire);
            int w = writerIndex(old_state);
            int p = publishedIndex(old_state);
            int r = readerIndex(old_state);
            new_state = encode(p, w, r, true);
        } while (!_state.compare_exchange_weak(old_state, new_state,
                    std::memory_order_release, std::memory_order_relaxed));
    }

    /// Reset to initial state (no data published).
    void reset() {
        std::memset(&_buffers[0], 0, sizeof(T));
        std::memset(&_buffers[1], 0, sizeof(T));
        std::memset(&_buffers[2], 0, sizeof(T));
        _state.store(encode(0, 1, 2, false), std::memory_order_relaxed);
    }

    /// Returns true if data has been published at least once.
    bool hasData() const {
        uint32_t s = _state.load(std::memory_order_acquire);
        return hasNewData(s);
    }

    /// Returns a const reference to the latest published data.
    /// Swaps reader and published indices so next read gets fresh data.
    const T& read() {
        uint32_t old_state, new_state;
        do {
            old_state = _state.load(std::memory_order_acquire);
            int w = writerIndex(old_state);
            int p = publishedIndex(old_state);
            int r = readerIndex(old_state);
            new_state = encode(w, r, p, false);
        } while (!_state.compare_exchange_weak(old_state, new_state,
                    std::memory_order_release, std::memory_order_relaxed));
        // The old published index is now our reader index
        return _buffers[publishedIndex(old_state)];
    }

private:
    T _buffers[3];
    // Packed state: bits [0-1] writer, [2-3] published, [4-5] reader, [6] hasNewData
    std::atomic<uint32_t> _state{0};

    static constexpr uint32_t encode(int w, int p, int r, bool hasNew) {
        return static_cast<uint32_t>(w)
             | (static_cast<uint32_t>(p) << 2)
             | (static_cast<uint32_t>(r) << 4)
             | (hasNew ? (1u << 6) : 0u);
    }

    static int writerIndex(uint32_t s) { return s & 0x3; }
    static int publishedIndex(uint32_t s) { return (s >> 2) & 0x3; }
    static int readerIndex(uint32_t s) { return (s >> 4) & 0x3; }
    static bool hasNewData(uint32_t s) { return (s >> 6) & 0x1; }
};

#endif // TRIPLE_BUFFER_H

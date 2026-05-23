#pragma once
#include <atomic>
#include <array>

// AtomicParamBuffer<Params> - lock-free double-buffer for controller parameter updates.
//
// Problem:
//   A background thread (gain scheduler, auto-tuner, telemetry) needs to push
//   new parameters to a controller while the real-time thread is inside
//   compute().  A mutex would stall the RT thread; relaxed atomics allow the
//   swap to be visible without a lock.
//
// Design:
//   Two copies of Params (bufs_[0] and bufs_[1]).  active_ holds the index of
//   the current "reader" buffer.  The background writer writes to the inactive
//   copy, then atomically promotes it.
//
//   - read()    (RT thread)   : O(1), no allocation, no blocking.
//   - publish() (BG thread)   : copies Params into the inactive slot, then
//                               atomically swaps active_.
//
// Guarantees:
//   - Single writer + single reader: lock-free and data-race-free.
//   - Params must be trivially copyable (struct of doubles, ints, etc.) so
//     that write to the inactive buffer is never torn by the reader.
//   - The RT thread may observe the old or the new Params on the step that
//     publish() fires; it will always observe the new Params on the next step.
//
// Limitation:
//   Multiple concurrent writers are not supported.  Guard publish() with a
//   mutex when more than one background thread updates parameters.
//
// Usage:
//   struct PIDParams { double Kp, Ki, Kd; };
//
//   ctrl::AtomicParamBuffer<PIDParams> buf({1.0, 0.1, 0.05});
//
//   // RT thread (called at Ts):
//   const PIDParams& p = buf.read();
//   pid.setParams(p);
//
//   // Background thread (called on tuner update):
//   buf.publish({2.0, 0.2, 0.1});
namespace ctrl {

template <typename Params>
class AtomicParamBuffer {
    static_assert(std::is_trivially_copyable<Params>::value,
                  "AtomicParamBuffer requires Params to be trivially copyable. "
                  "Use only plain-old-data structs (no std::string, no pointers).");

public:
    explicit AtomicParamBuffer(const Params& initial)
        : active_(0)
    {
        bufs_[0] = initial;
        bufs_[1] = initial;
    }

    // Real-time thread: read the current parameter set.
    // Returns a const reference to the active buffer - zero copy, no allocation.
    const Params& read() const
    {
        return bufs_[active_.load(std::memory_order_acquire)];
    }

    // Background thread: publish a new parameter set.
    // Writes to the inactive buffer, then atomically makes it the active one.
    void publish(const Params& p)
    {
        const int inactive = 1 - active_.load(std::memory_order_relaxed);
        bufs_[inactive] = p;
        active_.store(inactive, std::memory_order_release);
    }

    // Peek at the last published value (background thread use only).
    const Params& latest() const
    {
        return bufs_[active_.load(std::memory_order_relaxed)];
    }

private:
    std::array<Params, 2>  bufs_;
    std::atomic<int>       active_;
};

} // namespace ctrl

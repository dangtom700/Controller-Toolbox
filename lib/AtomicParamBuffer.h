#pragma once
#include <atomic>
#include <array>
#include <cstdint>

// AtomicParamBuffer<Params> - seqlock-based double-buffer for controller parameter updates.
//
// Problem:
//   A background thread (gain scheduler, auto-tuner, telemetry) needs to push
//   new parameters to a controller while the real-time thread is inside
//   compute().  A mutex would stall the RT thread; this seqlock allows the
//   RT thread to read without blocking while the writer atomically publishes.
//
// Design - seqlock:
//   A sequence counter (seq_) is incremented twice per write: once before
//   (making it odd = "write in progress") and once after (making it even = done).
//   The reader spins until it sees an even seq_ value that is unchanged across
//   its entire read.  This is race-free under the C++ memory model.
//
//   Two Params slots (bufs_[]) are retained so the writer can prepare the next
//   value in the inactive slot before publishing, avoiding a mid-write view.
//
// Guarantees:
//   - Single writer + single reader: lock-free on the reader side.
//   - No torn reads: the reader retries if a write overlaps its read window.
//   - Params must be trivially copyable (struct of doubles, ints, etc.).
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
//   PIDParams p = buf.read();
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
        : seq_(0), active_(0)
    {
        bufs_[0] = initial;
        bufs_[1] = initial;
    }

    // Real-time thread: read the current parameter set.
    // Returns by value (copied out under seqlock protection).
    // Spins only if a publish() is mid-flight - typically zero retries.
    Params read() const
    {
        Params result;
        uint64_t s0, s1;
        do {
            s0 = seq_.load(std::memory_order_acquire);
            if (s0 & 1u) continue;  // writer in progress - spin
            result = bufs_[active_.load(std::memory_order_acquire)];
            s1 = seq_.load(std::memory_order_acquire);
        } while (s0 != s1);
        return result;
    }

    // Background thread: publish a new parameter set.
    // Writes to the inactive buffer, then atomically promotes it.
    void publish(const Params& p)
    {
        seq_.fetch_add(1, std::memory_order_release);   // seq becomes odd
        const int inactive = 1 - active_.load(std::memory_order_relaxed);
        bufs_[inactive] = p;
        active_.store(inactive, std::memory_order_release);
        seq_.fetch_add(1, std::memory_order_release);   // seq becomes even again
    }

    // Peek at the last published value (background thread use only).
    Params latest() const
    {
        return bufs_[active_.load(std::memory_order_relaxed)];
    }

private:
    std::atomic<uint64_t>  seq_;
    std::array<Params, 2>  bufs_;
    std::atomic<int>       active_;
};

} // namespace ctrl

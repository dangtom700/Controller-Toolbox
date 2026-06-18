#pragma once
#include <cstddef>

namespace ctrl_embedded {

/**
 * @brief Fixed-capacity FIFO ring buffer - no heap allocation.
 *
 * @tparam T  Element type.
 * @tparam N  Capacity (compile-time constant). Must be >= 1.
 *
 * @code
 *   RingBuffer<float, 8> buf;
 *   buf.push(1.0f);
 *   buf.push(2.0f);
 *   float v = buf.pop();  // 1.0f  (FIFO order)
 *   float p = buf.peek(0);  // 2.0f (oldest remaining)
 * @endcode
 */
template <typename T, int N>
class RingBuffer
{
    static_assert(N >= 1, "RingBuffer capacity N must be >= 1");

public:
    RingBuffer() : head_(0), tail_(0), count_(0) {}

    /** @brief Number of elements currently in the buffer. */
    int size() const { return count_; }

    /** @brief Capacity (compile-time constant N). */
    static constexpr int capacity() { return N; }

    /** @brief True if the buffer is empty. */
    bool empty() const { return count_ == 0; }

    /** @brief True if the buffer is full. */
    bool full() const { return count_ == N; }

    /**
     * @brief Push an element to the back of the queue.
     * @param value Element to push.
     * @return true if pushed successfully; false if buffer is full.
     */
    bool push(const T &value)
    {
        if (full()) return false;
        buf_[tail_] = value;
        tail_ = (tail_ + 1) % N;
        ++count_;
        return true;
    }

    /**
     * @brief Pop the oldest element from the front of the queue.
     * @param out Output reference where the element is stored.
     * @return true if popped successfully; false if buffer is empty.
     */
    bool pop(T &out)
    {
        if (empty()) return false;
        out = buf_[head_];
        head_ = (head_ + 1) % N;
        --count_;
        return true;
    }

    /**
     * @brief Non-destructive read at a given offset from the oldest element.
     * @param offset 0 = oldest, size()-1 = newest.
     * @return Element value. Undefined if offset >= size().
     */
    T peek(int offset) const
    {
        return buf_[(head_ + offset) % N];
    }

    /** @brief Remove all elements (does not zero the buffer memory). */
    void clear() { head_ = 0; tail_ = 0; count_ = 0; }

private:
    T   buf_[N];
    int head_;
    int tail_;
    int count_;
};

} // namespace ctrl_embedded

#pragma once
#include "ControllerRegistry.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <type_traits>

/**
 * @file NetworkChannel.h
 * @brief Deterministic simulation of a lossy, jittery communication link (server/PLC fusions).
 *
 * `NetworkChannel` models the transport between a master (server) and a slave (PLC): one-way
 * latency with Gaussian jitter, random packet loss, optional out-of-order delivery, and a
 * bounded in-flight queue. It exists because `lib/` had no transport abstraction at all -
 * `lib/hal/` stops at the device boundary (`ISensor`/`IActuator`/`ITimer`/`IScheduler`) and
 * `ComputationalDelayWrapper` models only a FIXED one-sample compute delay, not jitter.
 *
 * @warning This is a **simulation**, not networking. There is no socket, no fieldbus driver and
 *          no real peer. It is driven by an explicit simulation clock supplied by the caller.
 *
 * **Usage (paired master/slave links):**
 * @code
 *   ctrl::NetworkChannelParams p;
 *   p.latency_mean = 0.008;   // 8 ms one way
 *   p.jitter_sigma = 0.003;   // 3 ms std dev
 *   p.loss_prob    = 0.02;    // 2 % drop
 *   p.seed         = 7u;
 *   ctrl::NetworkChannel<double> up(p), down(p);
 *
 *   double y_rx = y_hold, u_rx = u_hold;
 *   for (int k = 0; k < N; ++k) {
 *       const double t = k * Ts;
 *       up.send(plant.output(), t);                    // slave -> master
 *       if (up.tryReceive(y_rx, t)) y_hold = y_rx;     // master takes newest, else holds
 *       down.send(ctrl_.compute(r - y_hold), t);       // master -> slave
 *       if (down.tryReceive(u_rx, t)) u_hold = u_rx;
 *       plant.step(u_hold);                            // slave holds last on starvation
 *   }
 * @endcode
 *
 * @par Latest-wins delivery
 * `tryReceive()` releases every packet whose delivery time has arrived and returns only the one
 * with the highest sequence number, discarding the rest. This is the correct semantic for
 * control telemetry - a superseded setpoint or measurement is worthless once a newer one has
 * landed - and it is what makes `allow_reorder` safe to switch on.
 *
 * @par Real-time properties
 * Zero allocation: the in-flight queue is a fixed `std::array` ring of `Capacity` slots, so
 * `send()`/`tryReceive()` never touch the heap and are safe in a hot loop
 * (`docs/deployment.md` - Zero-Allocation Checklist). Overflow drops the oldest in-flight packet
 * and counts it, rather than growing.
 *
 * @par Determinism
 * The channel owns its `std::mt19937`, seeded from `NetworkChannelParams::seed`. Two channels
 * built with the same parameters produce byte-identical delivery traces. This is deliberate:
 * seeding from `std::random_device` would make every dependent example and Catch2 guard flaky.
 * Comparing a compensated against an uncompensated control loop is only meaningful when both
 * see the *same* channel trace, so always pair the two arms on one seed.
 *
 * @see docs/server_plc_fusion_plan.md - the design this component was built for.
 * @see AtomicParamBuffer.h - the other master/slave building block (opt-in, not in the umbrella).
 */

namespace ctrl {

/** @brief Tuning parameters for NetworkChannel. */
struct NetworkChannelParams
{
    /** @brief Mean one-way latency [s]. Must be >= 0; negative values are clamped to 0. */
    double latency_mean = 0.010;

    /**
     * @brief Standard deviation of the latency jitter [s].
     *
     * Sampled latency is `max(0, N(latency_mean, jitter_sigma))` - the truncation at zero
     * enforces causality, so a packet can never be delivered before it was sent. Set to 0 for
     * a perfectly constant delay.
     */
    double jitter_sigma = 0.002;

    /** @brief Per-packet drop probability in [0, 1]. Values outside the range are clamped. */
    double loss_prob = 0.0;

    /**
     * @brief Permit out-of-order delivery.
     *
     * When @c false, each packet's delivery time is forced to be at least that of the previous
     * packet, so `lastSequence()` increases monotonically across deliveries. When @c true,
     * jitter alone decides and a later packet may overtake an earlier one; `tryReceive()` still
     * returns the newest by sequence number.
     */
    bool allow_reorder = false;

    /** @brief RNG seed. Same seed + same call sequence => identical delivery trace. */
    unsigned seed = 42u;
};

/**
 * @brief Fixed-capacity, zero-allocation simulated network link.
 *
 * @tparam T        Payload type. Copied by value into the ring; keep it small and trivially
 *                  copyable for realistic RT behaviour.
 * @tparam Capacity Maximum number of packets in flight. Overflow drops the oldest.
 */
template <typename T, int Capacity = 64>
class NetworkChannel
{
    static_assert(Capacity > 0, "NetworkChannel Capacity must be positive");
    static_assert(std::is_default_constructible_v<T>,
                  "NetworkChannel payload T must be default-constructible "
                  "(the in-flight ring is value-initialised at construction)");

public:
    /** @brief Construct with tuning parameters; the RNG is seeded here. */
    explicit NetworkChannel(const NetworkChannelParams &params = NetworkChannelParams())
        : p_(sanitise(params)), rng_(p_.seed)
    {
        reset();
    }

    /**
     * @brief Replace the link parameters mid-run without disturbing traffic or statistics.
     *
     * In-flight packets keep the delivery times they were already assigned; only packets sent
     * *after* this call see the new values. Statistics and the RNG stream are left untouched, so
     * a demo can degrade a link part-way through a run (raise `jitter_sigma`, start dropping) and
     * still compare cumulative counters across the whole run. Use `reset()` instead when you want
     * a clean, re-seeded channel.
     *
     * @note Changing `seed` here has no effect until the next `reset()` - the RNG is not re-seeded,
     *       precisely so a mid-run parameter change cannot silently replay an earlier trace.
     */
    void setParams(const NetworkChannelParams &params) { p_ = sanitise(params); }

    /**
     * @brief Queue a packet for delivery at `t_now + latency`.
     *
     * Increments `sent()` unconditionally. The packet is counted in `dropped()` and discarded
     * when it is lost to `loss_prob`, when the ring is full, or when @p value / @p t_now is
     * non-finite (the NaN contract - a poisoned sample must never reach the far end).
     *
     * @param value Payload to transmit.
     * @param t_now Current simulation time [s].
     */
    void send(const T &value, double t_now)
    {
        ++sent_;

        if (!std::isfinite(t_now) || !payloadFinite(value)) { ++dropped_; return; }

        if (p_.loss_prob > 0.0) {
            std::uniform_real_distribution<double> u(0.0, 1.0);
            if (u(rng_) < p_.loss_prob) { ++dropped_; return; }
        }

        double latency = p_.latency_mean;
        if (p_.jitter_sigma > 0.0) {
            std::normal_distribution<double> n(p_.latency_mean, p_.jitter_sigma);
            latency = n(rng_);
        }
        if (!std::isfinite(latency) || latency < 0.0) latency = 0.0;  // causality

        double t_deliver = t_now + latency;
        if (!p_.allow_reorder && t_deliver < last_scheduled_) t_deliver = last_scheduled_;
        last_scheduled_ = t_deliver;

        if (inflight_ >= Capacity) {          // ring full: drop the OLDEST, never allocate
            head_ = next(head_);
            --inflight_;
            ++dropped_;
        }

        Packet &slot = ring_[tail_];
        slot.value     = value;
        slot.t_deliver = t_deliver;
        slot.t_sent    = t_now;
        slot.seq       = next_seq_++;
        tail_ = next(tail_);
        ++inflight_;
    }

    /**
     * @brief Release every packet due by @p t_now and return the newest of them.
     *
     * Latest-wins: packets superseded by a higher sequence number in the same release batch are
     * discarded and counted in `superseded()`. `delivered()` counts only packets actually handed
     * to the caller, so `sent() == delivered() + dropped() + superseded() + inFlight()` always
     * holds.
     *
     * @param[out] out   Receives the newest due payload; untouched when nothing is due.
     * @param      t_now Current simulation time [s].
     * @return @c true if a packet was delivered, @c false on starvation (caller should hold its
     *         last value).
     */
    bool tryReceive(T &out, double t_now)
    {
        if (!std::isfinite(t_now)) return false;

        bool     got      = false;
        T        best{};
        uint32_t best_seq = 0;
        double   best_lat = 0.0;

        while (inflight_ > 0) {
            const Packet &head = ring_[head_];
            if (head.t_deliver > t_now) break;         // not due yet

            if (!got || head.seq > best_seq) {
                if (got) ++superseded_;                // the one we are replacing is discarded
                best     = head.value;
                best_seq = head.seq;
                best_lat = head.t_deliver - head.t_sent;
                got      = true;
            } else {
                ++superseded_;                         // this one arrived already superseded
            }

            head_ = next(head_);
            --inflight_;
        }

        if (got) {
            out           = best;
            last_latency_ = best_lat;
            last_seq_     = best_seq;
            ++delivered_;
        }
        return got;
    }

    /** @brief Clear all in-flight packets and statistics, and re-seed the RNG. */
    void reset()
    {
        head_ = tail_ = 0;
        inflight_ = 0;
        next_seq_ = 1;
        last_scheduled_ = -std::numeric_limits<double>::infinity();
        sent_ = delivered_ = dropped_ = superseded_ = 0;
        last_latency_ = 0.0;
        last_seq_     = 0;
        rng_.seed(p_.seed);
    }

    /** @brief Total packets handed to send(), including those later dropped. */
    unsigned sent() const { return sent_; }
    /** @brief Packets actually returned by tryReceive(). */
    unsigned delivered() const { return delivered_; }
    /** @brief Packets lost to loss_prob, ring overflow, or the NaN guard. */
    unsigned dropped() const { return dropped_; }
    /** @brief Packets discarded because a newer one superseded them in the same batch. */
    unsigned superseded() const { return superseded_; }
    /** @brief Measured latency [s] of the most recently delivered packet. */
    double lastLatency() const { return last_latency_; }
    /**
     * @brief Sequence number of the most recently delivered packet (0 before the first).
     *
     * With `allow_reorder = false` this is strictly increasing across deliveries; that is the
     * observable that distinguishes in-order from out-of-order transport.
     */
    uint32_t lastSequence() const { return last_seq_; }
    /** @brief Packets currently queued for future delivery. */
    int inFlight() const { return inflight_; }
    /** @brief Read-only access to the current parameters. */
    const NetworkChannelParams &params() const { return p_; }

private:
    struct Packet {
        T        value{};
        double   t_deliver = 0.0;
        double   t_sent    = 0.0;
        uint32_t seq       = 0;
    };

    static NetworkChannelParams sanitise(NetworkChannelParams p)
    {
        if (!std::isfinite(p.latency_mean) || p.latency_mean < 0.0) p.latency_mean = 0.0;
        if (!std::isfinite(p.jitter_sigma) || p.jitter_sigma < 0.0) p.jitter_sigma = 0.0;
        if (!std::isfinite(p.loss_prob) || p.loss_prob < 0.0) p.loss_prob = 0.0;
        if (p.loss_prob > 1.0) p.loss_prob = 1.0;
        return p;
    }

    /// Finiteness check for the payload; only meaningful for floating-point T.
    template <typename U = T>
    static bool payloadFinite(const U &v)
    {
        if constexpr (std::is_floating_point_v<U>) return std::isfinite(v);
        else                                       { (void)v; return true; }
    }

    static int next(int i) { return (i + 1 == Capacity) ? 0 : i + 1; }

    NetworkChannelParams   p_;
    std::mt19937           rng_;
    std::array<Packet, static_cast<std::size_t>(Capacity)> ring_{};

    int      head_ = 0, tail_ = 0, inflight_ = 0;
    uint32_t next_seq_ = 1;
    double   last_scheduled_ = -std::numeric_limits<double>::infinity();

    unsigned sent_ = 0, delivered_ = 0, dropped_ = 0, superseded_ = 0;
    double   last_latency_ = 0.0;
    uint32_t last_seq_     = 0;
};

} // namespace ctrl

CTRL_REGISTER_FEATURE(network_channel)

#ifndef MORPHEUS_SENSOR_CLOCK_H
#define MORPHEUS_SENSOR_CLOCK_H

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

// Converts the PSVR sensor report's 24-bit, 1 MHz device clock into the
// process-wide monotonic steady-clock domain.
//
// The class is deliberately independent of HID and pose-filter code so that it
// can be reset at each device-session boundary and tested deterministically.
// It is stateful and intended to be owned and called by one sensor thread.
class MorpheusSensorClock
{
public:
    typedef std::chrono::steady_clock Clock;
    typedef Clock::time_point TimePoint;

    struct Config
    {
        Config();

        std::size_t skew_window_size;
        std::chrono::microseconds nominal_sample_period;
        std::chrono::microseconds max_forward_gap;
        std::chrono::microseconds clock_reset_threshold;
        std::chrono::microseconds reset_holdoff;
        std::chrono::nanoseconds max_future_lead;
    };

    enum class Status
    {
        Synchronized,
        Holding,
        Duplicate,
        Discontinuity
    };

    struct Result
    {
        Result();

        Status status;
        std::size_t sample_count;
        std::array<bool, 2> sample_valid;
        std::array<TimePoint, 2> sample_timestamps;
        std::array<std::uint64_t, 2> unwrapped_ticks_us;
        std::uint32_t missing_samples;
        std::uint32_t missing_reports;
        bool tick_wrapped;
        bool clock_reset;
    };

    struct Stats
    {
        Stats();

        std::uint64_t accepted_reports;
        std::uint64_t holding_reports;
        std::uint64_t duplicate_reports;
        std::uint64_t discontinuities;
        std::uint64_t missing_samples;
        std::uint64_t missing_reports;
        std::uint64_t tick_wraps;
        std::uint64_t sequence_anomalies;
        std::uint16_t last_reported_sampling_period_us;
        std::int64_t current_skew_ns;
    };

    MorpheusSensorClock();
    explicit MorpheusSensorClock(const Config &config);

    // host_read_complete must be captured immediately after a complete 64-byte
    // HID read. reported_sampling_period_us is retained as telemetry only:
    // observed device ticks remain authoritative for sample timing.
    Result processReport(
        std::uint32_t raw_tick_0,
        std::uint32_t raw_tick_1,
        std::uint8_t packet_sequence,
        std::uint16_t reported_sampling_period_us,
        TimePoint host_read_complete);

    // Call at every open, close, HID restart, or sensors-not-ready boundary.
    void reset();

    const Stats &getStats() const;

    static std::uint32_t decodeTick24(const std::uint8_t bytes[3]);

private:
    enum class TickStep
    {
        Forward,
        Duplicate,
        Invalid
    };

    struct Timeline
    {
        Timeline();

        std::array<std::uint32_t, 2> raw_ticks;
        std::array<std::uint64_t, 2> unwrapped_ticks_us;
        std::array<bool, 2> sample_valid;
        std::uint32_t missing_samples;
        std::uint32_t wraps;
    };

    TickStep advanceTick(
        std::uint32_t from_raw_tick,
        std::uint64_t from_unwrapped_tick_us,
        std::uint32_t to_raw_tick,
        std::uint64_t &to_unwrapped_tick_us,
        std::uint32_t &missing_samples,
        bool &wrapped) const;

    bool makeInitialTimeline(
        std::uint32_t raw_tick_0,
        std::uint32_t raw_tick_1,
        Timeline &timeline) const;

    bool makeNextTimeline(
        std::uint32_t raw_tick_0,
        std::uint32_t raw_tick_1,
        Timeline &timeline) const;

    void commitTimeline(const Timeline &timeline);
    void rebaseToReport(
        std::uint32_t raw_tick_0,
        std::uint32_t raw_tick_1,
        std::uint8_t packet_sequence,
        TimePoint host_read_complete);
    Result resetForDiscontinuity(
        std::uint32_t raw_tick_0,
        std::uint32_t raw_tick_1,
        std::uint8_t packet_sequence,
        TimePoint host_read_complete);

    std::uint32_t observeSequence(std::uint8_t packet_sequence, bool duplicate);
    void pushSkewObservation(std::int64_t skew_ns);
    std::int64_t minimumSkewObservation() const;
    void clearClockState();

    static std::int64_t toNanoseconds(TimePoint time_point);
    static TimePoint fromNanoseconds(std::int64_t nanoseconds);
    static bool differenceAtLeast(
        std::int64_t lhs,
        std::int64_t rhs,
        std::int64_t threshold);

    Config m_config;
    Stats m_stats;

    bool m_has_tick;
    std::uint32_t m_last_raw_tick;
    std::uint64_t m_last_unwrapped_tick_us;

    bool m_has_report_signature;
    std::array<std::uint32_t, 2> m_last_report_signature;

    bool m_has_sequence;
    std::uint8_t m_last_sequence;

    bool m_has_host_receive;
    TimePoint m_last_host_receive;

    bool m_has_last_skew_observation;
    std::int64_t m_last_skew_observation_ns;
    std::vector<std::int64_t> m_skew_window;
    std::size_t m_skew_window_position;
    std::size_t m_skew_window_count;
    bool m_has_current_skew;
    std::int64_t m_current_skew_ns;

    bool m_holdoff_active;
    TimePoint m_holdoff_until;

    bool m_has_last_emitted_timestamp;
    TimePoint m_last_emitted_timestamp;
};

// Converts synchronized Morpheus steady-clock samples into the legacy
// high_resolution_clock domain consumed by the pose-filter API.
//
// Unlike a fresh "now minus age" conversion for every report, this bridge
// keeps a persistent clock-pair anchor. It therefore retains device sample
// spacing and monotonicity across callbacks. Since high_resolution_clock is
// not required by the standard to be monotonic, destination regressions and
// material clock-offset changes explicitly re-anchor and suppress a batch.
class MorpheusClockDomainBridge
{
public:
    typedef std::chrono::steady_clock SourceClock;
    typedef SourceClock::time_point SourceTimePoint;
    typedef std::chrono::high_resolution_clock DestinationClock;
    typedef DestinationClock::time_point DestinationTimePoint;

    struct Config
    {
        Config();

        std::chrono::milliseconds clock_reset_threshold;
        std::chrono::nanoseconds max_future_lead;
    };

    enum class Status
    {
        Synchronized,
        Holding,
        Discontinuity
    };

    struct Result
    {
        Result();

        Status status;
        std::array<bool, 2> sample_valid;
        std::array<DestinationTimePoint, 2> sample_timestamps;
        bool clock_reset;
    };

    MorpheusClockDomainBridge();
    explicit MorpheusClockDomainBridge(const Config &config);

    Result mapSamples(
        const std::array<SourceTimePoint, 2> &sample_timestamps,
        const std::array<bool, 2> &sample_valid,
        SourceTimePoint source_now,
        DestinationTimePoint destination_now);

    // Reset only while the producer callback is stopped.
    void reset();

private:
    void reanchor(
        SourceTimePoint source_now,
        DestinationTimePoint destination_now);

    Config m_config;
    bool m_has_anchor;
    SourceTimePoint m_source_anchor;
    DestinationTimePoint m_destination_anchor;

    bool m_has_last_source_now;
    SourceTimePoint m_last_source_now;
    bool m_has_last_destination_now;
    DestinationTimePoint m_last_destination_now;

    bool m_has_last_output;
    DestinationTimePoint m_last_output;
};

#endif // MORPHEUS_SENSOR_CLOCK_H

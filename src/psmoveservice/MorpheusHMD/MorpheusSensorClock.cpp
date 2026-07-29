#include "MorpheusSensorClock.h"

#include <algorithm>
#include <limits>

namespace
{
    const std::uint32_t kTickMask = 0x00ffffffu;
    const std::uint32_t kTickHalfRange = 0x00800000u;
    const std::int64_t kNanosecondsPerMicrosecond = 1000;

    std::uint32_t saturatingToUint32(std::uint64_t value)
    {
        return value > std::numeric_limits<std::uint32_t>::max()
            ? std::numeric_limits<std::uint32_t>::max()
            : static_cast<std::uint32_t>(value);
    }
}

MorpheusSensorClock::Config::Config()
    : skew_window_size(64)
    , nominal_sample_period(std::chrono::microseconds(500))
    , max_forward_gap(std::chrono::milliseconds(100))
    , clock_reset_threshold(std::chrono::milliseconds(100))
    , reset_holdoff(std::chrono::milliseconds(30))
    , max_future_lead(std::chrono::nanoseconds(0))
{
}

MorpheusSensorClock::Result::Result()
    : status(Status::Holding)
    , sample_count(0)
    , sample_valid{{false, false}}
    , sample_timestamps{{TimePoint(), TimePoint()}}
    , unwrapped_ticks_us{{0, 0}}
    , missing_samples(0)
    , missing_reports(0)
    , tick_wrapped(false)
    , clock_reset(false)
{
}

MorpheusSensorClock::Stats::Stats()
    : accepted_reports(0)
    , holding_reports(0)
    , duplicate_reports(0)
    , discontinuities(0)
    , missing_samples(0)
    , missing_reports(0)
    , tick_wraps(0)
    , sequence_anomalies(0)
    , last_reported_sampling_period_us(0)
    , current_skew_ns(0)
{
}

MorpheusSensorClock::Timeline::Timeline()
    : raw_ticks{{0, 0}}
    , unwrapped_ticks_us{{0, 0}}
    , sample_valid{{false, false}}
    , missing_samples(0)
    , wraps(0)
{
}

MorpheusSensorClock::MorpheusSensorClock()
    : MorpheusSensorClock(Config())
{
}

MorpheusSensorClock::MorpheusSensorClock(const Config &config)
    : m_config(config)
    , m_stats()
    , m_has_tick(false)
    , m_last_raw_tick(0)
    , m_last_unwrapped_tick_us(0)
    , m_has_report_signature(false)
    , m_last_report_signature{{0, 0}}
    , m_has_sequence(false)
    , m_last_sequence(0)
    , m_has_host_receive(false)
    , m_last_host_receive()
    , m_has_last_skew_observation(false)
    , m_last_skew_observation_ns(0)
    , m_skew_window()
    , m_skew_window_position(0)
    , m_skew_window_count(0)
    , m_has_current_skew(false)
    , m_current_skew_ns(0)
    , m_holdoff_active(false)
    , m_holdoff_until()
    , m_has_last_emitted_timestamp(false)
    , m_last_emitted_timestamp()
{
    if (m_config.skew_window_size == 0)
    {
        m_config.skew_window_size = 1;
    }
    if (m_config.nominal_sample_period.count() <= 0)
    {
        m_config.nominal_sample_period = std::chrono::microseconds(500);
    }
    if (m_config.max_forward_gap.count() <= 0)
    {
        m_config.max_forward_gap = std::chrono::milliseconds(100);
    }
    if (m_config.clock_reset_threshold.count() <= 0)
    {
        m_config.clock_reset_threshold = std::chrono::milliseconds(100);
    }
    if (m_config.reset_holdoff.count() < 0)
    {
        m_config.reset_holdoff = std::chrono::microseconds(0);
    }
    if (m_config.max_future_lead.count() < 0)
    {
        m_config.max_future_lead = std::chrono::nanoseconds(0);
    }

    m_skew_window.resize(m_config.skew_window_size, 0);
}

std::uint32_t MorpheusSensorClock::decodeTick24(const std::uint8_t bytes[3])
{
    return static_cast<std::uint32_t>(bytes[0])
        | (static_cast<std::uint32_t>(bytes[1]) << 8)
        | (static_cast<std::uint32_t>(bytes[2]) << 16);
}

MorpheusSensorClock::TickStep MorpheusSensorClock::advanceTick(
    std::uint32_t from_raw_tick,
    std::uint64_t from_unwrapped_tick_us,
    std::uint32_t to_raw_tick,
    std::uint64_t &to_unwrapped_tick_us,
    std::uint32_t &missing_samples,
    bool &wrapped) const
{
    from_raw_tick &= kTickMask;
    to_raw_tick &= kTickMask;

    const std::uint32_t delta = (to_raw_tick - from_raw_tick) & kTickMask;
    if (delta == 0)
    {
        to_unwrapped_tick_us = from_unwrapped_tick_us;
        missing_samples = 0;
        wrapped = false;
        return TickStep::Duplicate;
    }

    if (delta >= kTickHalfRange
        || delta > static_cast<std::uint64_t>(m_config.max_forward_gap.count()))
    {
        to_unwrapped_tick_us = from_unwrapped_tick_us;
        missing_samples = 0;
        wrapped = false;
        return TickStep::Invalid;
    }

    to_unwrapped_tick_us = from_unwrapped_tick_us + delta;
    wrapped = to_raw_tick < from_raw_tick;

    const std::uint64_t nominal_period_us =
        static_cast<std::uint64_t>(m_config.nominal_sample_period.count());
    const std::uint64_t rounded_intervals =
        (static_cast<std::uint64_t>(delta) + nominal_period_us / 2)
        / nominal_period_us;
    missing_samples = rounded_intervals > 1
        ? saturatingToUint32(rounded_intervals - 1)
        : 0;

    return TickStep::Forward;
}

bool MorpheusSensorClock::makeInitialTimeline(
    std::uint32_t raw_tick_0,
    std::uint32_t raw_tick_1,
    Timeline &timeline) const
{
    timeline = Timeline();
    timeline.raw_ticks[0] = raw_tick_0 & kTickMask;
    timeline.raw_ticks[1] = raw_tick_1 & kTickMask;
    timeline.unwrapped_ticks_us[0] = timeline.raw_ticks[0];
    timeline.sample_valid[0] = true;

    std::uint32_t missing = 0;
    bool wrapped = false;
    const TickStep step = advanceTick(
        timeline.raw_ticks[0],
        timeline.unwrapped_ticks_us[0],
        timeline.raw_ticks[1],
        timeline.unwrapped_ticks_us[1],
        missing,
        wrapped);

    if (step == TickStep::Invalid)
    {
        return false;
    }

    if (step == TickStep::Forward)
    {
        timeline.sample_valid[1] = true;
        timeline.missing_samples = missing;
        timeline.wraps = wrapped ? 1u : 0u;
    }

    return true;
}

bool MorpheusSensorClock::makeNextTimeline(
    std::uint32_t raw_tick_0,
    std::uint32_t raw_tick_1,
    Timeline &timeline) const
{
    timeline = Timeline();
    timeline.raw_ticks[0] = raw_tick_0 & kTickMask;
    timeline.raw_ticks[1] = raw_tick_1 & kTickMask;

    std::uint32_t working_raw_tick = m_last_raw_tick;
    std::uint64_t working_unwrapped_tick_us = m_last_unwrapped_tick_us;

    std::uint32_t missing = 0;
    bool wrapped = false;
    TickStep step = advanceTick(
        working_raw_tick,
        working_unwrapped_tick_us,
        timeline.raw_ticks[0],
        timeline.unwrapped_ticks_us[0],
        missing,
        wrapped);

    if (step == TickStep::Invalid)
    {
        return false;
    }
    if (step == TickStep::Forward)
    {
        timeline.sample_valid[0] = true;
        timeline.missing_samples += missing;
        timeline.wraps += wrapped ? 1u : 0u;
        working_raw_tick = timeline.raw_ticks[0];
        working_unwrapped_tick_us = timeline.unwrapped_ticks_us[0];
    }
    else
    {
        timeline.unwrapped_ticks_us[0] = working_unwrapped_tick_us;
    }

    missing = 0;
    wrapped = false;
    step = advanceTick(
        working_raw_tick,
        working_unwrapped_tick_us,
        timeline.raw_ticks[1],
        timeline.unwrapped_ticks_us[1],
        missing,
        wrapped);

    if (step == TickStep::Invalid)
    {
        return false;
    }
    if (step == TickStep::Forward)
    {
        timeline.sample_valid[1] = true;
        timeline.missing_samples += missing;
        timeline.wraps += wrapped ? 1u : 0u;
    }
    else
    {
        timeline.unwrapped_ticks_us[1] = working_unwrapped_tick_us;
    }

    return true;
}

void MorpheusSensorClock::commitTimeline(const Timeline &timeline)
{
    if (timeline.sample_valid[1])
    {
        m_last_raw_tick = timeline.raw_ticks[1];
        m_last_unwrapped_tick_us = timeline.unwrapped_ticks_us[1];
        m_has_tick = true;
    }
    else if (timeline.sample_valid[0])
    {
        m_last_raw_tick = timeline.raw_ticks[0];
        m_last_unwrapped_tick_us = timeline.unwrapped_ticks_us[0];
        m_has_tick = true;
    }
}

void MorpheusSensorClock::clearClockState()
{
    m_has_tick = false;
    m_last_raw_tick = 0;
    m_last_unwrapped_tick_us = 0;

    m_has_report_signature = false;
    m_last_report_signature = {{0, 0}};

    m_has_sequence = false;
    m_last_sequence = 0;

    m_has_host_receive = false;
    m_last_host_receive = TimePoint();

    m_has_last_skew_observation = false;
    m_last_skew_observation_ns = 0;
    std::fill(m_skew_window.begin(), m_skew_window.end(), 0);
    m_skew_window_position = 0;
    m_skew_window_count = 0;
    m_has_current_skew = false;
    m_current_skew_ns = 0;

    m_holdoff_active = false;
    m_holdoff_until = TimePoint();

    m_has_last_emitted_timestamp = false;
    m_last_emitted_timestamp = TimePoint();
}

void MorpheusSensorClock::reset()
{
    m_stats = Stats();
    clearClockState();
}

const MorpheusSensorClock::Stats &MorpheusSensorClock::getStats() const
{
    return m_stats;
}

void MorpheusSensorClock::pushSkewObservation(std::int64_t skew_ns)
{
    m_skew_window[m_skew_window_position] = skew_ns;
    m_skew_window_position =
        (m_skew_window_position + 1) % m_skew_window.size();
    if (m_skew_window_count < m_skew_window.size())
    {
        ++m_skew_window_count;
    }

    const std::int64_t minimum_skew_ns = minimumSkewObservation();
    if (!m_has_current_skew)
    {
        m_current_skew_ns = minimum_skew_ns;
        m_has_current_skew = true;
    }
    else
    {
        const std::int64_t weight =
            static_cast<std::int64_t>(m_skew_window_count);
        m_current_skew_ns =
            m_current_skew_ns
            + (minimum_skew_ns - m_current_skew_ns) / weight;
    }

    m_stats.current_skew_ns = m_current_skew_ns;
}

std::int64_t MorpheusSensorClock::minimumSkewObservation() const
{
    if (m_skew_window_count == 0)
    {
        return 0;
    }

    std::int64_t minimum_skew_ns = m_skew_window[0];
    for (std::size_t index = 1; index < m_skew_window_count; ++index)
    {
        minimum_skew_ns = std::min(minimum_skew_ns, m_skew_window[index]);
    }
    return minimum_skew_ns;
}

std::uint32_t MorpheusSensorClock::observeSequence(
    std::uint8_t packet_sequence,
    bool duplicate)
{
    if (!m_has_sequence)
    {
        m_has_sequence = true;
        m_last_sequence = packet_sequence;
        return 0;
    }

    const std::uint8_t sequence_delta =
        static_cast<std::uint8_t>(packet_sequence - m_last_sequence);

    if (duplicate)
    {
        if (sequence_delta > 0 && sequence_delta < 128)
        {
            m_last_sequence = packet_sequence;
        }
        else if (sequence_delta >= 128)
        {
            ++m_stats.sequence_anomalies;
        }
        return 0;
    }

    if (sequence_delta == 0)
    {
        ++m_stats.sequence_anomalies;
        return 0;
    }

    m_last_sequence = packet_sequence;
    if (sequence_delta < 128)
    {
        return static_cast<std::uint32_t>(sequence_delta - 1);
    }

    ++m_stats.sequence_anomalies;
    return 0;
}

void MorpheusSensorClock::rebaseToReport(
    std::uint32_t raw_tick_0,
    std::uint32_t raw_tick_1,
    std::uint8_t packet_sequence,
    TimePoint host_read_complete)
{
    clearClockState();

    Timeline timeline;
    if (makeInitialTimeline(raw_tick_0, raw_tick_1, timeline))
    {
        commitTimeline(timeline);
    }
    else
    {
        m_has_tick = true;
        m_last_raw_tick = raw_tick_1 & kTickMask;
        m_last_unwrapped_tick_us = m_last_raw_tick;
    }

    m_has_report_signature = true;
    m_last_report_signature[0] = raw_tick_0 & kTickMask;
    m_last_report_signature[1] = raw_tick_1 & kTickMask;

    m_has_sequence = true;
    m_last_sequence = packet_sequence;

    m_has_host_receive = true;
    m_last_host_receive = host_read_complete;

    const std::int64_t host_ns = toNanoseconds(host_read_complete);
    const std::int64_t remote_ns =
        static_cast<std::int64_t>(m_last_unwrapped_tick_us)
        * kNanosecondsPerMicrosecond;
    const std::int64_t skew_ns = host_ns - remote_ns;
    m_has_last_skew_observation = true;
    m_last_skew_observation_ns = skew_ns;
    pushSkewObservation(skew_ns);

    m_holdoff_active = m_config.reset_holdoff.count() > 0;
    m_holdoff_until = host_read_complete + m_config.reset_holdoff;
}

MorpheusSensorClock::Result MorpheusSensorClock::resetForDiscontinuity(
    std::uint32_t raw_tick_0,
    std::uint32_t raw_tick_1,
    std::uint8_t packet_sequence,
    TimePoint host_read_complete)
{
    ++m_stats.discontinuities;
    rebaseToReport(
        raw_tick_0,
        raw_tick_1,
        packet_sequence,
        host_read_complete);

    Result result;
    result.status = Status::Discontinuity;
    result.clock_reset = true;
    return result;
}

std::int64_t MorpheusSensorClock::toNanoseconds(TimePoint time_point)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_point.time_since_epoch()).count();
}

MorpheusSensorClock::TimePoint MorpheusSensorClock::fromNanoseconds(
    std::int64_t nanoseconds)
{
    return TimePoint(std::chrono::duration_cast<Clock::duration>(
        std::chrono::nanoseconds(nanoseconds)));
}

bool MorpheusSensorClock::differenceAtLeast(
    std::int64_t lhs,
    std::int64_t rhs,
    std::int64_t threshold)
{
    if (lhs >= rhs)
    {
        return lhs - rhs >= threshold;
    }
    return rhs - lhs >= threshold;
}

MorpheusSensorClock::Result MorpheusSensorClock::processReport(
    std::uint32_t raw_tick_0,
    std::uint32_t raw_tick_1,
    std::uint8_t packet_sequence,
    std::uint16_t reported_sampling_period_us,
    TimePoint host_read_complete)
{
    raw_tick_0 &= kTickMask;
    raw_tick_1 &= kTickMask;
    m_stats.last_reported_sampling_period_us = reported_sampling_period_us;

    if (m_has_host_receive && host_read_complete < m_last_host_receive)
    {
        return resetForDiscontinuity(
            raw_tick_0,
            raw_tick_1,
            packet_sequence,
            host_read_complete);
    }

    const bool same_report =
        m_has_report_signature
        && m_last_report_signature[0] == raw_tick_0
        && m_last_report_signature[1] == raw_tick_1;
    if (same_report)
    {
        observeSequence(packet_sequence, true);
        m_last_host_receive = host_read_complete;
        m_has_host_receive = true;
        ++m_stats.duplicate_reports;

        Result result;
        result.status = Status::Duplicate;
        return result;
    }

    const bool initializing_timeline = !m_has_tick;
    Timeline timeline;
    const bool timeline_valid = m_has_tick
        ? makeNextTimeline(raw_tick_0, raw_tick_1, timeline)
        : makeInitialTimeline(raw_tick_0, raw_tick_1, timeline);
    if (!timeline_valid)
    {
        return resetForDiscontinuity(
            raw_tick_0,
            raw_tick_1,
            packet_sequence,
            host_read_complete);
    }

    const bool has_new_sample =
        timeline.sample_valid[0] || timeline.sample_valid[1];
    if (!has_new_sample)
    {
        observeSequence(packet_sequence, true);
        m_has_report_signature = true;
        m_last_report_signature = timeline.raw_ticks;
        m_last_host_receive = host_read_complete;
        m_has_host_receive = true;
        ++m_stats.duplicate_reports;

        Result result;
        result.status = Status::Duplicate;
        return result;
    }

    const std::size_t newest_sample_index =
        timeline.sample_valid[1] ? 1u : 0u;
    const std::int64_t host_ns = toNanoseconds(host_read_complete);
    const std::int64_t newest_remote_ns =
        static_cast<std::int64_t>(
            timeline.unwrapped_ticks_us[newest_sample_index])
        * kNanosecondsPerMicrosecond;
    const std::int64_t skew_observation_ns = host_ns - newest_remote_ns;
    const std::int64_t reset_threshold_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            m_config.clock_reset_threshold).count();

    if (m_has_last_skew_observation
        && differenceAtLeast(
            skew_observation_ns,
            m_last_skew_observation_ns,
            reset_threshold_ns))
    {
        return resetForDiscontinuity(
            raw_tick_0,
            raw_tick_1,
            packet_sequence,
            host_read_complete);
    }

    const std::uint32_t missing_reports =
        observeSequence(packet_sequence, false);
    commitTimeline(timeline);
    m_has_report_signature = true;
    m_last_report_signature = timeline.raw_ticks;
    m_has_host_receive = true;
    m_last_host_receive = host_read_complete;
    m_has_last_skew_observation = true;
    m_last_skew_observation_ns = skew_observation_ns;
    pushSkewObservation(skew_observation_ns);

    if (initializing_timeline)
    {
        m_holdoff_active = m_config.reset_holdoff.count() > 0;
        m_holdoff_until = host_read_complete + m_config.reset_holdoff;
    }

    ++m_stats.accepted_reports;
    m_stats.missing_samples += timeline.missing_samples;
    m_stats.missing_reports += missing_reports;
    m_stats.tick_wraps += timeline.wraps;

    Result result;
    result.unwrapped_ticks_us = timeline.unwrapped_ticks_us;
    result.missing_samples = timeline.missing_samples;
    result.missing_reports = missing_reports;
    result.tick_wrapped = timeline.wraps > 0;

    if (m_holdoff_active)
    {
        if (host_read_complete < m_holdoff_until)
        {
            ++m_stats.holding_reports;
            result.status = Status::Holding;
            return result;
        }

        m_holdoff_active = false;
        m_current_skew_ns = minimumSkewObservation();
        m_has_current_skew = true;
        m_stats.current_skew_ns = m_current_skew_ns;
    }

    std::int64_t candidate_skew_ns = m_current_skew_ns;
    const std::int64_t maximum_causal_skew_ns =
        skew_observation_ns + m_config.max_future_lead.count();
    if (candidate_skew_ns > maximum_causal_skew_ns)
    {
        candidate_skew_ns = maximum_causal_skew_ns;
    }

    std::int64_t earliest_timestamp_ns = 0;
    bool has_earliest_timestamp = false;
    for (std::size_t index = 0; index < timeline.sample_valid.size(); ++index)
    {
        if (timeline.sample_valid[index])
        {
            const std::int64_t mapped_timestamp_ns =
                static_cast<std::int64_t>(
                    timeline.unwrapped_ticks_us[index])
                * kNanosecondsPerMicrosecond
                + candidate_skew_ns;
            if (!has_earliest_timestamp)
            {
                earliest_timestamp_ns = mapped_timestamp_ns;
                has_earliest_timestamp = true;
            }
        }
    }

    if (m_has_last_emitted_timestamp)
    {
        const std::int64_t last_emitted_ns =
            toNanoseconds(m_last_emitted_timestamp);
        if (earliest_timestamp_ns <= last_emitted_ns)
        {
            candidate_skew_ns +=
                (last_emitted_ns - earliest_timestamp_ns) + 1;
        }
    }

    const std::int64_t newest_timestamp_ns =
        newest_remote_ns + candidate_skew_ns;
    const std::int64_t latest_allowed_timestamp_ns =
        host_ns + m_config.max_future_lead.count();
    if (newest_timestamp_ns > latest_allowed_timestamp_ns)
    {
        ++m_stats.holding_reports;
        result.status = Status::Holding;
        return result;
    }

    m_current_skew_ns = candidate_skew_ns;
    m_stats.current_skew_ns = m_current_skew_ns;
    for (std::size_t index = 0; index < timeline.sample_valid.size(); ++index)
    {
        if (timeline.sample_valid[index])
        {
            result.sample_valid[index] = true;
            result.sample_timestamps[index] = fromNanoseconds(
                static_cast<std::int64_t>(
                    timeline.unwrapped_ticks_us[index])
                * kNanosecondsPerMicrosecond
                + candidate_skew_ns);
            ++result.sample_count;
        }
    }

    result.status = Status::Synchronized;
    m_has_last_emitted_timestamp = true;
    m_last_emitted_timestamp =
        result.sample_timestamps[newest_sample_index];
    return result;
}

MorpheusClockDomainBridge::Config::Config()
    : clock_reset_threshold(std::chrono::milliseconds(100))
    , max_future_lead(std::chrono::nanoseconds(0))
{
}

MorpheusClockDomainBridge::Result::Result()
    : status(Status::Holding)
    , sample_valid{{false, false}}
    , sample_timestamps{{DestinationTimePoint(), DestinationTimePoint()}}
    , clock_reset(false)
{
}

MorpheusClockDomainBridge::MorpheusClockDomainBridge()
    : MorpheusClockDomainBridge(Config())
{
}

MorpheusClockDomainBridge::MorpheusClockDomainBridge(const Config &config)
    : m_config(config)
    , m_has_anchor(false)
    , m_source_anchor()
    , m_destination_anchor()
    , m_has_last_source_now(false)
    , m_last_source_now()
    , m_has_last_destination_now(false)
    , m_last_destination_now()
    , m_has_last_output(false)
    , m_last_output()
{
    if (m_config.clock_reset_threshold.count() <= 0)
    {
        m_config.clock_reset_threshold = std::chrono::milliseconds(100);
    }
    if (m_config.max_future_lead.count() < 0)
    {
        m_config.max_future_lead = std::chrono::nanoseconds(0);
    }
}

void MorpheusClockDomainBridge::reset()
{
    m_has_anchor = false;
    m_source_anchor = SourceTimePoint();
    m_destination_anchor = DestinationTimePoint();

    m_has_last_source_now = false;
    m_last_source_now = SourceTimePoint();
    m_has_last_destination_now = false;
    m_last_destination_now = DestinationTimePoint();

    m_has_last_output = false;
    m_last_output = DestinationTimePoint();
}

void MorpheusClockDomainBridge::reanchor(
    SourceTimePoint source_now,
    DestinationTimePoint destination_now)
{
    m_has_anchor = true;
    m_source_anchor = source_now;
    m_destination_anchor = destination_now;
}

MorpheusClockDomainBridge::Result
MorpheusClockDomainBridge::mapSamples(
    const std::array<SourceTimePoint, 2> &sample_timestamps,
    const std::array<bool, 2> &sample_valid,
    SourceTimePoint source_now,
    DestinationTimePoint destination_now)
{
    Result result;

    const bool source_clock_regressed =
        m_has_last_source_now && source_now < m_last_source_now;
    const bool destination_clock_regressed =
        m_has_last_destination_now
        && destination_now < m_last_destination_now;

    bool clock_offset_discontinuous = false;
    if (m_has_anchor && !source_clock_regressed && !destination_clock_regressed)
    {
        const DestinationTimePoint expected_destination_now =
            m_destination_anchor
            + std::chrono::duration_cast<DestinationClock::duration>(
                source_now - m_source_anchor);
        const std::chrono::nanoseconds clock_offset_error =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                destination_now - expected_destination_now);
        const std::chrono::nanoseconds reset_threshold =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                m_config.clock_reset_threshold);

        clock_offset_discontinuous =
            clock_offset_error >= reset_threshold
            || clock_offset_error <= -reset_threshold;
    }

    if (!m_has_anchor)
    {
        reanchor(source_now, destination_now);
    }
    else if (source_clock_regressed
        || destination_clock_regressed
        || clock_offset_discontinuous)
    {
        // Preserve m_last_output so a backward destination-clock correction
        // cannot silently send the pose filter a regressing timestamp.
        reanchor(source_now, destination_now);
        m_has_last_source_now = true;
        m_last_source_now = source_now;
        m_has_last_destination_now = true;
        m_last_destination_now = destination_now;

        result.status = Status::Discontinuity;
        result.clock_reset = true;
        return result;
    }

    m_has_last_source_now = true;
    m_last_source_now = source_now;
    m_has_last_destination_now = true;
    m_last_destination_now = destination_now;

    bool has_sample = false;
    std::size_t earliest_sample_index = 0;
    std::size_t latest_sample_index = 0;
    for (std::size_t index = 0; index < sample_valid.size(); ++index)
    {
        if (!sample_valid[index])
        {
            continue;
        }

        if (sample_timestamps[index] > source_now)
        {
            return result;
        }

        if (!has_sample)
        {
            earliest_sample_index = index;
            has_sample = true;
        }
        latest_sample_index = index;
    }

    if (!has_sample)
    {
        return result;
    }

    for (std::size_t index = 0; index < sample_valid.size(); ++index)
    {
        if (sample_valid[index])
        {
            result.sample_timestamps[index] =
                m_destination_anchor
                + std::chrono::duration_cast<DestinationClock::duration>(
                    sample_timestamps[index] - m_source_anchor);
            result.sample_valid[index] = true;
        }
    }

    if (m_has_last_output
        && result.sample_timestamps[earliest_sample_index] <= m_last_output)
    {
        const DestinationClock::duration monotonic_shift =
            (m_last_output + DestinationClock::duration(1))
            - result.sample_timestamps[earliest_sample_index];
        for (std::size_t index = 0; index < result.sample_valid.size(); ++index)
        {
            if (result.sample_valid[index])
            {
                result.sample_timestamps[index] += monotonic_shift;
            }
        }
    }

    const DestinationTimePoint latest_allowed_timestamp =
        destination_now
        + std::chrono::duration_cast<DestinationClock::duration>(
            m_config.max_future_lead);
    if (result.sample_timestamps[latest_sample_index]
        > latest_allowed_timestamp)
    {
        result.sample_valid = {{false, false}};
        result.sample_timestamps =
            {{DestinationTimePoint(), DestinationTimePoint()}};
        return result;
    }

    result.status = Status::Synchronized;
    m_has_last_output = true;
    m_last_output = result.sample_timestamps[latest_sample_index];
    return result;
}

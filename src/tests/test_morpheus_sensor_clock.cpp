#include "../psmoveservice/MorpheusHMD/MorpheusSensorClock.h"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>

namespace
{
    int g_failures = 0;

    void expect(bool condition, const std::string &message)
    {
        if (!condition)
        {
            std::cerr << "FAILED: " << message << std::endl;
            ++g_failures;
        }
    }

    MorpheusSensorClock::TimePoint timeUs(std::int64_t microseconds)
    {
        return MorpheusSensorClock::TimePoint(
            std::chrono::duration_cast<MorpheusSensorClock::Clock::duration>(
                std::chrono::microseconds(microseconds)));
    }

    std::int64_t timestampUs(MorpheusSensorClock::TimePoint time_point)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            time_point.time_since_epoch()).count();
    }

    MorpheusClockDomainBridge::DestinationTimePoint filterTimeUs(
        std::int64_t microseconds)
    {
        return MorpheusClockDomainBridge::DestinationTimePoint(
            std::chrono::duration_cast<
                MorpheusClockDomainBridge::DestinationClock::duration>(
                    std::chrono::microseconds(microseconds)));
    }

    std::int64_t filterTimestampUs(
        MorpheusClockDomainBridge::DestinationTimePoint time_point)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            time_point.time_since_epoch()).count();
    }

    MorpheusSensorClock::Config immediateConfig()
    {
        MorpheusSensorClock::Config config;
        config.skew_window_size = 4;
        config.reset_holdoff = std::chrono::microseconds(0);
        return config;
    }

    void testDecodeTick24()
    {
        const std::uint8_t bytes[3] = {0x34, 0x12, 0xab};
        expect(
            MorpheusSensorClock::decodeTick24(bytes) == 0x00ab1234u,
            "24-bit ticks decode as little-endian");
    }

    void testNormalReportsPreserveDeviceSpacing()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result first =
            clock.processReport(1000, 1500, 10, 500, timeUs(10000));
        const MorpheusSensorClock::Result second =
            clock.processReport(2000, 2500, 11, 500, timeUs(11000));

        expect(
            first.status == MorpheusSensorClock::Status::Synchronized,
            "first normal report synchronizes immediately");
        expect(first.sample_count == 2, "first normal report emits two samples");
        expect(
            timestampUs(first.sample_timestamps[1])
                - timestampUs(first.sample_timestamps[0]) == 500,
            "samples in one report retain their exact 500 us spacing");
        expect(
            timestampUs(second.sample_timestamps[0])
                - timestampUs(first.sample_timestamps[1]) == 500,
            "samples in adjacent reports retain device-clock spacing");
        expect(
            timestampUs(second.sample_timestamps[1]) == 11000,
            "minimum-skew mapping does not place the newest sample in the future");
    }

    void testTickRollover()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result first =
            clock.processReport(
                0x00ffff00u,
                0x000000f4u,
                30,
                500,
                timeUs(20000));
        const MorpheusSensorClock::Result second =
            clock.processReport(
                0x000002e8u,
                0x000004dcu,
                31,
                500,
                timeUs(21000));

        expect(
            first.status == MorpheusSensorClock::Status::Synchronized,
            "a valid 24-bit rollover is synchronized");
        expect(first.tick_wrapped, "rollover is reported");
        expect(
            first.unwrapped_ticks_us[1] - first.unwrapped_ticks_us[0] == 500,
            "rollover unwraps to a 500 us forward step");
        expect(
            timestampUs(second.sample_timestamps[0])
                - timestampUs(first.sample_timestamps[1]) == 500,
            "timestamps remain continuous after rollover");
        expect(clock.getStats().tick_wraps == 1, "rollover telemetry is counted");
    }

    void testDuplicateReport()
    {
        MorpheusSensorClock clock(immediateConfig());
        clock.processReport(1000, 1500, 4, 500, timeUs(10000));
        const MorpheusSensorClock::Result duplicate =
            clock.processReport(1000, 1500, 4, 500, timeUs(10100));

        expect(
            duplicate.status == MorpheusSensorClock::Status::Duplicate,
            "identical ticks are rejected as a duplicate report");
        expect(duplicate.sample_count == 0, "a duplicate emits no samples");
        expect(
            clock.getStats().duplicate_reports == 1,
            "duplicate report telemetry is counted");
    }

    void testDroppedSamplesAndReports()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result first =
            clock.processReport(1000, 1500, 10, 500, timeUs(10000));
        const MorpheusSensorClock::Result second =
            clock.processReport(3000, 3500, 12, 500, timeUs(12000));

        expect(second.missing_samples == 2, "two missing samples are detected");
        expect(second.missing_reports == 1, "one missing report is detected");
        expect(
            timestampUs(second.sample_timestamps[0])
                - timestampUs(first.sample_timestamps[1]) == 1500,
            "a sample gap is retained rather than synthesized");
        expect(
            clock.getStats().missing_samples == 2,
            "missing sample telemetry is accumulated");
        expect(
            clock.getStats().missing_reports == 1,
            "missing report telemetry is accumulated");
    }

    void testTickResetThatLooksLikeRollover()
    {
        MorpheusSensorClock clock(immediateConfig());
        clock.processReport(
            10000000,
            10000500,
            20,
            500,
            timeUs(10000));
        const MorpheusSensorClock::Result reset =
            clock.processReport(100, 600, 21, 500, timeUs(11000));
        const MorpheusSensorClock::Result recovered =
            clock.processReport(1100, 1600, 22, 500, timeUs(12000));

        expect(
            reset.status == MorpheusSensorClock::Status::Discontinuity,
            "an implausibly large modular step resets the clock");
        expect(reset.clock_reset, "a tick discontinuity identifies a clock reset");
        expect(
            recovered.status == MorpheusSensorClock::Status::Synchronized,
            "the report after a tick reset establishes a new timeline");
    }

    void testSequenceRollover()
    {
        MorpheusSensorClock clock(immediateConfig());
        clock.processReport(1000, 1500, 255, 500, timeUs(10000));
        const MorpheusSensorClock::Result result =
            clock.processReport(2000, 2500, 0, 500, timeUs(11000));

        expect(result.missing_reports == 0, "255-to-0 sequence rollover is normal");
        expect(
            clock.getStats().sequence_anomalies == 0,
            "sequence rollover is not counted as anomalous");
    }

    void testStartupHoldoff()
    {
        MorpheusSensorClock::Config config = immediateConfig();
        config.reset_holdoff = std::chrono::milliseconds(30);
        MorpheusSensorClock clock(config);

        MorpheusSensorClock::Result result =
            clock.processReport(1000, 1500, 1, 500, timeUs(0));
        expect(
            result.status == MorpheusSensorClock::Status::Holding,
            "the first startup report is held");

        std::uint32_t raw_tick = 2000;
        for (std::int64_t host_us = 1000; host_us < 30000; host_us += 1000)
        {
            result = clock.processReport(
                raw_tick,
                raw_tick + 500,
                static_cast<std::uint8_t>(host_us / 1000 + 1),
                500,
                timeUs(host_us));
            raw_tick += 1000;
            expect(
                result.status == MorpheusSensorClock::Status::Holding,
                "reports before the 30 ms drain deadline are held");
        }

        result = clock.processReport(
            raw_tick,
            raw_tick + 500,
            31,
            500,
            timeUs(30000));
        expect(
            result.status == MorpheusSensorClock::Status::Synchronized,
            "the report at the drain deadline is released");
    }

    void testHostClockBackstep()
    {
        MorpheusSensorClock clock(immediateConfig());
        clock.processReport(1000, 1500, 1, 500, timeUs(10000));
        const MorpheusSensorClock::Result result =
            clock.processReport(2000, 2500, 2, 500, timeUs(9000));

        expect(
            result.status == MorpheusSensorClock::Status::Discontinuity,
            "a host monotonic-clock backstep resets synchronization");
    }

    void testInvalidInternalTickOrder()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result result =
            clock.processReport(1000, 500, 1, 500, timeUs(10000));

        expect(
            result.status == MorpheusSensorClock::Status::Discontinuity,
            "backward ticks inside one report are rejected");
        expect(result.sample_count == 0, "an invalid report emits no samples");
    }

    void testArrivalJitterDoesNotDistortSampleIntervals()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result first =
            clock.processReport(1000, 1500, 1, 500, timeUs(10000));
        const MorpheusSensorClock::Result second =
            clock.processReport(2000, 2500, 2, 500, timeUs(11200));
        const MorpheusSensorClock::Result third =
            clock.processReport(3000, 3500, 3, 500, timeUs(12000));

        expect(
            timestampUs(second.sample_timestamps[1])
                - timestampUs(second.sample_timestamps[0]) == 500,
            "late arrival does not stretch within-report timing");
        expect(
            timestampUs(third.sample_timestamps[0])
                > timestampUs(second.sample_timestamps[1]),
            "mapped timestamps remain monotonic when arrival latency falls");
        expect(
            timestampUs(third.sample_timestamps[1])
                - timestampUs(third.sample_timestamps[0]) == 500,
            "causality correction shifts a batch without changing its spacing");
        expect(
            timestampUs(first.sample_timestamps[1])
                < timestampUs(second.sample_timestamps[0]),
            "normal jittered reports remain ordered");
    }

    void testReportedPeriodIsTelemetryOnly()
    {
        MorpheusSensorClock clock(immediateConfig());
        const MorpheusSensorClock::Result result =
            clock.processReport(1000, 1500, 1, 4321, timeUs(10000));

        expect(
            timestampUs(result.sample_timestamps[1])
                - timestampUs(result.sample_timestamps[0]) == 500,
            "reported period does not override observed device ticks");
        expect(
            clock.getStats().last_reported_sampling_period_us == 4321,
            "reported period is retained for diagnostics");
    }

    void testClockDomainBridgePreservesSpacing()
    {
        MorpheusClockDomainBridge bridge;
        const std::array<MorpheusSensorClock::TimePoint, 2> samples =
            {{timeUs(9000), timeUs(9500)}};
        const std::array<bool, 2> valid = {{true, true}};
        const MorpheusClockDomainBridge::Result result =
            bridge.mapSamples(
                samples,
                valid,
                timeUs(10000),
                filterTimeUs(20000));

        expect(
            result.status
                == MorpheusClockDomainBridge::Status::Synchronized,
            "clock-domain bridge synchronizes its first batch");
        expect(
            filterTimestampUs(result.sample_timestamps[0]) == 19000
                && filterTimestampUs(result.sample_timestamps[1]) == 19500,
            "clock-domain bridge maps sample age from a persistent anchor");
        expect(
            filterTimestampUs(result.sample_timestamps[1])
                - filterTimestampUs(result.sample_timestamps[0]) == 500,
            "clock-domain bridge preserves device sample spacing");
    }

    void testClockDomainBridgeRejectsDestinationRegression()
    {
        MorpheusClockDomainBridge bridge;
        const std::array<bool, 2> valid = {{true, true}};
        bridge.mapSamples(
            {{timeUs(9000), timeUs(9500)}},
            valid,
            timeUs(10000),
            filterTimeUs(20000));

        const MorpheusClockDomainBridge::Result reset =
            bridge.mapSamples(
                {{timeUs(10000), timeUs(10500)}},
                valid,
                timeUs(11000),
                filterTimeUs(19000));
        expect(
            reset.status
                == MorpheusClockDomainBridge::Status::Discontinuity,
            "a destination-clock regression resets the bridge");
        expect(reset.clock_reset, "destination regression is reported");

        const MorpheusClockDomainBridge::Result held =
            bridge.mapSamples(
                {{timeUs(11000), timeUs(11500)}},
                valid,
                timeUs(12000),
                filterTimeUs(20000));
        expect(
            held.status == MorpheusClockDomainBridge::Status::Holding,
            "bridge holds samples that monotonic correction would make future");

        const MorpheusClockDomainBridge::Result recovered =
            bridge.mapSamples(
                {{timeUs(12000), timeUs(12500)}},
                valid,
                timeUs(13000),
                filterTimeUs(21000));
        expect(
            recovered.status
                == MorpheusClockDomainBridge::Status::Synchronized,
            "bridge resumes once destination time catches up");
        expect(
            recovered.sample_timestamps[0]
                > filterTimeUs(19500),
            "bridge output remains newer than the pre-regression batch");
    }

    void testClockDomainBridgeRejectsSourceRegression()
    {
        MorpheusClockDomainBridge bridge;
        const std::array<bool, 2> valid = {{true, true}};
        bridge.mapSamples(
            {{timeUs(9000), timeUs(9500)}},
            valid,
            timeUs(10000),
            filterTimeUs(20000));

        const MorpheusClockDomainBridge::Result reset =
            bridge.mapSamples(
                {{timeUs(8000), timeUs(8500)}},
                valid,
                timeUs(9000),
                filterTimeUs(21000));
        expect(
            reset.status
                == MorpheusClockDomainBridge::Status::Discontinuity,
            "a source-clock regression resets the bridge");
        expect(reset.clock_reset, "source regression is reported");
    }

    void testClockDomainBridgeRejectsFutureSourceSample()
    {
        MorpheusClockDomainBridge bridge;
        const MorpheusClockDomainBridge::Result result =
            bridge.mapSamples(
                {{timeUs(10000), timeUs(10500)}},
                {{true, true}},
                timeUs(10250),
                filterTimeUs(20000));

        expect(
            result.status == MorpheusClockDomainBridge::Status::Holding,
            "a source timestamp newer than source-now is not mapped");
        expect(
            !result.sample_valid[0] && !result.sample_valid[1],
            "a batch containing a future source sample emits nothing");
    }

    void testClockDomainBridgeRejectsLargeOffsetJump()
    {
        MorpheusClockDomainBridge bridge;
        const std::array<bool, 2> valid = {{true, true}};
        bridge.mapSamples(
            {{timeUs(9000), timeUs(9500)}},
            valid,
            timeUs(10000),
            filterTimeUs(20000));

        const MorpheusClockDomainBridge::Result reset =
            bridge.mapSamples(
                {{timeUs(10000), timeUs(10500)}},
                valid,
                timeUs(11000),
                filterTimeUs(121000));
        expect(
            reset.status
                == MorpheusClockDomainBridge::Status::Discontinuity,
            "a material source-to-destination offset jump resets the bridge");
    }

    void testClockDomainBridgeResetStartsCleanSession()
    {
        MorpheusClockDomainBridge bridge;
        const std::array<bool, 2> valid = {{true, true}};
        bridge.mapSamples(
            {{timeUs(9000), timeUs(9500)}},
            valid,
            timeUs(10000),
            filterTimeUs(20000));

        bridge.reset();
        const MorpheusClockDomainBridge::Result new_session =
            bridge.mapSamples(
                {{timeUs(1000), timeUs(1500)}},
                valid,
                timeUs(2000),
                filterTimeUs(5000));

        expect(
            new_session.status
                == MorpheusClockDomainBridge::Status::Synchronized,
            "an explicit bridge reset accepts a new device session");
        expect(
            filterTimestampUs(new_session.sample_timestamps[0]) == 4000
                && filterTimestampUs(new_session.sample_timestamps[1]) == 4500,
            "a new session is not constrained by the prior output timeline");
    }
}

int main()
{
    testDecodeTick24();
    testNormalReportsPreserveDeviceSpacing();
    testTickRollover();
    testDuplicateReport();
    testDroppedSamplesAndReports();
    testTickResetThatLooksLikeRollover();
    testSequenceRollover();
    testStartupHoldoff();
    testHostClockBackstep();
    testInvalidInternalTickOrder();
    testArrivalJitterDoesNotDistortSampleIntervals();
    testReportedPeriodIsTelemetryOnly();
    testClockDomainBridgePreservesSpacing();
    testClockDomainBridgeRejectsDestinationRegression();
    testClockDomainBridgeRejectsSourceRegression();
    testClockDomainBridgeRejectsFutureSourceSample();
    testClockDomainBridgeRejectsLargeOffsetJump();
    testClockDomainBridgeResetStartsCleanSession();

    if (g_failures == 0)
    {
        std::cout << "MorpheusSensorClock tests passed" << std::endl;
        return 0;
    }

    std::cerr << g_failures << " MorpheusSensorClock test(s) failed" << std::endl;
    return 1;
}

#include "../psmoveservice/PSMoveTracker/TrackerVideoSource.h"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
    int failure_count = 0;

    void expect(bool condition, const char *description)
    {
        if (!condition)
        {
            ++failure_count;
            std::cerr << "FAILED: " << description << std::endl;
        }
    }

    void testMediaModeIdentity()
    {
        TrackerVideoMediaMode first;
        first.width = 640;
        first.height = 480;
        first.frame_rate_numerator = 30000;
        first.frame_rate_denominator = 1001;
        first.subtype_guid = "{32595559-0000-0010-8000-00aa00389b71}";

        TrackerVideoMediaMode same = first;
        same.subtype_guid = "{32595559-0000-0010-8000-00AA00389B71}";

        TrackerVideoMediaMode different_rate = first;
        different_rate.frame_rate_numerator = 30;
        different_rate.frame_rate_denominator = 1;

        TrackerVideoMediaMode interlaced = first;
        interlaced.interlace_mode = 3;

        expect(first.isValid(), "a complete native mode is valid");
        expect(first == same, "subtype GUID comparison is case-insensitive");
        expect(
            first != different_rate,
            "rational frame rate is part of explicit mode identity");
        expect(
            first != interlaced,
            "interlace mode is part of explicit mode identity");
    }

    void testTimestampMapping()
    {
        using Clock = std::chrono::steady_clock;

        TrackerVideoTimestampMapper mapper(std::chrono::milliseconds(5));
        const Clock::time_point first_arrival(
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::seconds(10)));
        bool used_source = false;
        const Clock::time_point first_capture = mapper.map(
            1000000,
            first_arrival,
            true,
            false,
            &used_source);
        expect(used_source, "valid source timestamp is used");
        expect(
            first_capture == first_arrival - std::chrono::milliseconds(5),
            "first source timestamp anchors to arrival minus latency");

        const Clock::time_point second_capture = mapper.map(
            1333333,
            first_arrival + std::chrono::milliseconds(40),
            true,
            false,
            &used_source);
        expect(
            second_capture - first_capture
                == std::chrono::nanoseconds(33333300),
            "source timestamp delta is preserved in the steady clock");

        const Clock::time_point reset_arrival =
            first_arrival + std::chrono::milliseconds(80);
        const Clock::time_point reset_capture = mapper.map(
            1100000,
            reset_arrival,
            true,
            false,
            &used_source);
        expect(
            reset_capture == reset_arrival - std::chrono::milliseconds(5),
            "backward source timestamps are re-anchored");

        const Clock::time_point fallback_arrival =
            first_arrival + std::chrono::milliseconds(120);
        const Clock::time_point fallback_capture = mapper.map(
            0,
            fallback_arrival,
            false,
            false,
            &used_source);
        expect(!used_source, "invalid source timestamps use arrival time");
        expect(
            fallback_capture
                == fallback_arrival - std::chrono::milliseconds(5),
            "arrival fallback still applies configured latency");
    }

    TrackerVideoFrame makeFrame(uint64_t sequence, unsigned char value)
    {
        TrackerVideoFrame frame;
        frame.sequence = sequence;
        frame.bgr = cv::Mat(2, 2, CV_8UC3, cv::Scalar(value, value, value));
        return frame;
    }

    void testLatestFrameBuffer()
    {
        TrackerVideoLatestFrameBuffer buffer;
        expect(!buffer.hasNewFrame(), "new frame buffer starts empty");

        expect(buffer.push(makeFrame(1, 1)), "first BGR frame is accepted");
        expect(buffer.push(makeFrame(2, 2)), "second BGR frame is accepted");
        expect(buffer.push(makeFrame(3, 3)), "third BGR frame is accepted");

        TrackerVideoFrame latest;
        expect(buffer.tryGetLatestFrame(latest), "latest frame can be consumed");
        expect(latest.sequence == 3, "consumer receives newest frame only");
        expect(
            latest.dropped_frames_before_this == 2,
            "overwritten unread frames are reported");
        expect(
            buffer.getTotalDroppedFrameCount() == 2,
            "lifetime drop counter is maintained");
        expect(
            !buffer.tryGetLatestFrame(latest),
            "a frame generation is consumed at most once");

        cv::Mat backing(2, 4, CV_8UC3, cv::Scalar(9, 9, 9));
        TrackerVideoFrame non_contiguous;
        non_contiguous.sequence = 4;
        non_contiguous.bgr = backing(cv::Rect(0, 0, 2, 2));
        expect(
            !non_contiguous.bgr.isContinuous(),
            "test fixture has a non-contiguous ROI");
        expect(
            buffer.push(non_contiguous),
            "non-contiguous BGR input is canonicalized");
        backing.setTo(cv::Scalar(1, 1, 1));
        expect(buffer.tryGetLatestFrame(latest), "canonical frame is available");
        expect(latest.bgr.isContinuous(), "published BGR frame is contiguous");
        expect(
            latest.bgr.at<cv::Vec3b>(0, 0)[0] == 9,
            "canonicalization owns non-contiguous image storage");

        TrackerVideoFrame invalid;
        invalid.bgr = cv::Mat(2, 2, CV_8UC1);
        expect(!buffer.push(invalid), "non-BGR frames are rejected");
    }

    void testControlNormalization()
    {
        expect(
            TrackerVideoControlNormalization::fromDeviceValue(-13, -13, -1)
                == 1,
            "manual range minimum maps to one");
        expect(
            TrackerVideoControlNormalization::fromDeviceValue(-1, -13, -1)
                == 255,
            "manual range maximum maps to 255");
        expect(
            TrackerVideoControlNormalization::toDeviceValue(1, -13, -1, 1)
                == -13,
            "one maps to the manual range minimum");
        expect(
            TrackerVideoControlNormalization::toDeviceValue(255, -13, -1, 1)
                == -1,
            "255 maps to the manual range maximum");
        expect(
            TrackerVideoControlNormalization::toDeviceValue(128, 0, 100, 10)
                == 50,
            "manual values honor the camera step size");
        expect(
            TrackerVideoControlNormalization::toDeviceValue(0, 10, 20, 1)
                == 10,
            "automatic sentinel has a safe manual fallback value");
    }
}

int main()
{
    testMediaModeIdentity();
    testTimestampMapping();
    testLatestFrameBuffer();
    testControlNormalization();

    if (failure_count == 0)
    {
        std::cout << "Tracker video source helper tests passed." << std::endl;
        return EXIT_SUCCESS;
    }

    std::cerr << failure_count << " helper test(s) failed." << std::endl;
    return EXIT_FAILURE;
}

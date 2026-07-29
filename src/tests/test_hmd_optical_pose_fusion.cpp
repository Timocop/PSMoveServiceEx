#include "../psmoveservice/Device/View/HMDOpticalPoseFusion.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

namespace
{
    int g_failure_count = 0;

    void expect(bool condition, const std::string &description)
    {
        if (!condition)
        {
            ++g_failure_count;
            std::cerr << "FAILED: " << description << std::endl;
        }
    }

    HMDOpticalPoseFusion::TimePoint milliseconds(int value)
    {
        return HMDOpticalPoseFusion::TimePoint(
            std::chrono::duration_cast<
                HMDOpticalPoseFusion::Clock::duration>(
                std::chrono::milliseconds(value)));
    }

    float orientationErrorDegrees(
        const Eigen::Quaternionf &first,
        const Eigen::Quaternionf &second)
    {
        const float dot = std::fabs(
            first.normalized().coeffs().dot(second.normalized().coeffs()));
        const float clamped = std::max(0.f, std::min(1.f, dot));
        return 2.f * std::acos(clamped) * 180.f / 3.14159265358979323846f;
    }

    void testTimingValidationAndDeduplication()
    {
        using HMDOpticalPoseFusion::TimingStatus;

        const HMDOpticalPoseFusion::TimingResult accepted =
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                true,
                milliseconds(900),
                true,
                milliseconds(850),
                milliseconds(1000),
                0.25f);
        expect(
            accepted.status == TimingStatus::Accepted,
            "a new measurement inside the age window is accepted");
        expect(
            std::string(
                HMDOpticalPoseFusion::timingStatusName(
                    accepted.status)) == "accepted",
            "timing diagnostics expose a stable human-readable status");
        expect(
            std::fabs(accepted.age_seconds - 0.1f) < 1e-5f,
            "accepted timing reports the capture-to-fusion age");

        expect(
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                true,
                milliseconds(900),
                true,
                milliseconds(900),
                milliseconds(1000),
                0.25f).status
                == TimingStatus::DuplicateOrOutOfOrder,
            "the same camera frame cannot be fused twice");
        expect(
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                true,
                milliseconds(899),
                true,
                milliseconds(900),
                milliseconds(1000),
                0.25f).status
                == TimingStatus::DuplicateOrOutOfOrder,
            "out-of-order camera frames are rejected");
        expect(
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                true,
                milliseconds(1001),
                false,
                HMDOpticalPoseFusion::TimePoint(),
                milliseconds(1000),
                0.25f).status
                == TimingStatus::FutureMeasurement,
            "measurements after the fusion state are rejected");
        expect(
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                true,
                milliseconds(700),
                false,
                HMDOpticalPoseFusion::TimePoint(),
                milliseconds(1000),
                0.25f).status
                == TimingStatus::StaleMeasurement,
            "measurements older than the bounded rewind window are rejected");
        expect(
            HMDOpticalPoseFusion::evaluateMeasurementTiming(
                false,
                HMDOpticalPoseFusion::TimePoint(),
                false,
                HMDOpticalPoseFusion::TimePoint(),
                milliseconds(1000),
                0.25f).status
                == TimingStatus::MissingTimestamp,
            "untimestamped optical samples are rejected");
    }

    void testNonCommutingLocalMotionAlignment()
    {
        const float degrees_to_radians =
            3.14159265358979323846f / 180.f;
        const Eigen::Quaternionf correction(
            Eigen::AngleAxisf(
                20.f * degrees_to_radians,
                Eigen::Vector3f::UnitY()));
        const Eigen::Quaternionf imu_capture(
            Eigen::AngleAxisf(
                30.f * degrees_to_radians,
                Eigen::Vector3f::UnitX()));
        const Eigen::Quaternionf local_motion(
            Eigen::AngleAxisf(
                15.f * degrees_to_radians,
                Eigen::Vector3f::UnitZ()));
        const Eigen::Quaternionf optical_capture =
            correction * imu_capture;
        const Eigen::Quaternionf imu_fusion =
            imu_capture * local_motion;
        const Eigen::Quaternionf expected =
            optical_capture * local_motion;

        Eigen::Quaternionf actual = Eigen::Quaternionf::Identity();
        expect(
            HMDOpticalPoseFusion::forwardAlignOrientation(
                optical_capture,
                imu_capture,
                imu_fusion,
                actual),
            "finite noncommuting orientations can be aligned");
        expect(
            orientationErrorDegrees(expected, actual) < 0.001f,
            "alignment applies the IMU motion on the local/right side");

        const Eigen::Quaternionf incorrect_world_delta =
            imu_fusion * imu_capture.conjugate() * optical_capture;
        expect(
            orientationErrorDegrees(expected, incorrect_world_delta) > 1.f,
            "the test distinguishes local motion from the wrong world-delta order");
    }

    void testPositionAlignment()
    {
        Eigen::Vector3f aligned = Eigen::Vector3f::Zero();
        expect(
            HMDOpticalPoseFusion::forwardAlignPosition(
                Eigen::Vector3f(10.f, -5.f, 120.f),
                Eigen::Vector3f(20.f, 10.f, -5.f),
                0.1f,
                aligned),
            "finite position and velocity can be aligned");
        expect(
            aligned.isApprox(
                Eigen::Vector3f(12.f, -4.f, 119.5f),
                1e-5f),
            "position alignment advances by velocity times frame age");
        expect(
            !HMDOpticalPoseFusion::forwardAlignPosition(
                Eigen::Vector3f::Zero(),
                Eigen::Vector3f::Zero(),
                -0.1f,
                aligned),
            "negative position age is rejected");
    }
}

int main()
{
    testTimingValidationAndDeduplication();
    testNonCommutingLocalMotionAlignment();
    testPositionAlignment();

    if (g_failure_count == 0)
    {
        std::cout << "HMD optical pose fusion tests passed" << std::endl;
        return 0;
    }

    std::cerr << g_failure_count
              << " HMD optical pose fusion test(s) failed"
              << std::endl;
    return 1;
}

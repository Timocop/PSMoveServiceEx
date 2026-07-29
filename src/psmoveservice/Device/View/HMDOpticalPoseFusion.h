#ifndef HMD_OPTICAL_POSE_FUSION_H
#define HMD_OPTICAL_POSE_FUSION_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <chrono>

namespace HMDOpticalPoseFusion
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;

    enum class TimingStatus
    {
        Accepted,
        InvalidMaximumAge,
        MissingTimestamp,
        DuplicateOrOutOfOrder,
        FutureMeasurement,
        StaleMeasurement
    };

    struct TimingResult
    {
        TimingResult()
            : status(TimingStatus::MissingTimestamp)
            , age_seconds(0.f)
        {
        }

        TimingStatus status;
        float age_seconds;
    };

    const char *timingStatusName(TimingStatus status);

    TimingResult evaluateMeasurementTiming(
        bool has_measurement_timestamp,
        TimePoint measurement_timestamp,
        bool has_last_measurement_timestamp,
        TimePoint last_measurement_timestamp,
        TimePoint fusion_timestamp,
        float maximum_age_seconds);

    // The project integrates local gyro motion as q_new = q_old * delta.
    // Apply the same right/local delta to the optical orientation captured
    // earlier, preserving the absolute correction supplied by the camera.
    bool forwardAlignOrientation(
        const Eigen::Quaternionf &optical_orientation_at_capture,
        const Eigen::Quaternionf &imu_orientation_at_capture,
        const Eigen::Quaternionf &imu_orientation_at_fusion,
        Eigen::Quaternionf &out_orientation_at_fusion);

    bool forwardAlignPosition(
        const Eigen::Vector3f &optical_position_at_capture_cm,
        const Eigen::Vector3f &velocity_at_fusion_cm_per_second,
        float age_seconds,
        Eigen::Vector3f &out_position_at_fusion_cm);
}

#endif // HMD_OPTICAL_POSE_FUSION_H

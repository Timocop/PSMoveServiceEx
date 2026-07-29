#include "HMDOpticalPoseFusion.h"

#include <cmath>

namespace
{
    bool isFiniteQuaternion(const Eigen::Quaternionf &value)
    {
        return value.coeffs().allFinite()
            && std::isfinite(value.norm())
            && value.norm() > 1e-6f;
    }
}

namespace HMDOpticalPoseFusion
{
    const char *timingStatusName(TimingStatus status)
    {
        switch (status)
        {
        case TimingStatus::Accepted:
            return "accepted";
        case TimingStatus::InvalidMaximumAge:
            return "invalid maximum age";
        case TimingStatus::MissingTimestamp:
            return "missing capture timestamp";
        case TimingStatus::DuplicateOrOutOfOrder:
            return "duplicate or out-of-order capture timestamp";
        case TimingStatus::FutureMeasurement:
            return "capture timestamp is newer than the filter state";
        case TimingStatus::StaleMeasurement:
            return "capture timestamp is too old";
        }

        return "unknown";
    }

    TimingResult evaluateMeasurementTiming(
        bool has_measurement_timestamp,
        TimePoint measurement_timestamp,
        bool has_last_measurement_timestamp,
        TimePoint last_measurement_timestamp,
        TimePoint fusion_timestamp,
        float maximum_age_seconds)
    {
        TimingResult result;

        if (!std::isfinite(maximum_age_seconds)
            || maximum_age_seconds <= 0.f)
        {
            result.status = TimingStatus::InvalidMaximumAge;
            return result;
        }

        if (!has_measurement_timestamp
            || measurement_timestamp == TimePoint())
        {
            result.status = TimingStatus::MissingTimestamp;
            return result;
        }

        if (has_last_measurement_timestamp
            && measurement_timestamp <= last_measurement_timestamp)
        {
            result.status = TimingStatus::DuplicateOrOutOfOrder;
            return result;
        }

        if (measurement_timestamp > fusion_timestamp)
        {
            result.status = TimingStatus::FutureMeasurement;
            return result;
        }

        result.age_seconds =
            std::chrono::duration<float>(
                fusion_timestamp - measurement_timestamp).count();
        if (!std::isfinite(result.age_seconds)
            || result.age_seconds > maximum_age_seconds)
        {
            result.status = TimingStatus::StaleMeasurement;
            return result;
        }

        result.status = TimingStatus::Accepted;
        return result;
    }

    bool forwardAlignOrientation(
        const Eigen::Quaternionf &optical_orientation_at_capture,
        const Eigen::Quaternionf &imu_orientation_at_capture,
        const Eigen::Quaternionf &imu_orientation_at_fusion,
        Eigen::Quaternionf &out_orientation_at_fusion)
    {
        if (!isFiniteQuaternion(optical_orientation_at_capture)
            || !isFiniteQuaternion(imu_orientation_at_capture)
            || !isFiniteQuaternion(imu_orientation_at_fusion))
        {
            return false;
        }

        const Eigen::Quaternionf optical =
            optical_orientation_at_capture.normalized();
        const Eigen::Quaternionf imu_capture =
            imu_orientation_at_capture.normalized();
        const Eigen::Quaternionf imu_fusion =
            imu_orientation_at_fusion.normalized();

        const Eigen::Quaternionf local_motion =
            imu_capture.conjugate() * imu_fusion;
        const Eigen::Quaternionf aligned = optical * local_motion;
        if (!isFiniteQuaternion(aligned))
        {
            return false;
        }

        out_orientation_at_fusion = aligned.normalized();
        return true;
    }

    bool forwardAlignPosition(
        const Eigen::Vector3f &optical_position_at_capture_cm,
        const Eigen::Vector3f &velocity_at_fusion_cm_per_second,
        float age_seconds,
        Eigen::Vector3f &out_position_at_fusion_cm)
    {
        if (!optical_position_at_capture_cm.allFinite()
            || !velocity_at_fusion_cm_per_second.allFinite()
            || !std::isfinite(age_seconds)
            || age_seconds < 0.f)
        {
            return false;
        }

        const Eigen::Vector3f aligned =
            optical_position_at_capture_cm
            + velocity_at_fusion_cm_per_second * age_seconds;
        if (!aligned.allFinite())
        {
            return false;
        }

        out_position_at_fusion_cm = aligned;
        return true;
    }
}

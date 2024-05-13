// -- includes --
#include "CompoundPoseFilter.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"

#if !defined(IS_TESTING_KALMAN)
#include "ExternalOrientationFilter.h"
#include "PositionExternalAttachment.h"
#include "OpticalOrientationFilter.h"
#endif

// -- public interface --
bool CompoundPoseFilter::init(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant)
{
	bool bSuccess = true;

	allocate_filters(deviceType, orientationFilterType, positionFilterType, constant);

	if (m_orientation_filter != nullptr)
	{
		bSuccess &= m_orientation_filter->init(constant.orientation_constants);
	}

	if (m_position_filter != nullptr)
	{
		bSuccess &= m_position_filter->init(constant.position_constants);
	}

	return bSuccess;
}

bool CompoundPoseFilter::init(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant,
	const Eigen::Vector3f &initial_position,
	const Eigen::Quaternionf &initial_orientation)
{
	bool bSuccess = true;

	allocate_filters(deviceType, orientationFilterType, positionFilterType, constant);

	if (m_orientation_filter != nullptr)
	{
		bSuccess &= m_orientation_filter->init(constant.orientation_constants, initial_orientation);
	}

	if (m_position_filter != nullptr)
	{
		bSuccess &= m_position_filter->init(constant.position_constants, initial_position);
	}

	return bSuccess;
}

void CompoundPoseFilter::allocate_filters(
	const CommonDeviceState::eDeviceType deviceType,
	const OrientationFilterType orientationFilterType,
	const PositionFilterType positionFilterType,
	const PoseFilterConstants &constant)
{
	dispose_filters();

	switch(orientationFilterType)
	{
    case OrientationFilterTypeNone:
		m_orientation_filter = nullptr;
		break;
    case OrientationFilterTypePassThru:
		m_orientation_filter = new OrientationFilterPassThru;
		break;
    case OrientationFilterTypeMadgwickARG:
		m_orientation_filter = new OrientationFilterMadgwickARG;
		break;
    case OrientationFilterTypeMadgwickMARG:
		m_orientation_filter = new OrientationFilterMadgwickMARG;
		break;
    case OrientationFilterTypeComplementaryOpticalARG:
		m_orientation_filter = new OrientationFilterComplementaryOpticalARG;
		break;
	case OrientationFilterTypeComplementaryMARG:
		m_orientation_filter = new OrientationFilterComplementaryMARG;
		break;
	case OrientationFilterTypeOrientationTargetOpticalARG:
#if !defined(IS_TESTING_KALMAN)
		m_orientation_filter = new OrientationTargetOpticalARG;
#else
		m_orientation_filter = nullptr;
#endif
		break;
	case OrientationFilterTypeExternal:
#if !defined(IS_TESTING_KALMAN)
		m_orientation_filter = new OrientationFilterExternal;
#else
		m_orientation_filter = nullptr;
#endif
		break;
	default:
		assert(0 && "unreachable");
	}

	switch(positionFilterType)
	{
    case PositionFilterTypeNone:
		m_position_filter = nullptr;
		break;
    case PositionFilterTypePassThru:
		m_position_filter = new PositionFilterPassThru;
		break;
    case PositionFilterTypeLowPassOptical:
		m_position_filter = new PositionFilterLowPassOptical;
		break;
    case PositionFilterTypeComplimentaryOpticalIMU:
		m_position_filter = new PositionFilterComplimentaryOpticalIMU;
		break;
	case PositionFilterTypeKalman:
		m_position_filter = new PositionFilterKalman;
		break;
	case PositionFilterTypeExternalAttachment:
#if !defined(IS_TESTING_KALMAN)
		m_position_filter = new PositionFilterExternalAttachment;
#else
		m_position_filter = nullptr;
#endif
		break;
	default:
		assert(0 && "unreachable");
	}
}

// -- IStateFilter --
bool CompoundPoseFilter::getIsStateValid() const
{
	return getIsOrientationStateValid() || getIsPositionStateValid();
}

double CompoundPoseFilter::getTimeInSeconds() const
{
	const t_high_resolution_duration_milli optical_time_delta = std::chrono::high_resolution_clock::now() - last_optical_timestamp;
	const t_high_resolution_duration_milli imu_time_delta = std::chrono::high_resolution_clock::now() - last_imu_timestamp;
	const float time_delta_milli = optical_time_delta.count() + imu_time_delta.count();

	return (time_delta_milli / 1000.f);
}

void CompoundPoseFilter::update(
	const t_high_resolution_timepoint timestamp,
	const PoseFilterPacket &orientation_filter_packet)
{
    Eigen::Quaternionf filtered_orientation= Eigen::Quaternionf::Identity();
	if (m_orientation_filter != nullptr && m_position_filter != nullptr)
	{
		// Update the orientation filter first
		m_orientation_filter->update(timestamp, orientation_filter_packet);
        filtered_orientation= m_orientation_filter->getOrientation();

		last_imu_timestamp = timestamp;
    }

    if (m_position_filter != nullptr)
    {
		// Update the position filter using the latest orientation
		PoseFilterPacket position_filter_packet= orientation_filter_packet;
		position_filter_packet.current_orientation= filtered_orientation;

		m_position_filter->update(timestamp, position_filter_packet);

		last_optical_timestamp = timestamp;
	}
}

void CompoundPoseFilter::resetState()
{
	if (m_orientation_filter != nullptr && m_position_filter != nullptr)
	{
		m_orientation_filter->resetState();
		m_position_filter->resetState();
		last_optical_timestamp = t_high_resolution_timepoint();
		last_optical_timestamp = t_high_resolution_timepoint();
	}
}

void CompoundPoseFilter::recenterOrientation(const Eigen::Quaternionf& q_pose)
{
	if (m_orientation_filter != nullptr)
	{
		m_orientation_filter->recenterOrientation(q_pose);
	}
}

bool CompoundPoseFilter::getIsPositionStateValid() const
{
	return m_position_filter != nullptr && m_position_filter->getIsStateValid();
}

bool CompoundPoseFilter::getIsOrientationStateValid() const
{
	return m_orientation_filter != nullptr && m_orientation_filter->getIsStateValid();
}

Eigen::Quaternionf CompoundPoseFilter::getOrientation(float time, float offset_x, float offset_y, float offset_z, float offset_world_x, float offset_world_y, float offset_world_z) const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getOrientation(time, offset_x, offset_y, offset_z, offset_world_x, offset_world_y, offset_world_z) : Eigen::Quaternionf::Identity();
}

Eigen::Quaternionf CompoundPoseFilter::getResetOrientation() const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getResetOrientation() : Eigen::Quaternionf::Identity();
}

Eigen::Vector3f CompoundPoseFilter::getAngularVelocityRadPerSec() const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getAngularVelocityRadPerSec() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getAngularAccelerationRadPerSecSqr() const
{
	return (m_orientation_filter != nullptr) ? m_orientation_filter->getAngularAccelerationRadPerSecSqr() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getPositionCm(float time) const
{
	return (m_position_filter != nullptr) ? m_position_filter->getPositionCm(time) : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getVelocityCmPerSec() const
{
	return (m_position_filter != nullptr) ? m_position_filter->getVelocityCmPerSec() : Eigen::Vector3f::Zero();
}

Eigen::Vector3f CompoundPoseFilter::getAccelerationCmPerSecSqr() const
{
	return (m_position_filter != nullptr) ? m_position_filter->getAccelerationCmPerSecSqr() : Eigen::Vector3f::Zero();
}

void CompoundPoseFilter::dispose_filters()
{
	if (m_orientation_filter != nullptr)
	{
		delete m_orientation_filter;
		m_orientation_filter= nullptr;
	}

	if (m_position_filter != nullptr)
	{
		delete m_position_filter;
		m_position_filter= nullptr;
	}
}
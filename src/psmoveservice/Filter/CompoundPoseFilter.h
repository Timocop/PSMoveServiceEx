#ifndef COMPOUND_POSE_FILTER_H
#define COMPOUND_POSE_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"
#include "DeviceInterface.h"

// -- constants --
enum OrientationFilterType {
    OrientationFilterTypeNone,
    OrientationFilterTypePassThru,
    OrientationFilterTypeMadgwickARG,
    OrientationFilterTypeMadgwickMARG,
    OrientationFilterTypeComplementaryOpticalARG,
    OrientationFilterTypeComplementaryMARG,
	OrientationFilterTypeOrientationTargetOpticalARG,
	OrientationFilterTypeExternal,
};

enum PositionFilterType {
    PositionFilterTypeNone,
    PositionFilterTypePassThru,
    PositionFilterTypeLowPassOptical,
    PositionFilterTypeComplimentaryOpticalIMU,
	PositionFilterTypeKalman,
	PositionFilterTypeExternalAttachment,
};

// -- definitions --
class CompoundPoseFilter : public IPoseFilter
{
public:
    CompoundPoseFilter() 
        : m_position_filter(nullptr)
        , m_orientation_filter(nullptr)
    { }
    ~CompoundPoseFilter()
    {
		dispose_filters();
	}

    bool init(
		const CommonDeviceState::eDeviceType deviceType,
        const OrientationFilterType orientationFilterType, 
        const PositionFilterType positionFilterType,
        const PoseFilterConstants &constant);
	bool init(
		const CommonDeviceState::eDeviceType deviceType,
		const OrientationFilterType orientationFilterType,
		const PositionFilterType positionFilterType,
		const PoseFilterConstants &constant,
		const Eigen::Vector3f &initial_position,
		const Eigen::Quaternionf &initial_orientation);

    // -- IStateFilter --
    bool getIsStateValid() const override;
    double getTimeInSeconds() const override;
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
    void resetState() override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

    // -- IPoseFilter ---
    bool getIsPositionStateValid() const override;
    bool getIsOrientationStateValid() const override;
	Eigen::Quaternionf getOrientation(float time = 0.f, float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f, float offset_world_x = 0.f, float offset_world_y = 0.f, float offset_world_z = 0.f) const override;
	Eigen::Quaternionf getResetOrientation() const override;
    Eigen::Vector3f getAngularVelocityRadPerSec() const override;
    Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;
    Eigen::Vector3f getPositionCm(float time = 0.f) const override;
    Eigen::Vector3f getVelocityCmPerSec() const override;
    Eigen::Vector3f getAccelerationCmPerSecSqr() const override;

protected:
	void allocate_filters(
		const CommonDeviceState::eDeviceType deviceType,
		const OrientationFilterType orientationFilterType,
		const PositionFilterType positionFilterType,
		const PoseFilterConstants &constant);
    void dispose_filters();

    IPositionFilter *m_position_filter;
    IOrientationFilter *m_orientation_filter;

	t_high_resolution_timepoint last_optical_timestamp;
	t_high_resolution_timepoint last_imu_timestamp;
};

#endif // COMPOUND_POSE_FILTER_H
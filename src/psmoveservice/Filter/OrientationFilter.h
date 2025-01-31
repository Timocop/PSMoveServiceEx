#ifndef ORIENTATION_FILTER_H
#define ORIENTATION_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"

#if !defined(IS_TESTING_KALMAN)
#include "DeviceManager.h" 
#include "ControllerManager.h"
#include "ServerControllerView.h"
#include "ServerHMDView.h"
#include "PSMoveController.h"
#include "PSDualShock4Controller.h"
#include "VirtualController.h"
#include "MorpheusHMD.h"
#include "VirtualHMD.h"
#endif


//-- definitions --
/// Abstract base class for all orientation only filters
class OrientationFilter : public IOrientationFilter
{
public:
    OrientationFilter();
    ~OrientationFilter();

    //-- IStateFilter --
    bool getIsStateValid() const override;
    double getTimeInSeconds() const override;
    void resetState() override;
    void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

    // -- IOrientationFilter --
    bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const Eigen::Quaternionf &initial_orientation) override;

	Eigen::Quaternionf getOrientation(float time = 0.f, float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f, float offset_world_x = 0.f, float offset_world_y = 0.f, float offset_world_z = 0.f) const override;
	Eigen::Quaternionf getResetOrientation() const override;
    Eigen::Vector3f getAngularVelocityRadPerSec(float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f) const override;
    Eigen::Vector3f getAngularAccelerationRadPerSecSqr(float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f) const override;

protected:
    OrientationFilterConstants m_constants;
    struct OrientationFilterState *m_state;
	struct OrientationInterpolationState *m_interpolationState;
};

/// Just use the optical orientation passed in unfiltered
class OrientationFilterPassThru : public OrientationFilter
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
};

/// Magnetic, Angular Rate, Gravity and fusion algorithm (hybrid Madgwick)
/// Blends between best fit Mag-Grav alignment and Angular Rate integration
class OrientationFilterComplementaryMARG : public OrientationFilter
{
public:
	enum PassiveDriftCorrectionMethod
	{
		StableGravity = 0,
		StableGyroAccel,
		Both,
	};

	OrientationFilterComplementaryMARG()
		: OrientationFilter()
		, mg_weight(1.f)
		, mg_ignored(false)
	{
		timeStableDelay = std::chrono::high_resolution_clock::now();

		last_accelerometer_g_units = Eigen::Vector3f(0.f, 0.f, 0.f);
		last_accelerometer_derivative_g_per_sec = Eigen::Vector3f(0.f, 0.f, 0.f);
		last_acceleration_m_per_sec_sqr = Eigen::Vector3f(0.f, 0.f, 0.f);
	}

	void resetState() override;
	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;

	bool filter_process_passive_drift_correction(
		const float delta_time,
		const PoseFilterPacket & packet,
		bool filter_use_passive_drift_correction,
		PassiveDriftCorrectionMethod filter_passive_drift_correction_method,
		float filter_passive_drift_correction_deadzone,
		float filter_passive_drift_correction_gravity_deadzone,
		float filter_passive_drift_correction_delay);

	void filter_process_stabilization(
		const float delta_time,
		const PoseFilterPacket & packet,
		float filter_stabilization_min_scale);

protected:
	float mg_weight;
	bool mg_ignored;
	std::chrono::time_point<std::chrono::high_resolution_clock> timeStableDelay;
	Eigen::Vector3f last_accelerometer_g_units;
	Eigen::Vector3f last_accelerometer_derivative_g_per_sec;
	Eigen::Vector3f last_acceleration_m_per_sec_sqr;
	float m_mag_scale;
};

/// Angular Rate and Gravity fusion algorithm from Madgwick
class OrientationFilterMadgwickARG : public OrientationFilterComplementaryMARG
{
public:

	OrientationFilterMadgwickARG() 
		: OrientationFilterComplementaryMARG()
		, m_beta(0.f)
		, m_smartResetTime(0.0f)
	{
		m_smartOrientation = Eigen::Quaternionf::Identity();
		timeReset = std::chrono::high_resolution_clock::now();
	}

	void resetState() override;
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

protected:
	float m_beta;
	float m_smartResetTime;
	Eigen::Quaternionf m_smartOrientation;
	std::chrono::time_point<std::chrono::high_resolution_clock> timeReset;
};

/// Magnetic, Angular Rate, and Gravity fusion algorithm from Madgwick
class OrientationFilterMadgwickMARG : public OrientationFilterMadgwickARG
{
public:
    OrientationFilterMadgwickMARG()
        : OrientationFilterMadgwickARG()
    {
		timeReset = std::chrono::high_resolution_clock::now();
	}

    void resetState() override;
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;

protected:
	std::chrono::time_point<std::chrono::high_resolution_clock> timeReset;
	float m_mag_scale;
};

/// Angular Rate, Gravity, and Optical fusion algorithm
/// Blends between AngularRate-Grav Madgwick IMU update and optical orientation
class OrientationFilterComplementaryOpticalARG : public OrientationFilterMadgwickARG
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
};

#endif // ORIENTATION_FILTER_H

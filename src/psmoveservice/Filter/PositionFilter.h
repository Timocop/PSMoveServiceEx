#ifndef POSITION_FILTER_H
#define POSITION_FILTER_H

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

#include <chrono>
#include <list>

//-- definitions -----
/// Abstract base class for all orientation only filters
class PositionFilter : public IPositionFilter
{
public:
    PositionFilter();
    virtual ~PositionFilter();

    //-- IStateFilter --
    bool getIsStateValid() const override;
    double getTimeInSeconds() const override;
    void resetState() override;
    void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

    // -- IOrientationFilter --
    bool init(const PositionFilterConstants &constant) override;
	bool init(const PositionFilterConstants &constant, const Eigen::Vector3f &initial_position) override;

    Eigen::Vector3f getPositionCm(float time = 0.f) const override;
    Eigen::Vector3f getVelocityCmPerSec() const override;
    Eigen::Vector3f getAccelerationCmPerSecSqr() const override;

protected:
    PositionFilterConstants m_constants;
	struct PositionFilterState *m_state;
	struct PositionInterpolationState *m_interpolationState;
};

/// Just use the optical orientation passed in unfiltered
class PositionFilterPassThru : public PositionFilter
{
public:
	PositionFilterPassThru()
		: PositionFilter()
		, m_resetVelocity(false)
	{
		lastOpticalFrame = std::chrono::high_resolution_clock::now();
	}

    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
	bool m_resetVelocity;
};

class PositionFilterLowPassOptical : public PositionFilter
{
public:
	PositionFilterLowPassOptical()
		: PositionFilter()
		, m_resetVelocity(false)
	{
		lastOpticalFrame = std::chrono::high_resolution_clock::now();
	}

    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
	bool m_resetVelocity;
};

class PositionFilterComplimentaryOpticalIMU : public PositionFilter
{
public:
	PositionFilterComplimentaryOpticalIMU()
		: PositionFilter()
	{
		lastOpticalFrame = std::chrono::high_resolution_clock::now();
	}

    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
};

class PositionFilterKalman : public PositionFilter
{
public:
	PositionFilterKalman()
		: PositionFilter()
		, m_resetVelocity(false)
	{
		kal_pos_err_estimate = Eigen::Vector3f::Zero();
		kal_pos_current_estimate = Eigen::Vector3f::Zero();
		kal_pos_gain = Eigen::Vector3f::Zero();
		kal_pos_err_estimate = Eigen::Vector3f::Zero();
		kal_vel_err_estimate = Eigen::Vector3f::Zero();
		kal_vel_current_estimate = Eigen::Vector3f::Zero();
		lastOpticalFrame = std::chrono::high_resolution_clock::now();
	}

	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
	bool m_resetVelocity;

private:
	Eigen::Vector3f kal_pos_err_estimate;
	Eigen::Vector3f kal_pos_current_estimate;
	Eigen::Vector3f kal_pos_gain;
	Eigen::Vector3f kal_vel_err_estimate;
	Eigen::Vector3f kal_vel_current_estimate;
	Eigen::Vector3f kal_vel_gain;
};


#endif // POSITION_FILTER_H
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

class PositionFilterLowPassIMU : public PositionFilter
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
};

class PositionFilterLowPassExponential : public PositionFilter
{
	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	std::list<float> deltaTimeHistory;
	std::list<Eigen::Vector3f> blendedPositionHistory;
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
		kal_err_estimate[0] = 0.0;
		kal_err_estimate[1] = 0.0;
		kal_err_estimate[2] = 0.0;

		kal_current_estimate = Eigen::Vector3f::Zero();
		kal_gain = Eigen::Vector3f::Zero();
		lastOpticalFrame = std::chrono::high_resolution_clock::now();
	}

	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
	bool m_resetVelocity;

private:
	float kal_err_estimate[3];
	Eigen::Vector3f kal_current_estimate;
	Eigen::Vector3f kal_gain;
};


#endif // POSITION_FILTER_H
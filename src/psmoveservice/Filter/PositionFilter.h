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
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	std::list<Eigen::Vector3f> blendedPositionHistory;
	std::list<int> historyQueueLenght;
	t_high_resolution_timepoint lastOpticalFrame;
};

class PositionFilterLowPassOptical : public PositionFilter
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	std::list<Eigen::Vector3f> blendedPositionHistory;
	std::list<int> historyQueueLenght;
	t_high_resolution_timepoint lastOpticalFrame;
};

class PositionFilterLowPassIMU : public PositionFilter
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
};

class PositionFilterLowPassExponential : public PositionFilter
{
public:
	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	std::list<float> deltaTimeHistory;
	std::list<Eigen::Vector3f> blendedPositionHistory;
};

class PositionFilterComplimentaryOpticalIMU : public PositionFilter
{
public:
    void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
	t_high_resolution_timepoint lastOpticalFrame;
};

#endif // POSITION_FILTER_H
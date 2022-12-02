#ifndef OPTICAL_ORIENTATION_FILTER_H
#define OPTICAL_ORIENTATION_FILTER_H

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

#ifdef WIN32
#include <windows.h> 
#include <stdio.h> 
#include <tchar.h>
#include <strsafe.h>
#include <fstream>
#endif

//-- definitions --
/// Abstract base class for all orientation only filters
class OpticalOrientationFilter : public IOrientationFilter
{
public:
	OpticalOrientationFilter();
	~OpticalOrientationFilter();

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
	Eigen::Vector3f getAngularVelocityRadPerSec() const override;
	Eigen::Vector3f getAngularAccelerationRadPerSecSqr() const override;

protected:
	Eigen::Quaternionf getCombinedOrientation() const;

	OrientationFilterConstants m_constants;
	struct OpticalOrientationFilterState *m_state;
};

/// Just use the optical orientation passed in unfiltered
class OrientationTargetOpticalARG : public OpticalOrientationFilter
{
public:
	OrientationTargetOpticalARG()
		: OpticalOrientationFilter()
		, mg_weight(1.f)
	{
	}

	void update(const float delta_time, const PoseFilterPacket &packet) override;

protected:
	void UpdateComplementaryMARG(const float delta_time, const PoseFilterPacket &packet);
	void UpdateOpticalTarget(const float delta_time, const PoseFilterPacket &packet);

	float mg_weight;
};

#endif // ORIENTATION_FILTER_H
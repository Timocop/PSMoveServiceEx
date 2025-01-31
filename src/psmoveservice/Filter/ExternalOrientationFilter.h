#ifndef EXTERNAL_ORIENTATION_FILTER_H
#define EXTERNAL_ORIENTATION_FILTER_H

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

#include <list>

#ifdef WIN32
#include <windows.h> 
#include <stdio.h> 
#include <tchar.h>
#include <strsafe.h>
#include <fstream>
#endif

//-- definitions --
/// Abstract base class for all orientation only filters
class ExternalOrientationFilter : public IOrientationFilter
{
public:
	ExternalOrientationFilter();
	~ExternalOrientationFilter();

	//-- IStateFilter --
	bool getIsStateValid() const override;
	double getTimeInSeconds() const override;
	void resetState() override;
	void recenterOrientation(const Eigen::Quaternionf& q_pose) override;

	// -- IOrientationFilter --
	bool init(const OrientationFilterConstants &constant) override;
	bool init(const OrientationFilterConstants &constant, const  Eigen::Quaternionf &initial_orientation) override;

	Eigen::Quaternionf getOrientation(float time = 0.f, float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f, float offset_world_x = 0.f, float offset_world_y = 0.f, float offset_world_z = 0.f) const override;
	Eigen::Quaternionf getResetOrientation() const override;
	Eigen::Vector3f getAngularVelocityRadPerSec(float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f) const override;
	Eigen::Vector3f getAngularAccelerationRadPerSecSqr(float offset_x = 0.f, float offset_y = 0.f, float offset_z = 0.f) const override;

protected:
	OrientationFilterConstants m_constants;
	struct ExternalOrientationFilterState *m_state;
	struct OrientationInterpolationState *m_interpolationState;

	HANDLE orientationPipe;
	char pipeBuffer[128];
	bool showMessage;

	std::list<Eigen::Quaternionf> blendedPositionHistory;

#ifdef WIN32
	_locale_t localeInvariant;
#endif
};

/// Just use the optical orientation passed in unfiltered
class OrientationFilterExternal : public ExternalOrientationFilter
{
public:
	void update(const t_high_resolution_timepoint timestamp, const PoseFilterPacket &packet) override;
};

#endif // ORIENTATION_FILTER_H

#ifndef POSITION_EXTERNAL_ATTACHMENT_FILTER_H
#define POSITION_EXTERNAL_ATTACHMENT_FILTER_H

//-- includes -----
#include "PoseFilterInterface.h"
#include <chrono>
#include <list>

#ifdef WIN32
#include <windows.h> 
#include <stdio.h> 
#include <tchar.h>
#include <strsafe.h>
#include <fstream>
#endif

//-- definitions -----
/// Abstract base class for all orientation only filters
class PositionExternalAttachmentFilter : public IPositionFilter
{
public:
	PositionExternalAttachmentFilter();
    ~PositionExternalAttachmentFilter();

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
    struct ExternalAttachmentPositionFilterState *m_state;

	HANDLE attachmentPipe;
	char pipeBuffer[512];
	bool showMessage;

#ifdef WIN32
	_locale_t localeInvariant;
#endif

};

/// Just use the optical orientation passed in unfiltered
class PositionFilterExternalAttachment : public PositionExternalAttachmentFilter
{
public:
    void update(const float delta_time, const PoseFilterPacket &packet) override;
	std::list<Eigen::Vector3f> blendedPositionHistory;
};


#endif // POSITION_FILTER_H
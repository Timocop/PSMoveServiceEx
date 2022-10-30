#ifndef APP_STAGE_OPTICAL_RECENTER_H
#define APP_STAGE_OPTICAL_RECENTER_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"
#include "AppStage_TrackerSettings.h"

#include <deque>
#include <chrono>

//-- pre-declarations -----

//-- definitions -----
class AppStage_OpticalRecenter : public AppStage
{
public:
	AppStage_OpticalRecenter(class App *app);
    virtual ~AppStage_OpticalRecenter();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

	inline void setTargetControllerId(int iIndex)
	{
		m_iControllerId = iIndex;
	}

	inline void setTargetTrackerSettings(AppStage_TrackerSettings *tracker_settings)
	{
		m_tracker_settings = tracker_settings;
	}

protected:
	enum eCalibrationMenuState
	{
		inactive,

		pendingTrackerListRequest,
		failedTrackerListRequest,
		waitingForStreamStartResponse,
		failedStreamStart,
		waitForStablePosition,
		measureOpticalPosition,
		waitForStableOrientation,
		measureOpticalOrientation,
		measureComplete,
		test
	};

	void setState(eCalibrationMenuState newState);
	void onExitState(eCalibrationMenuState newState);
	void onEnterState(eCalibrationMenuState newState);

	void request_tracker_list();
	static void handle_tracker_list_response(
		const PSMResponseMessage *response_message,
		void *userdata);
    static void handle_acquire_controller(
        const PSMResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);

	void drawTrackerListSelected(const PSMClientTrackerInfo *trackerList, const int trackerCount, const bool bIsTracking);

private:

    eCalibrationMenuState m_menuState;
	AppStage_TrackerSettings *m_tracker_settings;
	int m_iControllerId;

    PSMController *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;

    PSMVector3f m_lastMulticamPositionCm;
	PSMQuatf m_lastMulticamOrientation;
	PSMPosef m_lastControllerPose;
	bool m_bLastMulticamPositionValid;
	bool m_bLastMulticamOrientationValid;

	PSMVector3f m_CenterSample;
	PSMVector3f m_ForwardSample;

	PSMTrackerList m_trackerList;
};

#endif // APP_STAGE_OPTICAL_CALIBRATION_H
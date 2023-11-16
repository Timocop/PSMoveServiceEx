#ifndef APP_STAGE_ACCELEROMETER_CALIBRATION_H
#define APP_STAGE_ACCELEROMETER_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"

#include <deque>
#include <chrono>

//-- pre-declarations -----
struct AccelerometerStatistics;
struct EigenFitEllipsoid;

//-- definitions -----
class AppStage_AccelerometerCalibration : public AppStage
{
public:
    AppStage_AccelerometerCalibration(class App *app);
    virtual ~AppStage_AccelerometerCalibration();

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    inline void setBypassCalibrationFlag(bool bFlag)
    {
        m_bBypassCalibration = bFlag;
    }

protected:
	void request_playspace_info();
	static void handle_playspace_info_response(
		const PSMResponseMessage *response_message,
		void *userdata);

    static void handle_acquire_controller(
        const PSMResponseMessage *response,
        void *userdata);
    void request_exit_to_app_stage(const char *app_stage_name);

private:
    enum eCalibrationMenuState
    {
        inactive,

		pendingPlayspaceRequest,
        waitingForStreamStartResponse,
        failedStreamStart,
        placeController,
        measureNoise,
        measureComplete,
        test
    };
    eCalibrationMenuState m_menuState;

	enum eTestMode 
	{
		controllerRelative,
		worldRelative
	};
	eTestMode m_testMode;

    bool m_bBypassCalibration;

    PSMController *m_controllerView;
    bool m_isControllerStreamActive;
    int m_lastControllerSeqNum;
	float m_playspaceYawOffset;

	PSMVector3i m_lastRawAccelerometer;
	PSMVector3f m_lastCalibratedAccelerometer;

	struct ControllerAccelerometerPoseSamples *m_noiseSamples;
};

#endif // APP_STAGE_ACCELEROMETER_CALIBRATION_H
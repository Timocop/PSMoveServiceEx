#ifndef APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H
#define APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H

//-- includes -----
#include "AppStage.h"
#include "ClientGeometry_CAPI.h"
#include "PSMoveClient_CAPI.h"

#include <deque>
#include <chrono>

//-- definitions -----
class AppStage_HMDGyroscopeCalibration : public AppStage
{
public:
	AppStage_HMDGyroscopeCalibration(class App *app);
    virtual ~AppStage_HMDGyroscopeCalibration();

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

    void request_set_gyroscope_calibration(
		const PSMVector3f &raw_bias,
		const float raw_drift, 
		const float raw_variance);
    static void handle_acquire_hmd(
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
        waitForStable,
        measureBiasAndDrift,
        measureComplete,
        test
    };
    eCalibrationMenuState m_menuState;
    bool m_bBypassCalibration;

    PSMHeadMountedDisplay *m_hmdView;
    bool m_isHMDStreamActive;
    int m_lastHMDSeqNum;
	float m_playspaceYawOffset;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastSampleTime;
    bool m_bLastSampleTimeValid;

    PSMVector3i m_lastRawGyroscope;
    PSMVector3f m_lastCalibratedGyroscope;
    PSMVector3f m_lastCalibratedAccelerometer;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_stableStartTime;
    bool m_bIsStable;

    struct GyroscopeNoiseSamples *m_gyroNoiseSamples;
};

#endif // APP_STAGE_HMD_GYROSCOPE_CALIBRATION_H
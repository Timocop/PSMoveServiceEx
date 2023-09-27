#ifndef APP_STAGE_TEST_TRACKER_H
#define APP_STAGE_TEST_TRACKER_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

#include <vector>

//-- definitions -----
class AppStage_TestTracker : public AppStage
{
public:
    AppStage_TestTracker(class App *app);

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

    void request_tracker_start_stream();
    void request_tracker_stop_stream();

protected:
    static void handle_tracker_start_stream_response(
        const PSMResponseMessage *response,
        void *userdata);

    static void handle_tracker_stop_stream_response(
        const PSMResponseMessage *response,
        void *userdata);

	void request_tracker_set_exposure();
	void request_tracker_set_gain();

	static void handle_tracker_set_gain_response(
		const PSMResponseMessage * response, 
		void * userdata);
	static void handle_tracker_set_exposure_response(
		const PSMResponseMessage * response, 
		void * userdata);

	void request_tracker_reset_exposure_gain();
	void request_tracker_reset_exposure();
	void request_tracker_reset_gain();

	static void handle_tracker_reset_gain_response(
		const PSMResponseMessage * response, 
		void * userdata);
	static void handle_tracker_reset_exposure_response(
		const PSMResponseMessage * response, 
		void * userdata);

	void request_tracker_get_settings();
	static void handle_tracker_get_settings_response(
		const PSMResponseMessage * response, 
		void * userdata);

private:
    enum eTrackerMenuState
    {
        inactive,
        idle,

        pendingTrackerStartStreamRequest,
        failedTrackerStartStreamRequest,

		pendingTrackerGetSettings,
		failedTrackerGetSettings,

		pendingTrackerSetExposureGain,
		failedTrackerSetExposureGain,

		pendingTrackerResetExposureGain,
		failedTrackerResetExposureGain,

        pendingTrackerStopStreamRequest,
        failedTrackerStopStreamRequest,
    };

    eTrackerMenuState m_menuState;
    bool m_bStreamIsActive;
	bool m_bChangedExposure;
	bool m_bChangedGain;
    PSMTracker *m_tracker_view;
    class TextureAsset *m_video_texture;
	int m_streamFps;
	int m_displayFps;
	std::chrono::time_point<std::chrono::high_resolution_clock> m_lastStreamFps;
	float m_trackerExposure;
	float m_trackerGain;
	float m_trackerFrameRate;
};

#endif // APP_STAGE_TEST_TRACKER_H
#ifndef APP_STAGE_HMD_SETTINGS_H
#define APP_STAGE_HMD_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include <vector>
#include <string>

//-- definitions -----
class AppStage_HMDSettings : public AppStage
{
public:
	enum AdaptiveDriftCorrectionMethod
	{
		AdaptiveNone = 0,
		AdaptiveGyro,
		AdaptiveAccel,
		AdaptiveBoth,
	};

    enum eHMDType
    {
        Morpheus,
        VirtualHMD
    };

	struct DevicePosition
	{
		float x;
		float y;
		float z;
	};

	struct DeviceOrientation
	{
		float x;
		float y;
		float z;
	};

	struct OffsetSettings
	{
		DeviceOrientation offset_world_orientation;
		DeviceOrientation offset_orientation;
		DevicePosition offset_position;
		DevicePosition offset_scale;
	};

	struct FilterSettings
	{
		float filter_lowpassoptical_distance;
		float filter_lowpassoptical_smoothing;
		float filter_madgwick_beta;
		bool filter_madgwick_stabilization;
		float filter_madgwick_stabilization_min_beta;
		float filter_madgwick_stabilization_smoothing_factor;
		float filter_velocity_smoothing_factor;
		float filter_angular_smoothing_factor;
		float filter_velocity_prediction_cutoff;
		float filter_angular_prediction_cutoff;
		float filter_position_kalman_error;
		float filter_position_kalman_noise;
		bool filter_position_kalman_disable_cutoff;
		bool filter_madgwick_smart_correct;
	};

    struct HMDInfo
    {
        int HmdID;
        eHMDType HmdType;
        std::string DevicePath;
        PSMTrackingColorType TrackingColorType;
		int PositionFilterIndex;
		std::string PositionFilterName;
		int OrientationFilterIndex;
		std::string OrientationFilterName;
		float PredictionTime;
		float AngPredictionTime;

		DeviceOrientation OffsetOrientation;
		DeviceOrientation OffsetWorldOrientation;
		DevicePosition OffsetPosition;
		DevicePosition OffsetScale;

		float FilterLowPassOpticalDistance;
		float FilterLowPassOpticalSmoothing;
		float FilterMadgwickBeta;
		bool FilterMadgwickStabilization;
		float FilterMadgwickStabilizationMinBeta;
		float FilterMadgwickStabilizationSmoothingFactor;
		float FilterVelocitySmoothingFactor;
		float FilterAngularSmoothingFactor;
		float FilterVelocityPredictionCutoff;
		float FilterAngularPredictionCutoff;
		float FilterPositionKalmanError;
		float FilterPositionKalmanNoise;
		bool FilterPositionKalmanDisableCutoff;
		bool FilterMadgwickSmartCorrect;
    };


    AppStage_HMDSettings(class App *app);

    inline const HMDInfo *getSelectedHmdInfo() const
    {
        return
            (m_selectedHmdIndex != -1)
            ? &m_hmdInfos[m_selectedHmdIndex]
            : nullptr;
    }

    virtual void enter() override;
    virtual void exit() override;
    virtual void update() override;
    virtual void render() override;

    virtual void renderUI() override;

    static const char *APP_STAGE_NAME;

protected:
    virtual bool onClientAPIEvent(
        PSMEventMessage::eEventType event, 
        PSMEventDataHandle opaque_event_handle) override;

    void request_hmd_list();
	static void handle_hmd_list_response(
		const PSMResponseMessage *response,
		void *userdata);

	void show_position_filter_tooltip(const std::string name);

	void show_orientation_filter_tooltip(const std::string name);

	void request_set_orientation_filter(const int hmd_id, const std::string &filter_name);
	void request_set_position_filter(const int hmd_id, const std::string &filter_name);
	void request_set_hmd_prediction(const int hmd_id, float prediction_time);
	void request_set_hmd_angular_prediction(const int hmd_id, float prediction_time);
	void request_set_hmd_tracking_color_id(
		const int hmd_id,
		PSMTrackingColorType tracking_color_type);
	void request_set_hmd_filter_settings(
		const int HmdID,
		FilterSettings filterSettings);
	void request_set_hmd_offsets(
		int HmdID,
		OffsetSettings offset_settings);


private:
    enum eHmdMenuState
    {
        inactive,
        idle,

        pendingHmdListRequest,
        failedHmdListRequest,
    };
    eHmdMenuState m_menuState;

    std::vector<HMDInfo> m_hmdInfos;

    int m_selectedHmdIndex;
};

#endif // APP_STAGE_HMD_SETTINGS_H

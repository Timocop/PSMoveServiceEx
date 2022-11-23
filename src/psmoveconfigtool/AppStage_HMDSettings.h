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

		DeviceOrientation OffsetOrientation;
		DevicePosition OffsetPosition;
		DevicePosition OffsetScale;

		float FilterPredictionDistance;
		float FilterPredictionSmoothing;
		float FilterLowPassOpticalDistance;
		float FilterLowPassOpticalSmoothing;
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

	void request_set_orientation_filter(const int hmd_id, const std::string &filter_name);
	void request_set_position_filter(const int hmd_id, const std::string &filter_name);
	void request_set_hmd_prediction(const int hmd_id, float prediction_time);
	void request_set_hmd_tracking_color_id(
		const int hmd_id,
		PSMTrackingColorType tracking_color_type);
	void request_set_hmd_filter_settings(
		const int HmdID,
		float filter_prediction_distance,
		float filter_prediction_smoothing,
		float filter_lowpassoptical_distance,
		float filter_lowpassoptical_smoothing);
	void request_set_hmd_offsets(
		int ControllerID,
		float offset_orientation_x,
		float offset_orientation_y,
		float offset_orientation_z,
		float offset_position_x,
		float offset_position_y,
		float offset_position_z,
		float offset_scale_x,
		float offset_scale_y,
		float offset_scale_z);


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

#ifndef APP_STAGE_CONTROLLER_SETTINGS_H
#define APP_STAGE_CONTROLLER_SETTINGS_H

//-- includes -----
#include "AppStage.h"
#include "PSMoveClient_CAPI.h"

#include <vector>
#include <string>

#define MAX_GAMEPAD_LABELS  16+1

//-- definitions -----
class AppStage_ControllerSettings : public AppStage
{
public:
	enum AdaptiveDriftCorrectionMethod
	{
		AdaptiveNone = 0,
		AdaptiveGyro,
		AdaptiveAccel,
		AdaptiveBoth,
	};

	enum PassiveDriftCorrectionMethod
	{
		StableGravity = 0,
		StableGyroAccel,
		StableBoth,
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
		float offset_magnetometer;
	};

	struct FilterSettings
	{
		float filter_lowpassoptical_distance;
		float filter_lowpassoptical_smoothing;
		bool filter_enable_magnetometer;
		bool filter_use_passive_drift_correction;
		PassiveDriftCorrectionMethod filter_passive_drift_correction_method;
		float filter_passive_drift_correction_deadzone;
		float filter_passive_drift_correction_gravity_deadzone;
		float filter_passive_drift_correction_delay;
		bool filter_use_stabilization;
		float filter_stabilization_min_scale;
		float filter_madgwick_beta;
		bool filter_madgwick_stabilization;
		float filter_madgwick_stabilization_min_beta;
	};

	struct ControllerInfo
    {
        int ControllerID;
		int FirmwareVersion;
		int FirmwareRevision;
		int AssignedParentControllerIndex;
		std::string AssignedParentControllerSerial;		
		std::vector<std::string> PotentialParentControllerSerials;
        PSMControllerType ControllerType;
		PSMControllerHand ControllerHand;
        PSMTrackingColorType TrackingColorType;
        std::string DevicePath;
        std::string DeviceSerial;
        std::string AssignedHostSerial;
        bool IsBluetooth;
		bool HasMagnetometer;
		int PositionFilterIndex;
		std::string PositionFilterName;
		int OrientationFilterIndex;
		std::string OrientationFilterName;
		int GyroGainIndex;
		std::string GyroGainSetting;
		float PredictionTime;
		float AngPredictionTime;
        int GamepadIndex;
		bool OpticalTracking;
		bool PSmoveEmulation;

		DeviceOrientation OffsetOrientation;
		DeviceOrientation OffsetWorldOrientation;
		DevicePosition OffsetPosition;
		DevicePosition OffsetScale;
		float OffsetMagnetometer;

		float FilterLowPassOpticalDistance;
		float FilterLowPassOpticalSmoothing;
		bool FilterEnableMagnetometer;
		bool FilterUsePassiveDriftCorrection;
		PassiveDriftCorrectionMethod FilterPassiveDriftCorrectionMethod;
		float FilterPassiveDriftCorrectionDeazone;
		float FilterPassiveDriftCorrectionGravityDeazone;
		float FilterPassiveDriftCorrectionDelay;
		bool FilterUseStabilization;
		float FilterStabilizationMinScale;
		float FilterMadgwickBeta;
		bool FilterMadgwickStabilization;
		float FilterMadgwickStabilizationMinBeta;

		static bool ParentControllerComboItemGetter(void* userdata, int index, const char** out_string)
		{
			const ControllerInfo *this_ptr= reinterpret_cast<ControllerInfo *>(userdata);
			const int item_count= static_cast<int>(this_ptr->PotentialParentControllerSerials.size());
			
			if (index >= 0 && index < item_count)
			{
				*out_string= this_ptr->PotentialParentControllerSerials[index].c_str();
				return true;
			}
			else
			{
				*out_string= "<INVALID>";
				return false;
			}
		}

		static bool GamepadIndexComboItemGetter(void* userdata, int index, const char** out_string)
		{
			const AppStage_ControllerSettings *this_ptr= reinterpret_cast<AppStage_ControllerSettings *>(userdata);            
			
			if (index >= 0 && index <= this_ptr->m_gamepadCount && index < MAX_GAMEPAD_LABELS)
			{
                *out_string= AppStage_ControllerSettings::GAMEPAD_COMBO_LABELS[index];
				return true;
			}
			else
			{
				*out_string= "<NONE>";
				return false;
			}
		}
    };

    AppStage_ControllerSettings(class App *app);

    inline const ControllerInfo *getSelectedControllerInfo() const
    { 
        return 
            (m_selectedControllerIndex != -1) 
            ? &m_controllerInfos[m_selectedControllerIndex] 
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

    void request_controller_list();
    static void handle_controller_list_response(
        const PSMResponseMessage *response_message,
        void *userdata);
	void request_set_orientation_filter(const int controller_id, const std::string &filter_name);
	void request_set_position_filter(const int controller_id, const std::string &filter_name);
	void request_set_gyroscope_gain_setting(const int controller_id, const std::string& gain_setting);
	void request_set_controller_prediction(const int controller_id, float prediction_time);
	void request_set_controller_angular_prediction(const int controller_id, const float prediction_time);
	void request_set_controller_opticaltracking(const int controller_id, bool enabled);
	void request_set_controller_psmove_emulation(const int controller_id, bool enabled);
    void request_set_controller_gamepad_index(const int controller_id, const int gamepad_index);
	void request_set_controller_hand(const int controller_id, const PSMControllerHand controller_hand);
	void request_set_controller_filter_settings(
		const int controller_id,
		FilterSettings filterSettings
	);

	int find_controller_id_by_serial(std::string parent_controller_serial) const;

	void request_set_controller_tracking_color_id(
		int ControllerID,
		PSMTrackingColorType tracking_color_type);
	void request_set_parent_controller_id(
		int ControllerID,
		int ParentControllerID);
	void request_set_controller_offsets(
		int ControllerID,
		OffsetSettings offset_settings);

	void show_position_filter_tooltip(const std::string name);
	void show_orientation_filter_tooltip(const std::string name);

private:
    enum eControllerMenuState
    {
        inactive,
        idle,

        pendingControllerListRequest,
        failedControllerListRequest,
    };
    eControllerMenuState m_menuState;

    std::vector<ControllerInfo> m_controllerInfos;
    std::string m_hostSerial;
    int m_gamepadCount;

	std::vector<PSMControllerID> m_controllersStreams;
    int m_selectedControllerIndex;
    
    static const char *GAMEPAD_COMBO_LABELS[MAX_GAMEPAD_LABELS];
};

#endif // APP_STAGE_SELECT_CONTROLLER_H

// -- includes -----
#include "PSMoveClient_CAPI.h"
#include "PSMoveClient.h"
#include "ClientLog.h"
#include "ClientNetworkInterface.h"
#include "MathUtility.h"
#include "ProtocolVersion.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include <map>
#include <assert.h>

#ifdef _MSC_VER
	#pragma warning(disable:4996)  // ignore strncpy warning
#endif

// -- macros -----
#define IS_VALID_CONTROLLER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_CONTROLLER_COUNT)
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSMOVESERVICE_MAX_HMD_COUNT)

#define COPY_PROP(x) (out.x = state->x)

// -- constants ----
const PSMVector3f k_identity_gravity_calibration_direction= {0.f, 1.f, 0.f};

// -- private data ---
PSMoveClient *g_psm_client= nullptr;

// -- private definitions -----
class PSMCallbackTimeout
{
public:
    PSMCallbackTimeout(int timeout_ms) 
        : m_startTime(std::chrono::high_resolution_clock::now())
        , m_duration(static_cast<float>(timeout_ms))
    {        
    }

    bool HasElapsed() const
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
        std::chrono::duration<float, std::milli> timeSinceStart= now - m_startTime;

        return timeSinceStart > m_duration;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
    std::chrono::duration<float, std::milli> m_duration;
};

class PSMBlockingRequest
{
public:
	PSMBlockingRequest(PSMRequestID req_id) 
		: m_request_id(req_id)
		, m_bReceived(false)
	{
		memset(&m_response, 0, sizeof(PSMResponseMessage));
	}

	const PSMResponseMessage &get_response_message() const
	{
		return m_response;
	}

	PSMResponseMessage::eResponsePayloadType get_response_payload_type() const
	{
		return m_response.payload_type;
	}

	PSMResult send(int timeout_ms)
	{
		PSMResult result= PSMResult_Error;

		if (m_request_id != PSM_INVALID_REQUEST_ID)
		{
			PSMCallbackTimeout timeout(timeout_ms);

            assert(g_psm_client != nullptr);
			g_psm_client->register_callback(m_request_id, PSMBlockingRequest::response_callback, this);
    
			while (!m_bReceived && !timeout.HasElapsed())
			{
				_PAUSE(10);

				// Process responses, events and controller updates from the service
				PSM_Update();
			}

			if (timeout.HasElapsed())
			{
				g_psm_client->cancel_callback(m_request_id);
				result= PSMResult_Timeout;
			}
			else if (m_response.result_code == PSMResult_Success)
			{
				result= PSMResult_Success;
			}
		}

		return result;
	}
   
protected:
    static void response_callback(const PSMResponseMessage *response, void *userdata)
    {
        PSMBlockingRequest *result= reinterpret_cast<PSMBlockingRequest *>(userdata);
        
        result->m_response= *response;
        result->m_bReceived= true;
    }

protected:
	// Request
	PSMRequestID m_request_id;

	// Response
    bool m_bReceived;
    PSMResponseMessage m_response;
};

// -- public interface -----
const char* PSM_GetClientVersionString()
{
    const char *version_string= PSM_PROTOCOL_VERSION_STRING;

    return version_string;
}

bool PSM_GetIsInitialized()
{
	return g_psm_client != nullptr;
}

bool PSM_GetIsConnected()
{
    return g_psm_client != nullptr && g_psm_client->getIsConnected();
}

bool PSM_HasConnectionStatusChanged()
{
	return g_psm_client != nullptr && g_psm_client->pollHasConnectionStatusChanged();
}

bool PSM_HasControllerListChanged()
{
	return g_psm_client != nullptr && g_psm_client->pollHasControllerListChanged();
}

bool PSM_HasTrackerListChanged()
{
	return g_psm_client != nullptr && g_psm_client->pollHasTrackerListChanged();
}

bool PSM_HasHMDListChanged()
{
	return g_psm_client != nullptr && g_psm_client->pollHasHMDListChanged();
}

bool PSM_WasSystemButtonPressed()
{
	return g_psm_client != nullptr && g_psm_client->pollWasSystemButtonPressed();
}

bool PSM_HasPlayspaceOffsetChanged()
{
	return g_psm_client != nullptr && g_psm_client->pollHasPlayspaceOffsetsChanged();
}

PSMResult PSM_Initialize(const char* host, const char* port, int timeout_ms)
{
    PSMResult result = PSMResult_Error;

    if (PSM_InitializeAsync(host, port) != PSMResult_Error)
    {
        PSMCallbackTimeout timeout(timeout_ms);

        while (!g_psm_client->pollHasConnectionStatusChanged() && !timeout.HasElapsed())
        {
            _PAUSE(10);
			g_psm_client->update();
			g_psm_client->process_messages();
        }

        if (!timeout.HasElapsed())
        {
            result= g_psm_client->getIsConnected() ? PSMResult_Success : PSMResult_Error;
        }
        else
        {
            result= PSMResult_Timeout;
        }
    }

    return result;
}

PSMResult PSM_InitializeAsync(const char* host, const char* port)
{
	PSMResult result= PSMResult_Error;

	if (g_psm_client == nullptr || !g_psm_client->getIsConnected())
	{
		if (g_psm_client == nullptr)
		{
			std::string s_host(host);
			std::string s_port(port);

			g_psm_client= new PSMoveClient(s_host, s_port);
		}

		if (g_psm_client->startup(_log_severity_level_info))
		{
			result= PSMResult_RequestSent;
		}
		else
		{
			delete g_psm_client;
			g_psm_client= nullptr;
			result= PSMResult_Error;
		}
	}
	else
	{
		result= PSMResult_Success;
	}

    return result;
}

PSMResult PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
	    PSMBlockingRequest request(g_psm_client->get_service_version());
        result_code= request.send(timeout_ms);

        if (result_code == PSMResult_Success)
        {
            assert(request.get_response_payload_type() == PSMResponseMessage::_responsePayloadType_ServiceVersion);
        
		    const char* version_string= request.get_response_message().payload.service_version.version_string;
		    strncpy(out_version_string, version_string, max_version_string);
        }
    }
    
    return result_code;
}

PSMResult PSM_GetServiceVersionStringAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMRequestID req_id = g_psm_client->get_service_version();

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_Shutdown()
{
	PSMResult result= PSMResult_Error;

	if (g_psm_client != nullptr)
	{
		g_psm_client->shutdown();

		delete g_psm_client;
		g_psm_client= nullptr;

		result= PSMResult_Success;
	}

    return result;
}

PSMResult PSM_Update()
{
    PSMResult result = PSMResult_Error;

    if (PSM_UpdateNoPollMessages() == PSMResult_Success)
    {
		// Process all events and responses
		// Any incoming events become status flags we can poll (ex: pollHasConnectionStatusChanged)
		g_psm_client->process_messages();

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_UpdateNoPollMessages()
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        g_psm_client->update();

        result= PSMResult_Success;
    }

    return result;
}

PSMController *PSM_GetController(PSMControllerID controller_id)
{
	return (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id)) ? g_psm_client->get_controller_view(controller_id) : nullptr;
}

PSMResult PSM_GetControllerEx(PSMControllerID controller_id, PSMControllerEx *controller_out)
{
	PSMResult result = PSMResult_Error;
	 
	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *state = g_psm_client->get_controller_view(controller_id); 
		PSMControllerEx out = {};

		COPY_PROP(ControllerID);
		COPY_PROP(ControllerType);
		COPY_PROP(ControllerHand);

		COPY_PROP(bValid);
		COPY_PROP(OutputSequenceNum);
		COPY_PROP(InputSequenceNum);
		COPY_PROP(IsConnected);
		COPY_PROP(DataFrameLastReceivedTime);
		COPY_PROP(DataFrameAverageFPS);
		COPY_PROP(ListenerCount);

		*controller_out = out;
		result = PSMResult_Success;
	}

	return result;
}

PSMResult PSM_GetControllerPSMoveState(PSMControllerID controller_id, PSMPSMove *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Move)
		{
			*controller_out = controller->ControllerState.PSMoveState;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPSMoveStateEx(PSMControllerID controller_id, PSMPSMoveEx *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Move)
		{
			const PSMPSMove *state = &controller->ControllerState.PSMoveState; 
			PSMPSMoveEx out = {};
			
			COPY_PROP(bHasValidHardwareCalibration);
			COPY_PROP(bIsTrackingEnabled);
			COPY_PROP(bIsCurrentlyTracking);
			COPY_PROP(bIsOrientationValid);
			COPY_PROP(bIsPositionValid);
			COPY_PROP(bHasUnpublishedState);

			memcpy(out.DevicePath, state->DevicePath, sizeof(PSMPSMoveEx::DevicePath));
			memcpy(out.DeviceSerial, state->DeviceSerial, sizeof(PSMPSMoveEx::DeviceSerial));
			memcpy(out.AssignedHostSerial, state->AssignedHostSerial, sizeof(PSMPSMoveEx::AssignedHostSerial));

			COPY_PROP(PairedToHost);
			COPY_PROP(ConnectionType);
			COPY_PROP(TrackingColorType);

			COPY_PROP(TriangleButton);
			COPY_PROP(CircleButton);
			COPY_PROP(CrossButton);
			COPY_PROP(SquareButton);
			COPY_PROP(SelectButton);
			COPY_PROP(StartButton);
			COPY_PROP(PSButton);
			COPY_PROP(MoveButton);
			COPY_PROP(TriggerButton);
			COPY_PROP(BatteryValue);
			COPY_PROP(TriggerValue);
			COPY_PROP(Rumble);
			COPY_PROP(LED_r);
			COPY_PROP(LED_g);
			COPY_PROP(LED_b);

			COPY_PROP(ResetPoseButtonPressTime);
			COPY_PROP(bResetPoseRequestSent);
			COPY_PROP(bPoseResetButtonEnabled);

			*controller_out = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPSNaviState(PSMControllerID controller_id, PSMPSNavi *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Navi)
		{
			*controller_out = controller->ControllerState.PSNaviState;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerDualShock4State(PSMControllerID controller_id, PSMDualShock4 *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_DualShock4)
		{
			*controller_out = controller->ControllerState.PSDS4State;

			result = PSMResult_Success;
		}

	}

	return result;
}

PSMResult PSM_GetControllerDualShock4StateEx(PSMControllerID controller_id, PSMDualShock4Ex *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_DualShock4)
		{
			const PSMDualShock4 *state = &controller->ControllerState.PSDS4State; 
			PSMDualShock4Ex out = {};

			COPY_PROP(bHasValidHardwareCalibration);
			COPY_PROP(bIsTrackingEnabled);
			COPY_PROP(bIsCurrentlyTracking);
			COPY_PROP(bIsOrientationValid);
			COPY_PROP(bIsPositionValid);
			COPY_PROP(bHasUnpublishedState);

			memcpy(out.DevicePath, state->DevicePath, sizeof(PSMDualShock4Ex::DevicePath));
			memcpy(out.DeviceSerial, state->DeviceSerial, sizeof(PSMDualShock4Ex::DeviceSerial));
			memcpy(out.AssignedHostSerial, state->AssignedHostSerial, sizeof(PSMDualShock4Ex::AssignedHostSerial));

			COPY_PROP(PairedToHost);
			COPY_PROP(ConnectionType);
			COPY_PROP(TrackingColorType);

			COPY_PROP(DPadUpButton);
			COPY_PROP(DPadDownButton);
			COPY_PROP(DPadLeftButton);
			COPY_PROP(DPadRightButton);

			COPY_PROP(SquareButton);
			COPY_PROP(CrossButton);
			COPY_PROP(CircleButton);
			COPY_PROP(TriangleButton);

			COPY_PROP(L1Button);
			COPY_PROP(R1Button);
			COPY_PROP(L2Button);
			COPY_PROP(R2Button);
			COPY_PROP(L3Button);
			COPY_PROP(R3Button);

			COPY_PROP(ShareButton);
			COPY_PROP(OptionsButton);

			COPY_PROP(PSButton);
			COPY_PROP(TrackPadButton);

			COPY_PROP(LeftAnalogX);
			COPY_PROP(LeftAnalogY);
			COPY_PROP(RightAnalogX);
			COPY_PROP(RightAnalogY);
			COPY_PROP(LeftTriggerValue);
			COPY_PROP(RightTriggerValue);

			COPY_PROP(BigRumble);
			COPY_PROP(BigRumble);
			COPY_PROP(LED_r);
			COPY_PROP(LED_g);
			COPY_PROP(LED_b);

			COPY_PROP(ResetPoseButtonPressTime);
			COPY_PROP(bResetPoseRequestSent);
			COPY_PROP(bPoseResetButtonEnabled);

			*controller_out = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerVirtualControllerState(PSMControllerID controller_id, PSMVirtualController *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Virtual)
		{
			*controller_out = controller->ControllerState.VirtualController;

			result = PSMResult_Success;
		}

	}

	return result;
}

PSMResult PSM_GetControllerVirtualControllerStateEx(PSMControllerID controller_id, PSMVirtualControllerEx *controller_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Virtual)
		{
			const PSMVirtualController *state = &controller->ControllerState.VirtualController;
			PSMVirtualControllerEx out = {};

			COPY_PROP(bIsTrackingEnabled);
			COPY_PROP(bIsCurrentlyTracking);
			COPY_PROP(bIsPositionValid);

			memcpy(out.DevicePath, state->DevicePath, sizeof(PSMVirtualControllerEx::DevicePath));

			COPY_PROP(vendorID);
			COPY_PROP(productID);

			COPY_PROP(numAxes);
			COPY_PROP(numButtons);

			memcpy(out.axisStates, state->axisStates, sizeof(PSMVirtualControllerEx::axisStates));
			memcpy(out.buttonStates, state->buttonStates, sizeof(PSMVirtualControllerEx::buttonStates));

			COPY_PROP(TrackingColorType);

			*controller_out = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMRequestID req_id = g_psm_client->get_controller_list();

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_GetControllerList(PSMControllerList *out_controller_list, int timeout_ms)
{
	PSMResult result_code = PSMResult_Error;

    if (g_psm_client != nullptr)
    {
		PSMBlockingRequest request(g_psm_client->get_controller_list());
		result_code = request.send(timeout_ms);

		if (result_code == PSMResult_Success)
		{
			assert(request.get_response_payload_type() == PSMResponseMessage::_responsePayloadType_ControllerList);

			*out_controller_list = request.get_response_message().payload.controller_list;
			result_code = PSMResult_Success;
		}
    }
    
    return result_code;
}

PSMResult PSM_AllocateControllerListener(PSMControllerID controller_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        if (g_psm_client->allocate_controller_listener(controller_id))
        {
			result= PSMResult_Success;
        }
    }
    
    return result;
}

PSMResult PSM_FreeControllerListener(PSMControllerID controller_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		g_psm_client->free_controller_listener(controller_id);
        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_StartControllerDataStreamAsync(PSMControllerID controller_id, unsigned int data_stream_flags, PSMRequestID *out_request_id)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->start_controller_data_stream(controller_id, data_stream_flags);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result_code= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result_code;
}

PSMResult PSM_StartControllerDataStream(PSMControllerID controller_id, unsigned int data_stream_flags, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		PSMBlockingRequest request(g_psm_client->start_controller_data_stream(controller_id, data_stream_flags));
		result_code= request.send(timeout_ms);
    }

    return result_code;
}

PSMResult PSM_StopControllerDataStreamAsync(PSMControllerID controller_id, PSMRequestID *out_request_id)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->stop_controller_data_stream(controller_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result_code= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result_code;
}

PSMResult PSM_StopControllerDataStream(PSMControllerID controller_id, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		PSMBlockingRequest request(g_psm_client->stop_controller_data_stream(controller_id));

		result_code= request.send(timeout_ms);
    }

    return result_code;
}

PSMResult PSM_SetControllerLEDColorAsync(PSMControllerID controller_id, PSMTrackingColorType tracking_color, PSMRequestID *out_request_id)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->set_led_tracking_color(controller_id, tracking_color);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result_code= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result_code;
}

PSMResult PSM_SetControllerLEDTrackingColor(PSMControllerID controller_id, PSMTrackingColorType tracking_color, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		PSMBlockingRequest request(g_psm_client->set_led_tracking_color(controller_id, tracking_color));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_SetControllerLEDOverrideColor(PSMControllerID controller_id, unsigned char r, unsigned char g, unsigned char b)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                PSMPSMove *psmove= &controller->ControllerState.PSMoveState;

                if (r != psmove->LED_r || g != psmove->LED_g || b != psmove->LED_b)
                {
                    psmove->LED_r = r;
                    psmove->LED_g = g;
                    psmove->LED_b = b;

                    psmove->bHasUnpublishedState = true;
                }
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMDualShock4 *ds4= &controller->ControllerState.PSDS4State;

                if (r != ds4->LED_r || g != ds4->LED_g || b != ds4->LED_b)
                {
                    ds4->LED_r = r;
                    ds4->LED_g = g;
                    ds4->LED_b = b;

                    ds4->bHasUnpublishedState = true;
                }
            } break;
        case PSMController_Virtual:
            break;
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float *out_rumbleFraction)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        unsigned char rumbleByte= 0;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                rumbleByte= controller->ControllerState.PSMoveState.Rumble;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {                
                if (channel == PSMControllerRumbleChannel_Left)
                {
                    rumbleByte= controller->ControllerState.PSDS4State.BigRumble;
                }
                else if (channel == PSMControllerRumbleChannel_Right)
                {
                    rumbleByte= controller->ControllerState.PSDS4State.SmallRumble;
                }
            } break;
        case PSMController_Virtual:
            break;
        }

        *out_rumbleFraction= clampf01(static_cast<float>(rumbleByte / 255.f));
        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_SetControllerRumble(PSMControllerID controller_id, PSMControllerRumbleChannel channel, float rumbleFraction)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {		
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        const unsigned char rumbleByte= static_cast<unsigned char>(clampf01(rumbleFraction)*255.f);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
                PSMPSMove *psmove= &controller->ControllerState.PSMoveState;

                if (psmove->Rumble != rumbleByte)
                {
                    psmove->Rumble = rumbleByte;
                    psmove->bHasUnpublishedState = true;
                }
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMDualShock4 *ds4= &controller->ControllerState.PSDS4State;
                
                if ((channel == PSMControllerRumbleChannel_All || channel == PSMControllerRumbleChannel_Left) &&
                    ds4->BigRumble != rumbleByte)
                {
                    ds4->BigRumble = rumbleByte;
                    ds4->bHasUnpublishedState = true;
                }

                if ((channel == PSMControllerRumbleChannel_All || channel == PSMControllerRumbleChannel_Right) &&
                    ds4->SmallRumble != rumbleByte)
                {
                    ds4->SmallRumble = rumbleByte;
                    ds4->bHasUnpublishedState = true;
                }
            } break;
        case PSMController_Virtual:
            break;
        }

        result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetControllerOrientation(PSMControllerID controller_id, PSMQuatf *out_orientation)
{
    PSMResult result= PSMResult_Error;
	assert(out_orientation);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				PSMPSMove State= controller->ControllerState.PSMoveState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				PSMDualShock4 State= controller->ControllerState.PSDS4State;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Virtual:
            break;
        }
    }

    return result;
}

PSMResult PSM_GetControllerPosition(PSMControllerID controller_id, PSMVector3f *out_position)
{
    PSMResult result= PSMResult_Error;
	assert(out_position);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				PSMPSMove State= controller->ControllerState.PSMoveState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
				PSMDualShock4 State= controller->ControllerState.PSDS4State;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMController_Virtual:
            {
				PSMVirtualController State= controller->ControllerState.VirtualController;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetControllerPose(PSMControllerID controller_id, PSMPosef *out_pose)
{
	PSMResult result = PSMResult_Error;
	assert(out_pose);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			PSMPSMove State = controller->ControllerState.PSMoveState;
			*out_pose = State.Pose;

			result = (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			PSMDualShock4 State = controller->ControllerState.PSDS4State;
			*out_pose = State.Pose;

			result = (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
		} break;
		case PSMController_Virtual:
		{
			PSMVirtualController State = controller->ControllerState.VirtualController;
			*out_pose = State.Pose;

			result = (State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPhysicsData(PSMControllerID controller_id, PSMPhysicsData *out_physics)
{
	PSMResult result = PSMResult_Error;
	assert(out_physics);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			PSMPSMove State = controller->ControllerState.PSMoveState;
			*out_physics = State.PhysicsData;

			result = PSMResult_Success;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			PSMDualShock4 State = controller->ControllerState.PSDS4State;
			*out_physics = State.PhysicsData;

			result = PSMResult_Success;
		} break;
		case PSMController_Virtual:
		{
			PSMVirtualController State = controller->ControllerState.VirtualController;
			*out_physics = State.PhysicsData;

			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPSMoveRawSensorData(PSMControllerID controller_id, PSMPSMoveRawSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Move)
		{
			PSMPSMove State = controller->ControllerState.PSMoveState;
			*out_data = State.RawSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerDualShock4RawSensorData(PSMControllerID controller_id, PSMDS4RawSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_DualShock4)
		{
			PSMDualShock4 State = controller->ControllerState.PSDS4State;
			*out_data = State.RawSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPSMoveSensorData(PSMControllerID controller_id, PSMPSMoveCalibratedSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_Move)
		{
			PSMPSMove State = controller->ControllerState.PSMoveState;
			*out_data = State.CalibratedSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerDualShock4SensorData(PSMControllerID controller_id, PSMDS4CalibratedSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		if (controller->ControllerType == PSMControllerType::PSMController_DualShock4)
		{
			PSMDualShock4 State = controller->ControllerState.PSDS4State;
			*out_data = State.CalibratedSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetIsControllerStable(PSMControllerID controller_id, bool *out_is_stable)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_stable);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
            {
				const float acceleration_stable_range = cosf(10.f * k_degrees_to_radians);

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSMVector3f acceleration_direction = controller->ControllerState.PSMoveState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSM_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_psm_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					(PSM_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction)
						/ (PSM_Vector3fLength(&k_identity_gravity_calibration_direction) 
							* PSM_Vector3fLength(&acceleration_direction))) >= acceleration_stable_range;

				result= PSMResult_Success;
            } break;
        case PSMController_Navi:
            break;
        case PSMController_DualShock4:
            {
                PSMVector3f gyro= controller->ControllerState.PSDS4State.CalibratedSensorData.Gyroscope;
                
				const float k_gyro_noise= 10.f*k_degrees_to_radians; // noise threshold in rad/sec
				const float worst_rotation_rate = fabsf(PSM_Vector3fMaxValue(&gyro));

				*out_is_stable = worst_rotation_rate < k_gyro_noise;

				result= PSMResult_Success;
            } break;
        case PSMController_Virtual:
            // Virtual controller can never be stable
            *out_is_stable = false;
            result= PSMResult_Success;
            break;
        }
    }

    return result;
}

PSMResult PSM_GetIsControllerTracking(PSMControllerID controller_id, bool *out_is_tracking)
{
	PSMResult result = PSMResult_Error;
	assert(out_is_tracking);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			*out_is_tracking = controller->ControllerState.PSMoveState.bIsCurrentlyTracking;
			result = PSMResult_Success;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			*out_is_tracking = controller->ControllerState.PSDS4State.bIsCurrentlyTracking;
			result = PSMResult_Success;
		} break;
		case PSMController_Virtual:
		{
			*out_is_tracking = controller->ControllerState.VirtualController.bIsCurrentlyTracking;
			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetControllerRawTrackerData(PSMControllerID controller_id, PSMRawTrackerData *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			*out_tracker_data = controller->ControllerState.PSMoveState.RawTrackerData;
			result = PSMResult_Success;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			*out_tracker_data = controller->ControllerState.PSDS4State.RawTrackerData;
			result = PSMResult_Success;
		} break;
		case PSMController_Virtual:
		{
			*out_tracker_data = controller->ControllerState.VirtualController.RawTrackerData;
			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetControllerRawTrackerShape(PSMControllerID controller_id, PSMTrackingProjection::eShapeType *out_shape_type)
{
	PSMResult result = PSMResult_Error;
	assert(out_shape_type);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			*out_shape_type = controller->ControllerState.PSMoveState.RawTrackerData.TrackingProjection.shape_type;
			result = PSMResult_Success;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			*out_shape_type = controller->ControllerState.PSDS4State.RawTrackerData.TrackingProjection.shape_type;
			result = PSMResult_Success;
		} break;
		case PSMController_Virtual:
		{
			*out_shape_type = controller->ControllerState.VirtualController.RawTrackerData.TrackingProjection.shape_type;
			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetControllerRawTrackerDataEllipse(PSMControllerID controller_id, PSMRawTrackerDataEllipse *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *state = nullptr;

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			state = &controller->ControllerState.PSMoveState.RawTrackerData;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			state = &controller->ControllerState.PSDS4State.RawTrackerData;
		} break;
		case PSMController_Virtual:
		{
			state = &controller->ControllerState.VirtualController.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_Ellipse)
		{
			PSMRawTrackerDataEllipse out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			out.TrackingProjection.angle = state->TrackingProjection.shape.ellipse.angle;
			out.TrackingProjection.center = state->TrackingProjection.shape.ellipse.center;
			out.TrackingProjection.half_x_extent = state->TrackingProjection.shape.ellipse.half_x_extent;
			out.TrackingProjection.half_y_extent = state->TrackingProjection.shape.ellipse.half_y_extent;

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerRawTrackerDataLightbar(PSMControllerID controller_id, PSMRawTrackerDataLightbat *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *state = nullptr;

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			state = &controller->ControllerState.PSMoveState.RawTrackerData;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			state = &controller->ControllerState.PSDS4State.RawTrackerData;
		} break;
		case PSMController_Virtual:
		{
			state = &controller->ControllerState.VirtualController.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_LightBar)
		{
			PSMRawTrackerDataLightbat out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			memcpy(out.TrackingProjection.triangle, state->TrackingProjection.shape.lightbar.triangle, sizeof(PSMTrackingProjectionLightbat::triangle));
			memcpy(out.TrackingProjection.quad, state->TrackingProjection.shape.lightbar.quad, sizeof(PSMTrackingProjectionLightbat::quad));

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerRawTrackerDataPointcloud(PSMControllerID controller_id, PSMRawTrackerDataPointcloud *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSMController *controller = g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *state = nullptr;

		switch (controller->ControllerType)
		{
		case PSMController_Move:
		{
			state = &controller->ControllerState.PSMoveState.RawTrackerData;
		} break;
		case PSMController_Navi:
			break;
		case PSMController_DualShock4:
		{
			state = &controller->ControllerState.PSDS4State.RawTrackerData;
		} break;
		case PSMController_Virtual:
		{
			state = &controller->ControllerState.VirtualController.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_PointCloud)
		{
			PSMRawTrackerDataPointcloud out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			memcpy(out.TrackingProjection.points, state->TrackingProjection.shape.pointcloud.points, sizeof(PSMTrackingProjectionPointcloud::points));
			out.TrackingProjection.point_count = state->TrackingProjection.shape.pointcloud.point_count;

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetControllerPixelLocationOnTracker(PSMControllerID controller_id, PSMTrackerID *outTrackerId, PSMVector2f *outLocation)
{
	assert(outTrackerId);
	assert(outLocation);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        case PSMController_Virtual:
			trackerData= &controller->ControllerState.VirtualController.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outLocation = trackerData->ScreenLocation;
			return PSMResult_Success;
		}
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerPositionOnTracker(PSMControllerID controller_id, PSMTrackerID *outTrackerId, PSMVector3f *outPosition)
{
    assert(outTrackerId);
	assert(outPosition);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        case PSMController_Virtual:
			trackerData= &controller->ControllerState.VirtualController.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outPosition = trackerData->RelativePositionCm;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerOrientationOnTracker(PSMControllerID controller_id, PSMTrackerID *outTrackerId, PSMQuatf *outOrientation)
{
    assert(outTrackerId);
	assert(outOrientation);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outOrientation = trackerData->RelativeOrientation;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetControllerProjectionOnTracker(PSMControllerID controller_id, PSMTrackerID *outTrackerId, PSMTrackingProjection *outProjection)
{
    assert(outTrackerId);
	assert(outProjection);

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMController *controller= g_psm_client->get_controller_view(controller_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (controller->ControllerType)
        {
        case PSMController_Move:
			trackerData= &controller->ControllerState.PSMoveState.RawTrackerData;
            break;
        case PSMController_DualShock4:
			trackerData= &controller->ControllerState.PSDS4State.RawTrackerData;
            break;
        case PSMController_Virtual:
			trackerData= &controller->ControllerState.VirtualController.RawTrackerData;
            break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outProjection = trackerData->TrackingProjection;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_ResetControllerOrientationAsync(PSMControllerID controller_id, const PSMQuatf *q_pose, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->reset_orientation(controller_id, *q_pose);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_SetControllerDataStreamTrackerIndexAsync(PSMControllerID controller_id, PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->set_controller_data_stream_tracker_index(controller_id, tracker_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_SetControllerHandAsync(PSMControllerID controller_id, PSMControllerHand hand, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSMRequestID req_id = g_psm_client->set_controller_hand(controller_id, hand);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_ResetControllerOrientation(PSMControllerID controller_id, PSMQuatf *q_pose, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		PSMBlockingRequest request(g_psm_client->reset_orientation(controller_id, *q_pose));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_SetControllerDataStreamTrackerIndex(PSMControllerID controller_id, PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && 
        IS_VALID_CONTROLLER_INDEX(controller_id) &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMBlockingRequest request(g_psm_client->set_controller_data_stream_tracker_index(controller_id, tracker_id));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_SetControllerHand(PSMControllerID controller_id, PSMControllerHand hand, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && 
        IS_VALID_CONTROLLER_INDEX(controller_id))
    {
		PSMBlockingRequest request(g_psm_client->set_controller_data_stream_tracker_index(controller_id, hand));

		result= request.send(timeout_ms);
    }

    return result;
}

/// Tracker Pool
PSMTracker *PSM_GetTracker(PSMTrackerID tracker_id)
{
	if (g_psm_client != nullptr)
		return g_psm_client->get_tracker_view(tracker_id);
	else
		return nullptr;
}

PSMResult PSM_GetTrackerEx(PSMTrackerID tracker_id, PSMTracker *tracker_out)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr)
	{
		PSMTracker *state = g_psm_client->get_tracker_view(tracker_id);

		PSMTracker out = {};

		COPY_PROP(listener_count);
		COPY_PROP(is_connected);
		COPY_PROP(sequence_num);

		COPY_PROP(data_frame_last_received_time);
		COPY_PROP(data_frame_average_fps);

		COPY_PROP(tracker_exposure);
		COPY_PROP(tracker_gain);
		COPY_PROP(tracker_width);

		out.tracker_info = state->tracker_info;

		*tracker_out = out;

		result = PSMResult_Success;
	}


	return result;
}

PSMResult PSM_GetTrackerExposure(PSMTrackerID tracker_id, int *exposure)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr)
	{
		PSMTracker *state = g_psm_client->get_tracker_view(tracker_id);

		*exposure = state->tracker_exposure;

		result = PSMResult_Success;
	}


	return result;
}

PSMResult PSM_GetTrackerGain(PSMTrackerID tracker_id, int *gain)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr)
	{
		PSMTracker *state = g_psm_client->get_tracker_view(tracker_id);

		*gain = state->tracker_gain;

		result = PSMResult_Success;
	}


	return result;
}

PSMResult PSM_GetTrackerWidth(PSMTrackerID tracker_id, int *width)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr)
	{
		PSMTracker *state = g_psm_client->get_tracker_view(tracker_id);

		*width = state->tracker_width;

		result = PSMResult_Success;
	}


	return result;
}

PSMResult PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info)
{
    if (g_psm_client != nullptr)
	    return g_psm_client->allocate_tracker_listener(*tracker_info) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_FreeTrackerListener(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psm_client->free_tracker_listener(tracker_id);
        result= PSMResult_Success;
    }

    return result;
}

/// Tracker State Methods
PSMResult PSM_GetTrackerIntrinsicMatrix(PSMTrackerID tracker_id, PSMMatrix3f *out_matrix)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMTracker *tracker= g_psm_client->get_tracker_view(tracker_id);
		PSMClientTrackerInfo *tracker_info= &tracker->tracker_info;

		PSMVector3f basis_x= {tracker_info->tracker_focal_lengths.x, 0.f, tracker_info->tracker_principal_point.x};
		PSMVector3f basis_y= {0.f, tracker_info->tracker_focal_lengths.y, tracker_info->tracker_principal_point.y};
		PSMVector3f basis_z= {0.f, 0.f, 1.f};

		*out_matrix= PSM_Matrix3fCreate(&basis_x, &basis_y, &basis_z);
		result= PSMResult_Success;
	}

	return result;
}

/// Blocking Tracker Methods
PSMResult PSM_GetTrackerList(PSMTrackerList *out_tracker_list, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMBlockingRequest request(g_psm_client->get_tracker_list());
        result_code= request.send(timeout_ms);

        if (result_code == PSMResult_Success)
        {
            assert(request.get_response_payload_type() == PSMResponseMessage::_responsePayloadType_TrackerList);
        
            *out_tracker_list= request.get_response_message().payload.tracker_list;
            result_code= PSMResult_Success;
        }
    }
    
    return result_code;
}

PSMResult PSM_StartTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMBlockingRequest request(g_psm_client->start_tracker_data_stream(tracker_id));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_StopTrackerDataStream(PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMBlockingRequest request(g_psm_client->stop_tracker_data_stream(tracker_id));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMBlockingRequest request(g_psm_client->get_tracker_list());
        result_code= request.send(timeout_ms);

        if (result_code == PSMResult_Success)
        {
            assert(request.get_response_payload_type() == PSMResponseMessage::_responsePayloadType_TrackingSpace);
        
            *out_tracking_space= request.get_response_message().payload.tracking_space;
            result_code= PSMResult_Success;
        }
    }
    
    return result_code;
}

PSMResult PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psm_client->open_video_stream(tracker_id) ? PSMResult_Success : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_PollTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        result= g_psm_client->poll_video_stream(tracker_id) ? PSMResult_Success : PSMResult_NoData;
    }

    return result;
}

PSMResult PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
		g_psm_client->close_video_stream(tracker_id);
		result= PSMResult_Success;
    }

    return result;
}

PSMResult PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, const unsigned char **out_buffer)
{
    PSMResult result= PSMResult_Error;
	assert(out_buffer != nullptr);

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        const unsigned char *buffer= g_psm_client->get_video_frame_buffer(tracker_id);
		if (buffer != nullptr)
		{
			*out_buffer= buffer;
			result= PSMResult_Success;
		}
    }

    return result;
}

PSMResult PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum)
{
	PSMResult result = PSMResult_Error;
	assert(out_frustum != nullptr);

	if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSMTracker *tracker = g_psm_client->get_tracker_view(tracker_id);
		const PSMClientTrackerInfo *tracker_info = &tracker->tracker_info;
		PSM_FrustumSetPose(out_frustum, &tracker_info->tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
		out_frustum->HFOV = tracker_info->tracker_hfov * k_degrees_to_radians;
		out_frustum->VFOV = tracker_info->tracker_vfov * k_degrees_to_radians;
		out_frustum->zNear = tracker_info->tracker_znear;
		out_frustum->zFar = tracker_info->tracker_zfar;

		result = PSMResult_Success;
	}

	return result;
}

PSMResult PSM_GetTrackerPose(PSMTrackerID tracker_id, PSMPosef *out_pose)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSMTracker *tracker = g_psm_client->get_tracker_view(tracker_id);

		*out_pose = tracker->tracker_info.tracker_pose;

		result = PSMResult_Success;
	}

	return result;
}

/// Async Tracker Methods
PSMResult PSM_GetTrackerListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMRequestID req_id = g_psm_client->get_tracker_list();

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_StartTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMRequestID req_id = g_psm_client->start_tracker_data_stream(tracker_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result_code= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result_code;
}

PSMResult PSM_StopTrackerDataStreamAsync(PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMRequestID req_id = g_psm_client->stop_tracker_data_stream(tracker_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result_code= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result_code;
}

PSMResult PSM_GetTrackingSpaceSettingsAsync(PSMRequestID *out_request_id)
{
    if (g_psm_client == nullptr)
        return PSMResult_Error;

    PSMRequestID req_id = g_psm_client->get_tracking_space_settings();

    if (out_request_id != nullptr)
    {
        *out_request_id= req_id;
    }

    return (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
}

/// HMD Pool
PSMHeadMountedDisplay *PSM_GetHmd(PSMHmdID hmd_id)
{
	if (g_psm_client != nullptr)
		return g_psm_client->get_hmd_view(hmd_id);
	else
		return nullptr;
}

/// HMD Pool
PSMResult PSM_GetHmdEx(PSMHmdID hmd_id, PSMHeadMountedDisplayEx *out_hmd)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *state = g_psm_client->get_hmd_view(hmd_id); 
		PSMHeadMountedDisplayEx out = {};

		COPY_PROP(HmdID);
		COPY_PROP(HmdType);

		COPY_PROP(bValid);
		COPY_PROP(OutputSequenceNum);
		COPY_PROP(IsConnected);
		COPY_PROP(DataFrameLastReceivedTime);
		COPY_PROP(DataFrameAverageFPS);
		COPY_PROP(ListenerCount);

		*out_hmd = out;
		result = PSMResult_Success;
	}

	return result;
}

PSMResult PSM_GetHmdMorpheusState(PSMHmdID hmd_id, PSMMorpheus *hmd_state)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *state = g_psm_client->get_hmd_view(hmd_id);

		if (state->HmdType == PSMHmdType::PSMHmd_Morpheus)
		{
			*hmd_state = state->HmdState.MorpheusState;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdMorpheusStateEx(PSMHmdID hmd_id, PSMMorpheusEx *hmd_state)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *view = g_psm_client->get_hmd_view(hmd_id);

		if (view->HmdType == PSMHmdType::PSMHmd_Morpheus)
		{
			const PSMMorpheus *state = &view->HmdState.MorpheusState;
			PSMMorpheusEx out = {};

			COPY_PROP(bIsTrackingEnabled);
			COPY_PROP(bIsCurrentlyTracking);
			COPY_PROP(bIsOrientationValid);
			COPY_PROP(bIsPositionValid);

			*hmd_state = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdVirtualState(PSMHmdID hmd_id, PSMVirtualHMD *hmd_state)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *state = g_psm_client->get_hmd_view(hmd_id);

		if (state->HmdType == PSMHmdType::PSMHmd_Virtual)
		{
			*hmd_state = state->HmdState.VirtualHMDState;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdVirtualStateEx(PSMHmdID hmd_id, PSMVirtualHMDEx *hmd_state)
{
	PSMResult result = PSMResult_Error;

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *view = g_psm_client->get_hmd_view(hmd_id);

		if (view->HmdType == PSMHmdType::PSMHmd_Morpheus)
		{
			const PSMVirtualHMD *state = &view->HmdState.VirtualHMDState;
			PSMVirtualHMDEx out = {};

			COPY_PROP(bIsTrackingEnabled);
			COPY_PROP(bIsCurrentlyTracking); 
			COPY_PROP(bIsPositionValid);

			*hmd_state = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdPhysicsData(PSMHmdID hmd_id, PSMPhysicsData *out_physics)
{
	PSMResult result = PSMResult_Error;
	assert(out_physics);

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			PSMMorpheus State = hmd->HmdState.MorpheusState;
			*out_physics = State.PhysicsData;

			result = PSMResult_Success;
		} break;
		case PSMHmd_Virtual:
		{
			PSMVirtualHMD State = hmd->HmdState.VirtualHMDState;
			*out_physics = State.PhysicsData;

			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetHmdMorpheusRawSensorData(PSMHmdID hmd_id, PSMMorpheusRawSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_physics);

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);

		if (hmd->HmdType == PSMHmdType::PSMHmd_Morpheus)
		{
			PSMMorpheus State = hmd->HmdState.MorpheusState;
			*out_data = State.RawSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdMorpheusSensorData(PSMHmdID hmd_id, PSMMorpheusCalibratedSensorData *out_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_data);

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);

		if (hmd->HmdType == PSMHmdType::PSMHmd_Morpheus)
		{
			PSMMorpheus State = hmd->HmdState.MorpheusState;
			*out_data = State.CalibratedSensorData;

			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_AllocateHmdListener(PSMHmdID hmd_id)
{
    if (g_psm_client != nullptr)
	    return g_psm_client->allocate_hmd_listener(hmd_id) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_FreeHmdListener(PSMHmdID hmd_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		g_psm_client->free_hmd_listener(hmd_id);

        result= PSMResult_Success;
    }

    return result;

}

/// HMD State Methods
PSMResult PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation)
{
    PSMResult result= PSMResult_Error;
	assert(out_orientation);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {		
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_orientation = State.Pose.Orientation;

				result= State.bIsOrientationValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				result= PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position)
{
    PSMResult result= PSMResult_Error;
	assert(out_position);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				PSMVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_position = State.Pose.Position;

				result= State.bIsPositionValid ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose)
{
    PSMResult result= PSMResult_Error;
	assert(out_pose);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				PSMMorpheus State= hmd->HmdState.MorpheusState;
				*out_pose = State.Pose;

				result= (State.bIsOrientationValid && State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        case PSMHmd_Virtual:
            {
				PSMVirtualHMD State= hmd->HmdState.VirtualHMDState;
				*out_pose = State.Pose;

				result= (State.bIsPositionValid) ? PSMResult_Success : PSMResult_Error;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_stable);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				const float k_cosine_20_degrees = 0.9396926f;

				// Get the direction the gravity vector should be pointing 
				// while the controller is in cradle pose.
				const PSMVector3f acceleration_direction = hmd->HmdState.MorpheusState.CalibratedSensorData.Accelerometer;
				float acceleration_magnitude;
				PSM_Vector3fNormalizeWithDefaultGetLength(&acceleration_direction, k_psm_float_vector3_zero, &acceleration_magnitude);

				*out_is_stable =
					is_nearly_equal(1.f, acceleration_magnitude, 0.1f) &&
					PSM_Vector3fDot(&k_identity_gravity_calibration_direction, &acceleration_direction) >= k_cosine_20_degrees;

				result= PSMResult_Success;
            } break;
        case PSMHmd_Virtual:
            {
                // Virtual HMD can never be stable
				*out_is_stable = false;

				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdRawTrackerData(PSMHmdID hmd_id, PSMRawTrackerData *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			*out_tracker_data = hmd->HmdState.MorpheusState.RawTrackerData;
			result = PSMResult_Success;
		} break;
		case PSMHmd_Virtual:
		{
			*out_tracker_data = hmd->HmdState.VirtualHMDState.RawTrackerData;
			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetHmdRawTrackerShape(PSMHmdID hmd_id, PSMTrackingProjection::eShapeType *out_shape_type)
{
	PSMResult result = PSMResult_Error;
	assert(out_shape_type);

	if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			*out_shape_type = hmd->HmdState.MorpheusState.RawTrackerData.TrackingProjection.shape_type;
			result = PSMResult_Success;
		} break;
		case PSMHmd_Virtual:
		{
			*out_shape_type = hmd->HmdState.VirtualHMDState.RawTrackerData.TrackingProjection.shape_type;
			result = PSMResult_Success;
		} break;
		}
	}

	return result;
}

PSMResult PSM_GetHmdRawTrackerDataEllipse(PSMHmdID hmd_id, PSMRawTrackerDataEllipse *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *state = nullptr;

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			state = &hmd->HmdState.MorpheusState.RawTrackerData;
		} break;
		case PSMHmd_Virtual:
		{
			state = &hmd->HmdState.VirtualHMDState.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_Ellipse)
		{
			PSMRawTrackerDataEllipse out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			out.TrackingProjection.angle = state->TrackingProjection.shape.ellipse.angle;
			out.TrackingProjection.center = state->TrackingProjection.shape.ellipse.center;
			out.TrackingProjection.half_x_extent = state->TrackingProjection.shape.ellipse.half_x_extent;
			out.TrackingProjection.half_y_extent = state->TrackingProjection.shape.ellipse.half_y_extent;

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdRawTrackerDataLightbar(PSMHmdID hmd_id, PSMRawTrackerDataLightbat *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *state = nullptr;

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			state = &hmd->HmdState.MorpheusState.RawTrackerData;
		} break;
		case PSMHmd_Virtual:
		{
			state = &hmd->HmdState.VirtualHMDState.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_LightBar)
		{
			PSMRawTrackerDataLightbat out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			memcpy(out.TrackingProjection.triangle, state->TrackingProjection.shape.lightbar.triangle, sizeof(PSMTrackingProjectionLightbat::triangle));
			memcpy(out.TrackingProjection.quad, state->TrackingProjection.shape.lightbar.quad, sizeof(PSMTrackingProjectionLightbat::quad));

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetHmdRawTrackerDataPointcloud(PSMHmdID hmd_id, PSMRawTrackerDataPointcloud *out_tracker_data)
{
	PSMResult result = PSMResult_Error;
	assert(out_tracker_data);

	if (g_psm_client != nullptr && IS_VALID_CONTROLLER_INDEX(hmd_id))
	{
		PSMHeadMountedDisplay *hmd = g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *state = nullptr;

		switch (hmd->HmdType)
		{
		case PSMHmd_Morpheus:
		{
			state = &hmd->HmdState.MorpheusState.RawTrackerData;
		} break;
		case PSMHmd_Virtual:
		{
			state = &hmd->HmdState.VirtualHMDState.RawTrackerData;
		} break;
		}

		if (state != nullptr &&
			state->TrackingProjection.shape_type == PSMTrackingProjection::eShapeType::PSMShape_PointCloud)
		{
			PSMRawTrackerDataPointcloud out = {};

			COPY_PROP(TrackerID);

			COPY_PROP(ScreenLocation);
			COPY_PROP(RelativePositionCm);
			COPY_PROP(RelativeOrientation);

			memcpy(out.TrackingProjection.points, state->TrackingProjection.shape.pointcloud.points, sizeof(PSMTrackingProjectionPointcloud::points));
			out.TrackingProjection.point_count = state->TrackingProjection.shape.pointcloud.point_count;

			COPY_PROP(ValidTrackerBitmask);
			COPY_PROP(MulticamPositionCm);
			COPY_PROP(MulticamOrientation);
			COPY_PROP(bMulticamPositionValid);
			COPY_PROP(bMulticamOrientationValid);

			*out_tracker_data = out;
			result = PSMResult_Success;
		}
	}

	return result;
}

PSMResult PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking)
{
    PSMResult result= PSMResult_Error;
	assert(out_is_tracking);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				*out_is_tracking = hmd->HmdState.MorpheusState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        case PSMHmd_Virtual:
            {
				*out_is_tracking = hmd->HmdState.VirtualHMDState.bIsCurrentlyTracking;
				result= PSMResult_Success;
            } break;
        }
    }

    return result;
}

PSMResult PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMVector2f *outLocation)
{
	assert(outLocation);
    assert(outTrackerId);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outLocation = trackerData->ScreenLocation;
			return PSMResult_Success;
		}
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMVector3f *outPosition)
{
	assert(outPosition);
    assert(outTrackerId);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outPosition = trackerData->RelativePositionCm;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMQuatf *outOrientation)
{
	assert(outOrientation);
    assert(outTrackerId);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= nullptr;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId = trackerData->TrackerID;
			*outOrientation = trackerData->RelativeOrientation;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

PSMResult PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID *outTrackerId, PSMTrackingProjection *outProjection)
{
	assert(outProjection);
    assert(outTrackerId);

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMHeadMountedDisplay *hmd= g_psm_client->get_hmd_view(hmd_id);
		PSMRawTrackerData *trackerData= nullptr;
        
        switch (hmd->HmdType)
        {
        case PSMHmd_Morpheus:
            {
				trackerData= &hmd->HmdState.MorpheusState.RawTrackerData;
            } break;
        case PSMHmd_Virtual:
            {
				trackerData= &hmd->HmdState.VirtualHMDState.RawTrackerData;
            } break;
        }

		if (trackerData != nullptr)
		{
            *outTrackerId= trackerData->TrackerID;
			*outProjection = trackerData->TrackingProjection;
			return PSMResult_Success;
        }
	}

    return PSMResult_Error;
}

/// Blocking HMD Methods
PSMResult PSM_GetHmdList(PSMHmdList *out_hmd_list, int timeout_ms)
{
    PSMResult result_code= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
	    PSMBlockingRequest request(g_psm_client->get_hmd_list());
        result_code= request.send(timeout_ms);

        if (result_code == PSMResult_Success)
        {
            assert(request.get_response_payload_type() == PSMResponseMessage::_responsePayloadType_HmdList);
        
            *out_hmd_list= request.get_response_message().payload.hmd_list;
            result_code= PSMResult_Success;
        }
    }
    
    return result_code;
}

PSMResult PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		PSMBlockingRequest request(g_psm_client->start_hmd_data_stream(hmd_id, data_stream_flags));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_StopHmdDataStream(PSMHmdID hmd_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
		PSMBlockingRequest request(g_psm_client->stop_hmd_data_stream(hmd_id));

		result= request.send(timeout_ms);
    }

    return result;
}

PSMResult PSM_SetHmdDataStreamTrackerIndex(PSMHmdID hmd_id, PSMTrackerID tracker_id, int timeout_ms)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && 
        IS_VALID_HMD_INDEX(hmd_id) &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
		PSMBlockingRequest request(g_psm_client->set_hmd_data_stream_tracker_index(hmd_id, tracker_id));

		result= request.send(timeout_ms);
    }

    return result;
}

/// Async HMD Methods
PSMResult PSM_GetHmdListAsync(PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMRequestID req_id = g_psm_client->get_hmd_list();

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_StartHmdDataStreamAsync(PSMHmdID hmd_id, unsigned int data_stream_flags, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMRequestID req_id = g_psm_client->start_hmd_data_stream(hmd_id, data_stream_flags);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_StopHmdDataStreamAsync(PSMHmdID hmd_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr && IS_VALID_HMD_INDEX(hmd_id))
    {
        PSMRequestID req_id = g_psm_client->stop_hmd_data_stream(hmd_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_SetHmdDataStreamTrackerIndexAsync(PSMHmdID hmd_id, PSMTrackerID tracker_id, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr &&
        IS_VALID_HMD_INDEX(hmd_id) &&
        IS_VALID_TRACKER_INDEX(tracker_id))
    {
        PSMRequestID req_id = g_psm_client->set_hmd_data_stream_tracker_index(hmd_id, tracker_id);

        if (out_request_id != nullptr)
        {
            *out_request_id= req_id;
        }

        result= (req_id != PSM_INVALID_REQUEST_ID) ? PSMResult_RequestSent : PSMResult_Error;
    }

    return result;
}

PSMResult PSM_PollNextMessage(PSMMessage *message, size_t message_size)
{
    // Poll events queued up by the call to g_psm_client->update()
    if (g_psm_client != nullptr)
        return g_psm_client->poll_next_message(message, message_size) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_SendOpaqueRequest(PSMRequestHandle request_handle, PSMRequestID *out_request_id)
{
    PSMResult result= PSMResult_Error;

    if (g_psm_client != nullptr)
    {
        PSMRequestID request_id= g_psm_client->send_opaque_request(request_handle);

        if (request_id != PSM_INVALID_REQUEST_ID)
        {
            if (out_request_id != nullptr)
            {
                *out_request_id= request_id;
            }

            result= PSMResult_RequestSent;
        }
    }

    return result;
}

PSMResult PSM_RegisterCallback(PSMRequestID request_id, PSMResponseCallback callback, void *callback_userdata)
{
    if (g_psm_client != nullptr)
        return g_psm_client->register_callback(request_id, callback, callback_userdata) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

PSMResult PSM_CancelCallback(PSMRequestID request_id)
{
    if (g_psm_client != nullptr)
        return g_psm_client->cancel_callback(request_id) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

static void null_response_callback(
    const PSMResponseMessage *response,
    void *userdata)
{ }

PSMResult PSM_EatResponse(PSMRequestID request_id)
{
    if (g_psm_client != nullptr)
	    return g_psm_client->register_callback(request_id, null_response_callback, nullptr) ? PSMResult_Success : PSMResult_Error;
    else
        return PSMResult_Error;
}

//-- inludes -----
#include "AppStage_TestTracker.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "AssetManager.h"
#include "App.h"
#include "Camera.h"
#include "ClientLog.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

#define k_tracker_target_exposure (256.f - 8.f)
#define k_tracker_target_gain (128.f - 8.f)

//-- statics ----
const char *AppStage_TestTracker::APP_STAGE_NAME = "TestTracker";

//-- constants -----

//-- private methods -----

//-- public methods -----
AppStage_TestTracker::AppStage_TestTracker(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestTracker::inactive)
    , m_bStreamIsActive(false)
    , m_tracker_view(nullptr)
    , m_video_texture(nullptr)
	, m_streamFps(0)
	, m_displayFps(0)
	, m_bChangedExposure(false)
	, m_bChangedGain(false)
	, m_trackerFrameRate(0)
	, m_trackerExposure(0)
	, m_trackerGain(0)
{ }

void AppStage_TestTracker::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSMClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

	tracker_count = trackerSettings->get_tracker_count();
	tracker_index = trackerSettings->get_tracker_Index();

    assert(m_tracker_view == nullptr);
	PSM_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
	m_tracker_view = PSM_GetTracker(trackerInfo->tracker_id);

	assert(!m_bChangedExposure);
	assert(!m_bChangedGain);

    assert(!m_bStreamIsActive);
    request_tracker_start_stream();
}

void AppStage_TestTracker::exit()
{
    m_menuState = AppStage_TestTracker::inactive;

    PSM_FreeTrackerListener(m_tracker_view->tracker_info.tracker_id);
    m_tracker_view = nullptr;
}

void AppStage_TestTracker::update()
{
    // Try and read the next video frame from shared memory
    if (m_video_texture != nullptr)
    {
        if (PSM_PollTrackerVideoStream(m_tracker_view->tracker_info.tracker_id) == PSMResult_Success)
        {
			const unsigned char *buffer= nullptr;
			if (PSM_GetTrackerVideoFrameBuffer(m_tracker_view->tracker_info.tracker_id, &buffer) == PSMResult_Success)
			{
				m_streamFps++;

				m_video_texture->copyBufferIntoTexture(buffer);
			}
        }
    }


	switch (m_menuState)
	{
	case eTrackerMenuState::pendingTrackerResetExposureGain:
	{
		if (!m_bChangedExposure && !m_bChangedGain)
		{
			request_tracker_stop_stream();
		}
		break;
	}
	case eTrackerMenuState::pendingTrackerSetExposureGain:
	{
		if (m_bChangedExposure && m_bChangedGain)
		{
			m_menuState = AppStage_TestTracker::idle;
		}
		break;
	}
	}
}

void AppStage_TestTracker::render()
{
    // If there is a video frame available to render, show it
	if (m_video_texture != nullptr)
	{
		unsigned int texture_id = m_video_texture->texture_id;

		if (texture_id != 0)
		{
			drawFullscreenTexture(texture_id);
		}
	}
}

void AppStage_TestTracker::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Video Feed Test";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 100));
        ImGui::Begin(k_window_title, nullptr, window_flags);

		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastStreamFps;
		if (timeSinceLast.count() > 1000.f)
		{
			m_displayFps = m_streamFps;
			m_streamFps = 0;
			m_lastStreamFps = now;
		}

		if (ImGui::Button(" < ##TrackerIndex"))
		{
			m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(((tracker_index + tracker_count) - 1) % tracker_count);
			m_app->getAppStage<AppStage_TrackerSettings>()->gotoVideoTest(true);

			// Goes back to tracker settings.
			request_tracker_reset_exposure_gain();
		}
		ImGui::SameLine();
		if (ImGui::Button(" > ##TrackerIndex"))
		{
			m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(((tracker_index + tracker_count) + 1) % tracker_count);
			m_app->getAppStage<AppStage_TrackerSettings>()->gotoVideoTest(true);

			// Goes back to tracker settings.
			request_tracker_reset_exposure_gain();
		}
		ImGui::SameLine();
		ImGui::Text("Tracker: #%d", m_tracker_view->tracker_info.tracker_id);

		if (m_displayFps < m_trackerFrameRate - 7.5f)
		{
			ImGui::TextColored(ImColor(1.f, 0.f, 0.f), "Tracker Frame Rate: %d", m_displayFps);
		}
		else
		{
			ImGui::Text("Tracker Frame Rate: %d", m_displayFps);
		}

		ImGui::Separator();

        if (ImGui::Button("Return to Tracker Settings"))
        {
			request_tracker_reset_exposure_gain();
        }              

        ImGui::End();
    } break;

	case eTrackerMenuState::pendingTrackerStartStreamRequest:
	case eTrackerMenuState::pendingTrackerGetSettings:
	case eTrackerMenuState::pendingTrackerSetExposureGain:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to start...");

        ImGui::End();
    } break;

	case eTrackerMenuState::failedTrackerStartStreamRequest:
	case eTrackerMenuState::failedTrackerGetSettings:
	case eTrackerMenuState::failedTrackerSetExposureGain:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to start tracker stream!");

        if (ImGui::Button("Return to Tracker Settings"))
        {
			request_tracker_reset_exposure_gain();
        }

        ImGui::End();
    } break;

    case eTrackerMenuState::pendingTrackerStopStreamRequest:
	case eTrackerMenuState::pendingTrackerResetExposureGain:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to stop...");

        ImGui::End();
    } break;

	case eTrackerMenuState::failedTrackerStopStreamRequest:
	case eTrackerMenuState::failedTrackerResetExposureGain:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to stop tracker stream!");

        if (ImGui::Button("Return to Tracker Settings"))
        {
			request_tracker_reset_exposure_gain();
        }

        ImGui::End();
    } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_TestTracker::request_tracker_reset_exposure_gain()
{
	if (m_menuState != AppStage_TestTracker::pendingTrackerResetExposureGain) 
	{
		m_menuState = AppStage_TestTracker::pendingTrackerResetExposureGain;

		request_tracker_reset_exposure();
		request_tracker_reset_gain();
	}
}

void AppStage_TestTracker::request_tracker_reset_exposure()
{
	// Tell the psmove service that we want to change exposure.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_EXPOSURE);
	request->mutable_request_set_tracker_exposure()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
	request->mutable_request_set_tracker_exposure()->set_value(m_trackerExposure);
	request->mutable_request_set_tracker_exposure()->set_save_setting(false);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_TestTracker::handle_tracker_reset_exposure_response, this);
}

void AppStage_TestTracker::request_tracker_reset_gain()
{
	// Tell the psmove service that we want to change exposure.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_GAIN);
	request->mutable_request_set_tracker_gain()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
	request->mutable_request_set_tracker_gain()->set_value(m_trackerGain);
	request->mutable_request_set_tracker_gain()->set_save_setting(false);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_TestTracker::handle_tracker_reset_gain_response, this);
}

void AppStage_TestTracker::handle_tracker_reset_gain_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->m_bChangedGain = false;
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TestTracker::failedTrackerResetExposureGain;
	} break;
	}
}

void AppStage_TestTracker::handle_tracker_reset_exposure_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->m_bChangedExposure = false;
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TestTracker::failedTrackerResetExposureGain;
	} break;
	}
}

void AppStage_TestTracker::request_tracker_start_stream()
{
	if (m_menuState != AppStage_TestTracker::pendingTrackerStartStreamRequest)
	{
		m_menuState = AppStage_TestTracker::pendingTrackerStartStreamRequest;

		// Tell the psmove service that we want to start streaming data from the tracker
		PSMRequestID requestID;
		PSM_StartTrackerDataStreamAsync(
			m_tracker_view->tracker_info.tracker_id,
			&requestID);
		PSM_RegisterCallback(requestID, AppStage_TestTracker::handle_tracker_start_stream_response, this);
	}
}

void AppStage_TestTracker::handle_tracker_start_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            PSMTracker *trackerView= thisPtr->m_tracker_view;

            thisPtr->m_bStreamIsActive = true;

            // Open the shared memory that the vidoe stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSMResult_Success)
            {
                // Create a texture to render the video frame to
                thisPtr->m_video_texture = new TextureAsset();
                thisPtr->m_video_texture->init(
                    static_cast<unsigned int>(trackerView->tracker_info.tracker_screen_dimensions.x),
                    static_cast<unsigned int>(trackerView->tracker_info.tracker_screen_dimensions.y),
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);
            }

			thisPtr->request_tracker_get_settings();
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStartStreamRequest;
        } break;
    }
}

void AppStage_TestTracker::request_tracker_get_settings()
{
	if (m_menuState != AppStage_TestTracker::pendingTrackerGetSettings)
	{
		m_menuState = AppStage_TestTracker::pendingTrackerGetSettings;

		// Tell the psmove service that we want to change exposure.
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_SETTINGS);
		request->mutable_request_get_tracker_settings()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);

		request->mutable_request_get_tracker_settings()->set_device_id(-1);
		request->mutable_request_get_tracker_settings()->set_device_category(PSMoveProtocol::Request_RequestGetTrackerSettings_DeviceCategory_CONTROLLER);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TestTracker::handle_tracker_get_settings_response, this);
	}
}

void AppStage_TestTracker::handle_tracker_get_settings_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
		thisPtr->m_trackerExposure = response->result_tracker_settings().exposure();
		thisPtr->m_trackerGain = response->result_tracker_settings().gain();
		thisPtr->m_trackerFrameRate = response->result_tracker_settings().frame_rate();

		if (thisPtr->m_menuState != AppStage_TestTracker::pendingTrackerSetExposureGain)
		{
			thisPtr->m_menuState = AppStage_TestTracker::pendingTrackerSetExposureGain;

			thisPtr->request_tracker_set_exposure();
			thisPtr->request_tracker_set_gain();
		}
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TestTracker::failedTrackerGetSettings;
	} break;
	}
}

void AppStage_TestTracker::request_tracker_set_exposure()
{
	// Tell the psmove service that we want to change exposure.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_EXPOSURE);
	request->mutable_request_set_tracker_exposure()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
	request->mutable_request_set_tracker_exposure()->set_value(k_tracker_target_exposure);
	request->mutable_request_set_tracker_exposure()->set_save_setting(false);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_TestTracker::handle_tracker_set_exposure_response, this);
}

void AppStage_TestTracker::request_tracker_set_gain()
{
	// Tell the psmove service that we want to change exposure.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_GAIN);
	request->mutable_request_set_tracker_gain()->set_tracker_id(m_tracker_view->tracker_info.tracker_id);
	request->mutable_request_set_tracker_gain()->set_value(k_tracker_target_gain);
	request->mutable_request_set_tracker_gain()->set_save_setting(false);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_TestTracker::handle_tracker_set_gain_response, this);
}

void AppStage_TestTracker::handle_tracker_set_gain_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->m_bChangedGain = true;
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TestTracker::failedTrackerSetExposureGain;
	} break;
	}
}

void AppStage_TestTracker::handle_tracker_set_exposure_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->m_bChangedExposure = true;
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TestTracker::failedTrackerSetExposureGain;
	} break;
	}
}

void AppStage_TestTracker::request_tracker_stop_stream()
{
	if (!m_bStreamIsActive)
	{
		m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
		return;
	}

    if (m_menuState != AppStage_TestTracker::pendingTrackerStopStreamRequest)
    {
        m_menuState = AppStage_TestTracker::pendingTrackerStopStreamRequest;

        // Tell the psmove service that we want to stop streaming data from the tracker
		PSMRequestID requestId;
		PSM_StopTrackerDataStreamAsync(m_tracker_view->tracker_info.tracker_id, &requestId);
		PSM_RegisterCallback(requestId, AppStage_TestTracker::handle_tracker_stop_stream_response, this);
    }
}

void AppStage_TestTracker::handle_tracker_stop_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    // In either case consider the stream as now inactive
    thisPtr->m_bStreamIsActive = false;

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            thisPtr->m_menuState = AppStage_TestTracker::inactive;

            // Close the shared memory buffer
			PSM_CloseTrackerVideoStream(thisPtr->m_tracker_view->tracker_info.tracker_id);

            // Free the texture we were rendering to
            if (thisPtr->m_video_texture != nullptr)
            {
                delete thisPtr->m_video_texture;
                thisPtr->m_video_texture = nullptr;
            }

            // After closing the stream, we should go back to the tracker settings
            thisPtr->m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
	case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStopStreamRequest;
        } break;
    }
}

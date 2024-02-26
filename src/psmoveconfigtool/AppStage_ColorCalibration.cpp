//-- inludes -----
#include "AppStage_ColorCalibration.h"
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

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <thread>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

const int k_color_autodetect_probe_max = 64; 
const int k_color_autodetect_probe_step = 8;

const int k_auto_calib_sleep = 350;

//-- statics ----
const char *AppStage_ColorCalibration::APP_STAGE_NAME = "ColorCalibration";

//-- constants -----
static const char *k_tracking_color_names[] = {
    "Magenta",
    "Cyan",
    "Yellow",
    "Red",
    "Green",
	"Blue",

	"Custom0",
	"Custom1",
	"Custom2",
	"Custom3",
	"Custom4",
	"Custom5",
	"Custom6",
	"Custom7",
	"Custom8",
	"Custom9"
};

//-- private definitions -----
class VideoBufferState
{
public:
    VideoBufferState(PSMTracker *trackerView)
        : videoTexture(nullptr)
        , bgrBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
		, maskedBuffer(nullptr)
		, detectionMaskedBuffer(nullptr)
		, detectionLowerBuffer(nullptr)
    {
        const int frameWidth = static_cast<int>(trackerView->tracker_info.tracker_screen_dimensions.x);
        const int frameHeight = static_cast<int>(trackerView->tracker_info.tracker_screen_dimensions.y);

        // Create a texture to render the video frame to
        videoTexture = new TextureAsset();
        videoTexture->init(
            frameWidth,
            frameHeight,
            GL_RGB, // texture format
            GL_BGR, // buffer format
            nullptr);
		
        bgrBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        hsvBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsLowerBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsUpperBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
		maskedBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);

		detectionMaskedBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
		detectionLowerBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
    }

    virtual ~VideoBufferState()
    {
		if (detectionLowerBuffer != nullptr)
		{
			delete detectionLowerBuffer;
			detectionLowerBuffer = nullptr;
		}

		if (detectionMaskedBuffer != nullptr)
		{
			delete detectionMaskedBuffer;
			detectionMaskedBuffer = nullptr;
		}

		if (maskedBuffer != nullptr)
		{
			delete maskedBuffer;
			maskedBuffer = nullptr;
		}

        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
            gsLowerBuffer = nullptr;
        }

        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
            gsUpperBuffer = nullptr;
        }

        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
            hsvBuffer = nullptr;
        }

        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
            bgrBuffer = nullptr;
        }

        if (videoTexture != nullptr)
        {
            delete videoTexture;
            videoTexture = nullptr;
        }
    }

    TextureAsset *videoTexture;
    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
	cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask

	cv::Mat *detectionMaskedBuffer; // bgr masks images ANDed together with grayscale mask
	cv::Mat *detectionLowerBuffer;  // bit mask image
};

//-- public methods -----
AppStage_ColorCalibration::AppStage_ColorCalibration(App *app)
    : AppStage(app)
    , m_overrideControllerId(-1)
    , m_masterControllerView(nullptr)
    , m_pendingControllerStartCount(0)
    , m_areAllControllerStreamsActive(false)
	, m_lastMasterControllerSeqNum(-1)
    , m_overrideHmdId(-1)
    , m_hmdView(nullptr)
    , m_isHmdStreamActive(false)
    , m_lastHmdSeqNum(-1)
    , m_trackerView(nullptr)
    , m_menuState(AppStage_ColorCalibration::inactive)
    , m_video_buffer_state(nullptr)
    , m_videoDisplayMode(AppStage_ColorCalibration::eVideoDisplayMode::mode_bgr)
    , m_trackerFrameWidth(0)
    , m_trackerFrameRate(0)
    , m_trackerExposure(0)
    , m_trackerGain(0)
    , m_bTurnOnAllControllers(false)
    , m_bAutoChangeController(false)
    , m_bAutoChangeColor(false)
    , m_bAutoChangeTracker(false)
	, m_bAdvancedMode(false)
	, m_bShowWindows(true)
	, m_bAlignDetectColor(false)
	, m_bAlignPinned(false)
	, m_iColorSensitivity(sensitivity_normal)
	, m_bColorCollisionPrevent(false)
	, m_bColorCollsionShow(false)
    , m_masterTrackingColorType(PSMTrackingColorType_Magenta)
	, m_bDetectingColors(false)
	, m_iDetectingControllersLeft(0)
	, m_iDetectingExposure(0)
	, m_iDetectingAdjustMethod(eDetectionAdjustMethod::adjust_exposure)
	, m_bDetectingCancel(false)
	, m_iDetectingFailReason(eDetectionFailReason::failreason_unknown)
	, m_bProjectionBlacklistedShow(true)
	, m_streamFps(0)
	, m_displayFps(0)
	, m_bUpdateTrackingBulbs(false)
{
	memset(m_colorPresets, 0, sizeof(m_colorPresets));
	memset(m_blacklisted_projection, 0, sizeof(m_blacklisted_projection));

	m_mAlignPosition[0] = 0.0f;
	m_mAlignPosition[1] = 0.0f;
}

void AppStage_ColorCalibration::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSMClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    tracker_count = trackerSettings->get_tracker_count();
    tracker_index = trackerSettings->get_tracker_Index();

    // Use the tracker selected from the tracker settings menu
    assert(m_trackerView == nullptr);
    PSM_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
    m_trackerView = PSM_GetTracker(trackerInfo->tracker_id);

    if (m_overrideHmdId != -1)
    {
        assert(m_hmdView == nullptr);
        PSM_AllocateHmdListener(m_overrideHmdId);
        m_hmdView = PSM_GetHmd(m_overrideHmdId);
        m_isHmdStreamActive = false;
        m_lastHmdSeqNum = -1;
    }
    else
    {
        // Assume that we can bind to controller 0 if no controller override is given
        const int masterControllerID = (m_overrideControllerId != -1) ? m_overrideControllerId : 0;

        m_controllerViews.clear();
        m_controllerTrackingColorTypes.clear();

        for (int list_index = 0; list_index < trackerSettings->get_controller_count(); ++list_index)
        {
            const AppStage_TrackerSettings::ControllerInfo *controller_info= trackerSettings->get_controller_info(list_index);
            PSM_AllocateControllerListener(controller_info->ControllerID);
            PSMController *controllerView= PSM_GetController(controller_info->ControllerID);

            if (masterControllerID == controller_info->ControllerID)
            {
                assert(m_masterControllerView == nullptr);
                m_masterControllerView= controllerView;
            }

            m_controllerViews.push_back(controllerView);
            m_controllerTrackingColorTypes.push_back(controller_info->TrackingColorType);
        }
        
        m_areAllControllerStreamsActive = false;
		m_lastMasterControllerSeqNum = -1;
		m_bUpdateTrackingBulbs = true;
        m_pendingControllerStartCount= false;
    }

    // Request to start the tracker
    // Wait for the tracker response before requesting the controller
    assert(m_video_buffer_state == nullptr);
    request_tracker_start_stream();

    // In parallel, Get the settings for the selected tracker
    request_tracker_get_settings();
}

void AppStage_ColorCalibration::exit()
{
    setState(AppStage_ColorCalibration::inactive);

    release_devices();
}

void AppStage_ColorCalibration::update()
{
    if (m_menuState == eMenuState::waitingForStreamStartResponse)
    {
        if (m_areAllControllerStreamsActive && m_masterControllerView->OutputSequenceNum != m_lastMasterControllerSeqNum)
        {
			m_lastMasterControllerSeqNum = m_masterControllerView->OutputSequenceNum;

			request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            
			if (m_bDetectingColors)
			{
				setState(eMenuState::detection_exposure_adjust);
			}
			else
			{
				setState(eMenuState::manualConfig);
			}

        }
        else if (m_isHmdStreamActive && m_hmdView->OutputSequenceNum != m_lastHmdSeqNum)
        {
			m_lastHmdSeqNum = m_hmdView->OutputSequenceNum;

			if (m_bDetectingColors)
			{
				setState(eMenuState::detection_exposure_adjust);
			}
			else
			{
				setState(eMenuState::manualConfig);
			}
        }
    }
	else if (m_menuState == eMenuState::manualConfig)
	{
		if (m_areAllControllerStreamsActive && m_masterControllerView->OutputSequenceNum != m_lastMasterControllerSeqNum)
		{
			m_lastMasterControllerSeqNum = m_masterControllerView->OutputSequenceNum;

			if (m_bUpdateTrackingBulbs)
			{
				m_bUpdateTrackingBulbs = false;

				if (m_bTurnOnAllControllers || m_bColorCollsionShow)
				{
					request_turn_on_all_tracking_bulbs(true);
				}
			}

		}
	}

    // Try and read the next video frame from shared memory
    if (m_video_buffer_state != nullptr)
    {
        const unsigned char *video_buffer= nullptr;
        if (PSM_PollTrackerVideoStream(m_trackerView->tracker_info.tracker_id) == PSMResult_Success &&
            PSM_GetTrackerVideoFrameBuffer(m_trackerView->tracker_info.tracker_id, &video_buffer) == PSMResult_Success)
        {
			m_streamFps++;

            const int frameWidth = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.x);
            const int frameHeight = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.y);
            const unsigned char *display_buffer = video_buffer;
            const TrackerColorPreset &preset = getColorPreset();

            // Copy the video frame buffer into the bgr opencv buffer
            {
                const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

                videoBufferMat.copyTo(*m_video_buffer_state->bgrBuffer);
            }

            // Convert the video buffer to the HSV color space
            cv::cvtColor(*m_video_buffer_state->bgrBuffer, *m_video_buffer_state->hsvBuffer, cv::COLOR_BGR2HSV);

            // Clamp the HSV image, taking into account wrapping the hue angle
            {
                const float hue_min = preset.hue_center - preset.hue_range;
                const float hue_max = preset.hue_center + preset.hue_range;
                const float saturation_min = clampf(preset.saturation_center - preset.saturation_range, 0, 255);
                const float saturation_max = clampf(preset.saturation_center + preset.saturation_range, 0, 255);
                const float value_min = clampf(preset.value_center - preset.value_range, 0, 255);
                const float value_max = clampf(preset.value_center + preset.value_range, 0, 255);

                if (hue_min < 0)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(180 + hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else if (hue_max > 180)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max - 180, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(hue_min, saturation_min, value_min),
                        cv::Scalar(hue_max, saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                }
            }
			
            // Mask out the original video frame with the HSV filtered mask
            *m_video_buffer_state->maskedBuffer = cv::Scalar(0, 0, 0);
            cv::bitwise_and(
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->maskedBuffer, 
                *m_video_buffer_state->gsLowerBuffer);

			get_contures_lower(0, 2, m_mDetectedContures);


			if (!m_bAlignDetectColor && !m_bDetectingColors)
			{
				if (m_bProjectionBlacklistedShow)
				{
					for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
					{
						float proj_x, proj_y, proj_w, proj_h;
						proj_x = m_blacklisted_projection[i].x;
						proj_y = m_blacklisted_projection[i].y;
						proj_w = m_blacklisted_projection[i].w;
						proj_h = m_blacklisted_projection[i].h;

						cv::Rect rect = cv::Rect(
							static_cast<int>(proj_x), 
							static_cast<int>(proj_y),
							static_cast<int>(proj_w),
							static_cast<int>(proj_h));

						if (rect.width < 1 || rect.height < 1)
							continue;

						cv::rectangle(
							*m_video_buffer_state->bgrBuffer,
							rect,
							cv::Scalar(0, 255, 255)
						);

						std::string strIndex = std::to_string(i);
						strIndex.append("#");

						cv::putText(
							*m_video_buffer_state->bgrBuffer,
							strIndex,
							cv::Point(
								static_cast<int>(proj_x + 2.0), 
								static_cast<int>(proj_y + 10.0)
							),
							cv::FONT_HERSHEY_PLAIN,
							0.75,
							CvScalar(0, 255, 255),
							1
						);
					}
				}

				if (m_bColorCollsionShow)
				{
					float align_window_size = 32.f;

					int detect_count = 0;

					for (std::vector<int> item : m_mDetectedContures)
					{
						ImVec2 dispSize = ImGui::GetIO().DisplaySize;
						int img_x = (static_cast<int>(item[0]) * m_video_buffer_state->bgrBuffer->cols) / static_cast<int>(dispSize.x);
						int img_y = (static_cast<int>(item[1]) * m_video_buffer_state->bgrBuffer->rows) / static_cast<int>(dispSize.y);
						ImVec2 wndCenter = ImVec2(static_cast<float>(img_x), static_cast<float>(img_y));

						if (detect_count++ == 0)
						{
							cv::Rect rect = cv::Rect(
								static_cast<int>(wndCenter.x - (align_window_size / 2)),
								static_cast<int>(wndCenter.y - (align_window_size / 2)),
								static_cast<int>(align_window_size),
								static_cast<int>(align_window_size));

							cv::rectangle(
								*m_video_buffer_state->bgrBuffer,
								rect,
								cv::Scalar(0, 255, 0)
							);
						}
						else
						{
							cv::Rect rect = cv::Rect(
								static_cast<int>(wndCenter.x - (align_window_size / 2)),
								static_cast<int>(wndCenter.y - (align_window_size / 2)),
								static_cast<int>(align_window_size),
								static_cast<int>(align_window_size));

							cv::rectangle(
								*m_video_buffer_state->bgrBuffer,
								rect,
								cv::Scalar(0, 0, 255)
							);
						}
					}
				}
				else
				{
					float align_window_size = 32.f;

					for (std::vector<int> item : m_mDetectedContures)
					{
						ImVec2 dispSize = ImGui::GetIO().DisplaySize;
						int img_x = (static_cast<int>(item[0]) * m_video_buffer_state->bgrBuffer->cols) / static_cast<int>(dispSize.x);
						int img_y = (static_cast<int>(item[1]) * m_video_buffer_state->bgrBuffer->rows) / static_cast<int>(dispSize.y);
						ImVec2 wndCenter = ImVec2(static_cast<float>(img_x), static_cast<float>(img_y));

						cv::Rect rect = cv::Rect(
							static_cast<int>(wndCenter.x - (align_window_size / 2)),
							static_cast<int>(wndCenter.y - (align_window_size / 2)),
							static_cast<int>(align_window_size),
							static_cast<int>(align_window_size));

						cv::rectangle(
							*m_video_buffer_state->bgrBuffer,
							rect,
							cv::Scalar(255, 255, 255)
						);
						break;
					}
				}
			}

            switch (m_videoDisplayMode)
            {
            case AppStage_ColorCalibration::mode_bgr:
                display_buffer = m_video_buffer_state->bgrBuffer->data;
                break;
            case AppStage_ColorCalibration::mode_hsv:
                display_buffer = m_video_buffer_state->hsvBuffer->data;
                break;
            case AppStage_ColorCalibration::mode_masked:
                display_buffer = m_video_buffer_state->maskedBuffer->data;
                break;
            default:
                assert(0 && "unreachable");
                break;
            }

            // Display the selected buffer
            m_video_buffer_state->videoTexture->copyBufferIntoTexture(display_buffer);
        }
    }
}

void AppStage_ColorCalibration::render()
{
    // If there is a video frame available to render, show it
    if (m_video_buffer_state != nullptr)
    {
        unsigned int texture_id = m_video_buffer_state->videoTexture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_ColorCalibration::renderUI()
{
	// Tracker Alignment Marker
	if (m_bAlignDetectColor && m_video_buffer_state != nullptr) 
	{
		float align_window_size = 64.f;
		ImVec2 align_pos;
		ImVec2 align_pos_window;

		if (m_bAlignPinned)
		{
			align_pos.x = m_mAlignPosition[0];
			align_pos.y = m_mAlignPosition[1];
			align_pos_window.x = align_pos.x - (align_window_size / 2);
			align_pos_window.y = align_pos.y - (align_window_size / 2);
		}
		else
		{
			align_pos.x = ImGui::GetMousePos().x;
			align_pos.y = ImGui::GetMousePos().y;
			align_pos_window.x = align_pos.x - (align_window_size / 2);
			align_pos_window.y = align_pos.y - (align_window_size / 2);
		}
		
		// Is in window?
		if (align_pos.x > -1 && align_pos.y > -1)
		{
			float prevAlpha = ImGui::GetStyle().WindowFillAlphaDefault;
			float prevRound = ImGui::GetStyle().WindowRounding;
			ImGui::GetStyle().WindowFillAlphaDefault = 0.f;
			ImGui::GetStyle().WindowRounding = 20.f;

			ImVec2 dispSize = ImGui::GetIO().DisplaySize;
			int img_x = (static_cast<int>(align_pos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
			int img_y = (static_cast<int>(align_pos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
			cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));
			cv::Vec< unsigned char, 3 > bgrBuffer = m_video_buffer_state->bgrBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

			ImGui::SetNextWindowPos(align_pos_window);
			ImGui::SetNextWindowSize(ImVec2(align_window_size, align_window_size));

			ImGui::Begin("Alignment Window", nullptr,
				ImGuiWindowFlags_NoBringToFrontOnFocus |
				ImGuiWindowFlags_NoFocusOnAppearing |
				ImGuiWindowFlags_NoTitleBar |
				ImGuiWindowFlags_ShowBorders |
				ImGuiWindowFlags_NoResize |
				ImGuiWindowFlags_NoMove |
				ImGuiWindowFlags_NoScrollbar |
				ImGuiWindowFlags_NoCollapse);
			{
				ImU32 line_colour = ImColor(0xFF, 0xFF, 0xFF, 175);
				float line_thickness = 1.f;

				ImGui::GetWindowDrawList()->AddLine(
					ImVec2(align_pos_window.x + 15, align_pos_window.y + 15),
					ImVec2(align_pos_window.x + align_window_size - 15, align_pos_window.y + align_window_size - 15),
					line_colour,
					line_thickness
				);
				ImGui::GetWindowDrawList()->AddLine(
					ImVec2(align_pos_window.x + align_window_size - 15, align_pos_window.y + 15),
					ImVec2(align_pos_window.x + 15, align_pos_window.y + align_window_size - 15),
					line_colour,
					line_thickness
				);

			}
			ImGui::End();

			align_pos_window.x -= 96.0f;
			align_pos_window.y += align_window_size;

			ImGui::SetNextWindowPos(align_pos_window);
			ImGui::SetNextWindowSize(ImVec2(350, 116));

			ImGui::Begin("Alignment Window Tip", nullptr,
				ImGuiWindowFlags_NoBringToFrontOnFocus |
				ImGuiWindowFlags_NoFocusOnAppearing |
				ImGuiWindowFlags_NoTitleBar |
				ImGuiWindowFlags_NoResize |
				ImGuiWindowFlags_NoMove |
				ImGuiWindowFlags_NoScrollbar |
				ImGuiWindowFlags_NoCollapse |
				ImGuiWindowFlags_NoSavedSettings);
			{
				ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos(), ImVec2(ImGui::GetWindowPos().x + ImGui::GetWindowSize().x, ImGui::GetWindowPos().y + ImGui::GetWindowSize().y), ImColor(0.f, 0.f, 0.f, 0.5f));

				ImColor textColor = ImColor(0xFF, 0xFF, 0xFF, 255);

				ImGui::ColorButton(ImColor(bgrBuffer[2], bgrBuffer[1], bgrBuffer[0]), true);
				ImGui::SameLine();
				ImGui::TextColored(textColor, "Color | H: %d, S: %d, V: %d", hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]);
				ImGui::TextColored(textColor, "Left-click the mouse button to detect color.");
				ImGui::TextColored(textColor, "Right-click the mouse button to cancel.");
				ImGui::Separator();

				if (m_masterControllerView != nullptr)
				{
					ImGui::TextColored(textColor, "Controller #%d", m_masterControllerView->ControllerID);
				}
				else if (m_hmdView != nullptr)
				{
					ImGui::TextColored(textColor, "HMD #%d", m_hmdView->HmdID);
				}
				ImGui::TextColored(textColor, "Tracker #%d", m_trackerView->tracker_info.tracker_id);

				ImGui::TextColored(textColor, "Tracking Color:");
				ImGui::SameLine();
				switch (m_masterTrackingColorType)
				{
				case PSMTrackingColorType::PSMTrackingColorType_Red:
				{
					ImGui::ColorButton(ImColor(255, 0, 0), true);
					break;
				}
				case PSMTrackingColorType::PSMTrackingColorType_Green:
				{
					ImGui::ColorButton(ImColor(0, 255, 0), true);
					break;
				}
				case PSMTrackingColorType::PSMTrackingColorType_Blue:
				{
					ImGui::ColorButton(ImColor(0, 0, 255), true);
					break;
				}
				case PSMTrackingColorType::PSMTrackingColorType_Magenta:
				{
					ImGui::ColorButton(ImColor(255, 0, 255), true);
					break;
				}
				case PSMTrackingColorType::PSMTrackingColorType_Cyan:
				{
					ImGui::ColorButton(ImColor(0, 255, 255), true);
					break;
				}
				case PSMTrackingColorType::PSMTrackingColorType_Yellow:
				{
					ImGui::ColorButton(ImColor(255, 255, 0), true);
					break;
				}
				default:
				{
					ImGui::ColorButton(ImColor(255, 255, 255), true);
					break;
				}
				}
				ImGui::SameLine();
				ImGui::TextColored(textColor, k_tracking_color_names[m_masterTrackingColorType]);
			}
			ImGui::End();

			ImGui::GetStyle().WindowFillAlphaDefault = prevAlpha;
			ImGui::GetStyle().WindowRounding = prevRound;
		}
	}

    const float k_panel_width = 325.f;
	const char *k_window_title = "Color Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove;

	if (m_menuState > eMenuState::detection_init && m_menuState < eMenuState::detection_fail)
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(550, 150));
		ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

		ImGui::Text(
			"Color sampling is in progress! Please wait...\n"
			"Do not move the controllers or obscure the controller bulb!\n"
			"[sampling colors...]"
		);
		

		float total_progress;

		if (m_bAutoChangeTracker)
		{
			total_progress = static_cast<float>(m_controllerViews.size() * tracker_count);
		}
		else if (m_bAutoChangeController)
		{
			total_progress = static_cast<float>(m_controllerViews.size());
		}
		else
		{
			total_progress = 1.0f;
		}
		
		ImGui::ProgressBar(fmax(1.0f - (static_cast<float>(m_iDetectingControllersLeft) / total_progress), 0.0f));

		if (ImGui::Button("Cancel"))
		{
			m_bDetectingCancel = true;
		}

		ImGui::End();
	}

    switch (m_menuState)
    {
    case eMenuState::manualConfig:
    {
		if (m_bAlignPinned)
		{
			m_bAlignPinned = false;
		}

		if (m_video_buffer_state != nullptr)
		{
			if (m_bAlignDetectColor && ImGui::IsMouseClicked(0))
			{
				ImVec2 mousePos = ImGui::GetMousePos();
				ImVec2 dispSize = ImGui::GetIO().DisplaySize;
				int img_x = (static_cast<int>(mousePos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
				int img_y = (static_cast<int>(mousePos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
				cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

				TrackerColorPreset preset = getColorPreset();
				preset.hue_center = hsv_pixel[0];
				preset.saturation_center = hsv_pixel[1];
				preset.value_center = hsv_pixel[2];

				if (m_masterControllerView != nullptr)
				{
					auto_adjust_color_sensitivity(preset, m_masterControllerView->ControllerType != PSMController_Virtual && !is_tracker_virtual());
				}
				else
				{
					auto_adjust_color_sensitivity(preset, !is_tracker_virtual());
				}
				request_tracker_set_color_preset(m_masterTrackingColorType, preset);

				if (m_masterControllerView != nullptr)
				{
					if (m_bAutoChangeColor) {
						m_mAlignPosition[0] = mousePos.x;
						m_mAlignPosition[1] = mousePos.y;
						m_bAlignPinned = true;

						setState(eMenuState::autoConfig_wait1);
						request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType_Magenta);
						m_masterTrackingColorType = PSMTrackingColorType_Magenta;
					}
					else if (m_bAutoChangeController) {
						setState(eMenuState::changeController);
					}
					else if (m_bAutoChangeTracker) {
						setState(eMenuState::changeTracker);
					}
					else
					{
						m_bAlignDetectColor = false;
					}
				}
				else if (m_hmdView != nullptr)
				{
					if (m_bAutoChangeTracker) {
						setState(eMenuState::changeTracker);
					}
					else
					{
						m_bAlignDetectColor = false;
					}
				}
			}
		}
		else
		{
			m_bAlignDetectColor = false;
		}

		if (m_bAlignDetectColor && ImGui::IsMouseClicked(1))
		{
			m_bAlignDetectColor = false;
		}


        // Video Control Panel
        if (m_bShowWindows && !m_bAlignDetectColor)
        {
			static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

            ImGui::SetNextWindowPos(ImVec2(10.f, 10.f));
			ImGui::SetNextWindowSize(ImVec2(k_panel_width, fminf(lastWindowVec.y + 32.f, ImGui::GetIO().DisplaySize.y - 32)));
            ImGui::Begin(k_window_title, nullptr, window_flags);
			ImGui::BeginGroup();
			{
				if (ImGui::Button("Return to Tracker Settings"))
				{
					request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
				}

				ImGui::Separator();

				// $TODO: Not needed?
				/*if (m_video_buffer_state != nullptr)*/
				{
					int displayMode = m_videoDisplayMode;
					ImGui::Text("Video Preview:");
					ImGui::PushItemWidth(260.f);
					if (ImGui::Combo("##VideoFilterMode", &displayMode, "Color (BGR)\0Hue, Saturation, Value (HSV)\0Masked\0\0"))
					{
						m_videoDisplayMode = static_cast<eVideoDisplayMode>(displayMode);
					}
					ImGui::PopItemWidth();

					if (ImGui::CollapsingHeader("Advanced Settings", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##TrackerAdvancedSettingsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							if (is_tracker_virtual())
							{
								if (ImGui::Button(" - ##FrameWidth"))
								{
									if (m_trackerFrameWidth != 640) 
									{
										// Assume 4:3 ratio
										request_tracker_set_frame_width(640);
									}
								}
								ImGui::SameLine();
								if (ImGui::Button(" + ##FrameWidth"))
								{
									if (m_trackerFrameWidth != 1920)
									{
										// Assume 16:9 ratio
										request_tracker_set_frame_width(1920);
									}
								}
								ImGui::SameLine();
								ImGui::Text("Frame Width: %.0f", m_trackerFrameWidth);
							}
							else
							{
								if (ImGui::Button(" - ##FrameWidth"))
								{
									if (m_trackerFrameWidth != 320) request_tracker_set_frame_width(320);
								}
								ImGui::SameLine();
								if (ImGui::Button(" + ##FrameWidth"))
								{
									if (m_trackerFrameWidth != 640) request_tracker_set_frame_width(640);
								}
								ImGui::SameLine();
								ImGui::Text("Frame Width: %.0f", m_trackerFrameWidth);

								int frame_rate_positive_change = 10;
								int frame_rate_negative_change = -10;

								double val = m_trackerFrameRate;
								if (m_trackerFrameWidth == 320)
								{
									if (val == 2) { frame_rate_positive_change = 1; frame_rate_negative_change = 0; }
									else if (val == 3) { frame_rate_positive_change = 2; frame_rate_negative_change = -1; }
									else if (val == 5) { frame_rate_positive_change = 2; frame_rate_negative_change = -0; }
									else if (val == 7) { frame_rate_positive_change = 3; frame_rate_negative_change = -2; }
									else if (val == 10) { frame_rate_positive_change = 2; frame_rate_negative_change = -3; }
									else if (val == 12) { frame_rate_positive_change = 3; frame_rate_negative_change = -2; }
									else if (val == 15) { frame_rate_positive_change = 4; frame_rate_negative_change = -3; }
									else if (val == 17) { frame_rate_positive_change = 13; frame_rate_negative_change = -4; }
									else if (val == 30) { frame_rate_positive_change = 7; frame_rate_negative_change = -13; }
									else if (val == 37) { frame_rate_positive_change = 3; frame_rate_negative_change = -7; }
									else if (val == 40) { frame_rate_positive_change = 10; frame_rate_negative_change = -3; }
									else if (val == 50) { frame_rate_positive_change = 10; frame_rate_negative_change = -10; }
									else if (val == 60) { frame_rate_positive_change = 15; frame_rate_negative_change = -10; }
									else if (val == 75) { frame_rate_positive_change = 15; frame_rate_negative_change = -15; }
									else if (val == 90) { frame_rate_positive_change = 10; frame_rate_negative_change = -15; }
									else if (val == 100) { frame_rate_positive_change = 25; frame_rate_negative_change = -10; }
									else if (val == 125) { frame_rate_positive_change = 12; frame_rate_negative_change = -25; }
									else if (val == 137) { frame_rate_positive_change = 13; frame_rate_negative_change = -12; }
									else if (val == 150) { frame_rate_positive_change = 37; frame_rate_negative_change = -13; }
									else if (val == 187) { frame_rate_positive_change = 0; frame_rate_negative_change = -37; }
									else if (val == 205) { frame_rate_positive_change = 0; frame_rate_negative_change = -18; }
									else if (val == 290) { frame_rate_positive_change = 0; frame_rate_negative_change = -85; }
								}
								else
								{
									if (val == 2) { frame_rate_positive_change = 1; frame_rate_negative_change = 0; }
									else if (val == 3) { frame_rate_positive_change = 2; frame_rate_negative_change = -1; }
									else if (val == 5) { frame_rate_positive_change = 3; frame_rate_negative_change = -0; }
									else if (val == 8) { frame_rate_positive_change = 2; frame_rate_negative_change = -3; }
									else if (val == 10) { frame_rate_positive_change = 5; frame_rate_negative_change = -2; }
									else if (val == 15) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
									else if (val == 20) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
									else if (val == 25) { frame_rate_positive_change = 5; frame_rate_negative_change = -5; }
									else if (val == 30) { { frame_rate_negative_change = -5; } }
									else if (val == 60) { { frame_rate_positive_change = 15; } }
									else if (val == 75) { frame_rate_positive_change = 0; frame_rate_negative_change = -15; }
									else if (val == 83) { frame_rate_positive_change = 0; frame_rate_negative_change = -8; }
								}

								if (ImGui::Button(" - ##FrameRate"))
								{
									request_tracker_set_frame_rate(m_trackerFrameRate + frame_rate_negative_change);
								}
								ImGui::SameLine();
								if (ImGui::Button(" + ##FrameRate"))
								{
									request_tracker_set_frame_rate(m_trackerFrameRate + frame_rate_positive_change);
								}
								ImGui::SameLine();
								ImGui::Text("Frame Rate: %.0f", m_trackerFrameRate);

								if (ImGui::Button(" - ##Exposure"))
								{
									request_tracker_set_exposure(m_trackerExposure - 8);
								}
								ImGui::SameLine();
								if (ImGui::Button(" + ##Exposure"))
								{
									request_tracker_set_exposure(m_trackerExposure + 8);
								}
								ImGui::SameLine();
								ImGui::Text("Exposure: %.0f", m_trackerExposure);

								if (ImGui::Button(" - ##Gain"))
								{
									request_tracker_set_gain(m_trackerGain - 8);
								}
								ImGui::SameLine();
								if (ImGui::Button(" + ##Gain"))
								{
									request_tracker_set_gain(m_trackerGain + 8);
								}
								ImGui::SameLine();
								ImGui::Text("Gain: %.0f", m_trackerGain);

								// Render all of the option sets fetched from the settings query
								for (auto it = m_trackerOptions.begin(); it != m_trackerOptions.end(); ++it)
								{
									TrackerOption &option = *it;
									const int value_count = static_cast<int>(option.option_strings.size());

									ImGui::PushID(option.option_name.c_str());
									if (ImGui::Button(" < ##CustomProperty"))
									{
										request_tracker_set_option(option, (option.option_index + value_count - 1) % value_count);
									}
									ImGui::SameLine();
									if (ImGui::Button(" > ##CustomProperty"))
									{
										request_tracker_set_option(option, (option.option_index + 1) % value_count);
									}
									ImGui::SameLine();
									ImGui::Text("%s: %s", option.option_name.c_str(), option.option_strings[option.option_index].c_str());
									ImGui::PopID();
								}
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}

					if (ImGui::CollapsingHeader("Blacklisted Areas", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##BlacklistedAreasChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							ImGui::Checkbox("Show Blacklisted Areas", &m_bProjectionBlacklistedShow);

							for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
							{
								std::string option_name = std::to_string(i);

								ImGui::PushID(option_name.c_str());
								if (ImGui::CollapsingHeader(option_name.c_str(), 0, true, true))
								{
									float val;

									ImGui::Text("X: ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									val = m_blacklisted_projection[i].x;
									if (ImGui::InputFloat("##BlacklistedAreaX", &val, 4.f, 32.f, 2))
									{
										m_blacklisted_projection[i].x = fmaxf(0.f, val);
										request_tracker_set_projectionblacklist(m_blacklisted_projection);
									}
									ImGui::PopItemWidth();

									ImGui::Text("Y: ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									val = m_blacklisted_projection[i].y;
									if (ImGui::InputFloat("##BlacklistedAreaY", &val, 4.f, 32.f, 2))
									{
										m_blacklisted_projection[i].y = fmaxf(0.f, val);
										request_tracker_set_projectionblacklist(m_blacklisted_projection);
									}
									ImGui::PopItemWidth();

									ImGui::Text("Width: ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									val = m_blacklisted_projection[i].w;
									if (ImGui::InputFloat("##BlacklistedAreaW", &val, 4.f, 32.f, 2))
									{
										m_blacklisted_projection[i].w = fmaxf(0.f, val);
										request_tracker_set_projectionblacklist(m_blacklisted_projection);
									}
									ImGui::PopItemWidth();

									ImGui::Text("Height: ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									val = m_blacklisted_projection[i].h;
									if (ImGui::InputFloat("##BlacklistedAreaH", &val, 4.f, 32.f, 2))
									{
										m_blacklisted_projection[i].h = fmaxf(0.f, val);
										request_tracker_set_projectionblacklist(m_blacklisted_projection);
									}
									ImGui::PopItemWidth();

									if (ImGui::Button("Reset"))
									{
										m_blacklisted_projection[i].x = 0.f;
										m_blacklisted_projection[i].y = 0.f;
										m_blacklisted_projection[i].w = 0.f;
										m_blacklisted_projection[i].h = 0.f;
										request_tracker_set_projectionblacklist(m_blacklisted_projection);
									}
								}
								ImGui::PopID();
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}

					if (m_masterControllerView != nullptr)
					{
						if (ImGui::Checkbox("Turn on all bulbs", &m_bTurnOnAllControllers))
						{
							request_turn_on_all_tracking_bulbs(m_bTurnOnAllControllers || m_bColorCollsionShow);
						}

						if (ImGui::Checkbox("Show color collisions", &m_bColorCollsionShow))
						{
							request_turn_on_all_tracking_bulbs(m_bTurnOnAllControllers || m_bColorCollsionShow);
						}
					}
					else if (m_hmdView != nullptr)
					{
						// There are no bulbs to turn on.
						ImGui::Checkbox("Show color collisions", &m_bColorCollsionShow);
					}

					ImGui::Separator();

					std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
					std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastStreamFps;
					if (timeSinceLast.count() > 1000.f)
					{
						m_displayFps = m_streamFps;
						m_streamFps = 0;
						m_lastStreamFps = now;
					}
					if (m_displayFps < m_trackerFrameRate - 7.5f)
					{
						ImGui::TextColored(ImColor(1.f, 0.f, 0.f), "Tracker Frame Rate: %d", m_displayFps);
					}
					else
					{
						ImGui::Text("Tracker Frame Rate: %d", m_displayFps);
					}

					ImGui::Separator();

					if (ImGui::CollapsingHeader("Miscellaneous"))
					{
						if (ImGui::Button("Save Default Profile"))
						{
							request_save_default_tracker_profile();
						}

						if (ImGui::Button("Apply Default Profile"))
						{
							request_apply_default_tracker_profile();
						}
					}
				}
			}
			ImGui::EndGroup();
			if (ImGui::IsItemVisible())
				lastWindowVec = ImGui::GetItemRectSize();

            ImGui::End();
        }
        
        // Color Control Panel
		if (m_bShowWindows && !m_bAlignDetectColor)
		{
			static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - k_panel_width - 10, 10.f));
			ImGui::SetNextWindowSize(ImVec2(k_panel_width, fminf(lastWindowVec.y + 32.f, ImGui::GetIO().DisplaySize.y - 32)));
			ImGui::Begin("Controller Color", nullptr, window_flags);
			ImGui::BeginGroup();
			{

				if (m_masterControllerView != nullptr)
				{
					if (ImGui::Button(" < ##Color"))
					{
						PSMTrackingColorType new_color =
							static_cast<PSMTrackingColorType>(
							(m_masterTrackingColorType + PSMTrackingColorType_MaxColorTypes - 1)
								% PSMTrackingColorType_MaxColorTypes);
						request_set_controller_tracking_color(m_masterControllerView, new_color);
						m_masterTrackingColorType = new_color;
					}
					ImGui::SameLine();
					if (ImGui::Button(" > ##Color"))
					{
						PSMTrackingColorType new_color =
							static_cast<PSMTrackingColorType>(
							(m_masterTrackingColorType + 1) % PSMTrackingColorType_MaxColorTypes);
						request_set_controller_tracking_color(m_masterControllerView, new_color);
						m_masterTrackingColorType = new_color;
					}
					ImGui::SameLine();
				}
				ImGui::Text("Tracking Color: %s", k_tracking_color_names[m_masterTrackingColorType]);

				if (ImGui::CollapsingHeader("Advanced Settings", 0, true, false))
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##ColorAdvancedSettingsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						// -- Hue --
						if (ImGui::Button(" - ##HueCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.hue_center = wrap_range(preset.hue_center - 5.f, 0.f, 180.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##HueCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.hue_center = wrap_range(preset.hue_center + 5.f, 0.f, 180.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Hue Angle: %f", getColorPreset().hue_center);

						if (ImGui::Button(" - ##HueRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.hue_range = clampf(preset.hue_range - 5.f, 0.f, 90.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##HueRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.hue_range = clampf(preset.hue_range + 5.f, 0.f, 90.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Hue Range: %f", getColorPreset().hue_range);

						// -- Saturation --
						if (ImGui::Button(" - ##SaturationCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.saturation_center = clampf(preset.saturation_center - 5.f, 0.f, 255.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##SaturationCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.saturation_center = clampf(preset.saturation_center + 5.f, 0.f, 255.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Saturation Center: %f", getColorPreset().saturation_center);

						if (ImGui::Button(" - ##SaturationRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.saturation_range = clampf(preset.saturation_range - 5.f, 0.f, 125.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##SaturationRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.saturation_range = clampf(preset.saturation_range + 5.f, 0.f, 125.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Saturation Range: %f", getColorPreset().saturation_range);

						// -- Value --
						if (ImGui::Button(" - ##ValueCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.value_center = clampf(preset.value_center - 5.f, 0.f, 255.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##ValueCenter"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.value_center = clampf(preset.value_center + 5.f, 0.f, 255.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Value Center: %f", getColorPreset().value_center);

						if (ImGui::Button(" - ##ValueRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.value_range = clampf(preset.value_range - 5.f, 0.f, 125.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						if (ImGui::Button(" + ##ValueRange"))
						{
							TrackerColorPreset preset = getColorPreset();
							preset.value_range = clampf(preset.value_range + 5.f, 0.f, 125.f);
							request_tracker_set_color_preset(m_masterTrackingColorType, preset);
						}
						ImGui::SameLine();
						ImGui::Text("Value Range: %f", getColorPreset().value_range);
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				// -- Change Controller --
				if (m_masterControllerView != nullptr)
				{
					if (ImGui::Button(" < ##Controller"))
					{
						request_change_controller(-1);
						request_change_tracker(0);
					}
					ImGui::SameLine();
					if (ImGui::Button(" > ##Controller"))
					{
						request_change_controller(1);
						request_change_tracker(0);
					}
					ImGui::SameLine();
					ImGui::Text("PSMove Controller ID: %d", m_overrideControllerId);
				}

				// -- Change Tracker --
				if (ImGui::Button(" < ##Tracker"))
				{
					request_change_tracker(-1);
				}
				ImGui::SameLine();
				if (ImGui::Button(" > ##Tracker"))
				{
					request_change_tracker(1);
				}
				ImGui::SameLine();
				ImGui::Text("Tracker ID: %d", tracker_index);

				if (m_masterControllerView != nullptr)
				{
					if (ImGui::Button("Test Tracking Pose"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestControllerTracking(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
					ImGui::SameLine();
					if (ImGui::Button("Test One"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingControllerVideo(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
					ImGui::SameLine();
					if (ImGui::Button("Test All"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingVideoALL(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
				}
				else if (m_hmdView != nullptr)
				{
					if (ImGui::Button("Test Tracking Pose"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestHMDTracking(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
					ImGui::SameLine();
					if (ImGui::Button("Test One"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingHMDVideo(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
					ImGui::SameLine();
					if (ImGui::Button("Test All"))
					{
						m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingVideoALL(true);
						request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
					}
				}

				// -- Auto Calibration --
				if (ImGui::CollapsingHeader("Color Detection", 0, true, true))
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##ColorDetectionChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						if (m_masterControllerView != nullptr)
						{
							if (ImGui::Button("Automatically Detect Colors"))
							{
								setState(eMenuState::detection_init);
							}
						}

						if (ImGui::Button("Manually Detect Colors"))
						{
							m_bAlignDetectColor = true;
						}

						if (m_masterControllerView != nullptr)
						{
							ImGui::Checkbox("Automatically switch color", &m_bAutoChangeColor);
							if (ImGui::IsItemHovered())
								ImGui::SetTooltip(
									"Cycles through all available colors automatically.\n"
									"(This is always enabled when using 'Automatically detect colors')"
								);

							ImGui::Checkbox("Automatically switch controller", &m_bAutoChangeController);
							if (ImGui::IsItemHovered())
								ImGui::SetTooltip(
									"Cycles through all available controllers automatically."
								);
						}

						ImGui::Checkbox("Automatically switch tracker", &m_bAutoChangeTracker);
						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Cycles through all available trackers automatically."
							);

						if (ImGui::CollapsingHeader("Automatic Detection Settings", 0, true, true))
						{
							static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##AutoColorDetectionSettingsChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
							ImGui::BeginGroup();
							{
								int adjustMethod = m_iDetectingAdjustMethod;
								ImGui::Text("Automatic exposure/gain options:");
								if (ImGui::Combo("##DetectAdjustMethod", &adjustMethod, "Keep Settings\0Adjust Exposure\0Adjust Gain\0\0"))
								{
									m_iDetectingAdjustMethod = static_cast<eDetectionAdjustMethod>(adjustMethod);
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec2 = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}

						if (ImGui::CollapsingHeader("Automatic/Manual Detection Settings", 0, true, true))
						{
							static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##AutoManualColorDetectionSettingsChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
							ImGui::BeginGroup();
							{
								int colorSensitivity = m_iColorSensitivity;
								ImGui::Text("Color detection sensitivity:");
								if (ImGui::Combo("##SensitivityPostProcessing", &colorSensitivity, "Keep Settings\0Normal Sensitivity\0High Sensitivity\0Aggressive Sensitivity\0Extreme Sensitivity\0\0"))
								{
									if (colorSensitivity >= sensitivity_MAX)
										colorSensitivity = sensitivity_MAX - 1;

									m_iColorSensitivity = static_cast<eColorDetectionSensitivity>(colorSensitivity);
								}

								if (ImGui::IsItemHovered())
									ImGui::SetTooltip(
										"Automatically adjusts the color hue, hue range, saturation center,\n"
										"saturation range, value center and value range while color sampling.\n"
										"Using higher sensitivity can help improve tracking quality and\n"
										"tracking range but also creates more color noise and collisions\n"
										"between colors!"
									);

								if (m_iColorSensitivity > sensitivity_disabled) {
									ImGui::Checkbox("Prevent color collisions", &m_bColorCollisionPrevent);

									if (ImGui::IsItemHovered())
										ImGui::SetTooltip(
											"Adjusts the hue range to avoid collisions between\n"
											"controller colors and potential color noise while color sampling.\n"
											"This will reduce tracking quality if enabled."
										);
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec2 = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}

					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (ImGui::CollapsingHeader("Warnings and Issues", 0, true, true))
				{
					ImColor colorGreen = ImColor(0.f, 1.f, 0.f);
					ImColor colorOrange = ImColor(1.f, .5f, 0.f);
					ImColor colorRed = ImColor(1.f, 0.f, 0.f);
					ImColor colorBlue = ImColor(0.f, 0.25f, 1.f);

					bool bHasIssues = false;

					{
						// Tell if its a virtual device.
						if (is_tracker_virtual())
						{
							ImGui::ColorButton(colorBlue, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Tracker is virtual. "
								"Some settings on this page might not be available or have to be set manually."
							);
						}

						if (m_masterControllerView != nullptr && m_masterControllerView->ControllerType == PSMController_Virtual)
						{
							ImGui::ColorButton(colorBlue, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Controller is virtual. "
								"Some settings on this page might not be available or have to be set manually."
							);
						}

						if (m_hmdView != nullptr && m_hmdView->HmdType == PSMHmd_Virtual)
						{
							ImGui::ColorButton(colorBlue, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"HMD is virtual. "
								"Some settings on this page might not be available or have to be set manually."
							);
						}
					}

					{
						// Recommend exposure adjustments rather than gain adjustments
						if (!is_tracker_virtual() && m_trackerGain > 32)
						{
							ImGui::ColorButton(colorBlue, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Tracker gain not default. "
								"It's recommended to adjust exposure instead of gain. "
								"Increasing gain will increase random color noise which can negatively affect tracking quality."
							);
						}
					}

					{
						// Recommend higher fps
						if (!is_tracker_virtual() && m_trackerFrameRate < 30)
						{
							ImGui::ColorButton(colorBlue, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Tracker frame rate too low. "
								"It's recommended to run trackers at least at 30 fps or higher. "
								"Lowering the frame rate of the tracker below 30 fps can lead to less responsive tracking, input lag and tracking loss on fast movements."
							);
						}
					}

					{
						// Validate Exposure/Gain
						TrackerColorPreset preset = getColorPreset();

						// Saturation too low, its too bright!
						if (preset.saturation_center < 40)
						{
							ImGui::ColorButton(colorRed, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped("Color saturation too low! The tracking color is way too bright and will cause tracking problems! Adjust your tracker exposure/gain settings!");
							bHasIssues = true;
						}
						else if (preset.saturation_center < 80)
						{
							ImGui::ColorButton(colorOrange, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped("Color saturation not optimal! The tracking color is too bright and could cause tracking problems! Adjust your tracker exposure/gain settings.");
							bHasIssues = true;
						}

						// Value too low, its too dark!
						if (preset.value_center < 40)
						{
							ImGui::ColorButton(colorOrange, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped("Color value too low! The tracking color is way too dark and could cause tracking problems! Adjust your tracker exposure/gain settings!");
							bHasIssues = true;
						}
						else if (preset.value_center < 80)
						{
							ImGui::ColorButton(colorOrange, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped("Color value not optimal! The tracking color is too dark and could cause tracking problems! Adjust your tracker exposure/gain settings.");
							bHasIssues = true;
						}
					}

					{
						// Validate color settings
						TrackerColorPreset preset = getColorPreset();
						switch (m_masterTrackingColorType)
						{
						case PSMTrackingColorType::PSMTrackingColorType_Red:
						{
							const int targetHue = 0;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center > ((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center > ((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center > ((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to RED but the target hue center is not RED. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Green:
						{
							int targetHue = 60;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center >((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to GREEN but the target hue center is not GREEN. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Blue:
						{
							int targetHue = 120;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center >((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to BLUE but the target hue center is not BLUE. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Magenta:
						{
							int targetHue = 150;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center >((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to MAGENTA but the target hue center is not MAGENTA. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Cyan:
						{
							int targetHue = 90;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center >((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to CYAN but the target hue center is not CYAN. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Yellow:
						{
							int targetHue = 30;
							const int targetRange = 25;
							const float hue_min = preset.hue_center - targetRange;
							const float hue_max = preset.hue_center + targetRange;
							bool invalidHue = false;
							if (hue_min < 0)
							{
								invalidHue = (preset.hue_center < ((targetHue + (180 - targetRange)) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else if (hue_max > 180)
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) && preset.hue_center >((targetHue + targetRange) % 180));
							}
							else
							{
								invalidHue = (preset.hue_center < ((targetHue - targetRange) % 180) || preset.hue_center >((targetHue + targetRange) % 180));
							}

							if (invalidHue)
							{
								ImGui::ColorButton(colorRed, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Wrong tracking color set! "
									"The target tracking color is set to YELLOW but the target hue center is not YELLOW. "
									"This can cause collisions with other colors! "
									"Adjust your tracking color settings!"
								);
								bHasIssues = true;
							}

							break;
						}
						case PSMTrackingColorType::PSMTrackingColorType_Custom0:
						case PSMTrackingColorType::PSMTrackingColorType_Custom1:
						case PSMTrackingColorType::PSMTrackingColorType_Custom2:
						case PSMTrackingColorType::PSMTrackingColorType_Custom3:
						case PSMTrackingColorType::PSMTrackingColorType_Custom4:
						case PSMTrackingColorType::PSMTrackingColorType_Custom5:
						case PSMTrackingColorType::PSMTrackingColorType_Custom6:
						case PSMTrackingColorType::PSMTrackingColorType_Custom7:
						case PSMTrackingColorType::PSMTrackingColorType_Custom8:
						case PSMTrackingColorType::PSMTrackingColorType_Custom9:
						{
							if (m_masterControllerView != nullptr && m_masterControllerView->ControllerType == PSMController_Move)
							{
								ImGui::ColorButton(colorBlue, true);
								if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
								ImGui::SameLine();
								ImGui::TextWrapped(
									"Custom preset color set. PSmove bulb has been turned off."
								);
							}

							break;
						}
						}
					}

					{
						// Get the avg of detected contures to avoid frequent UI changes.
						const int DETECTED_CONTURES_SMOOTH_SIZE = 32;
						static int detectedContures = 0;
						static int detectedConturesAvg = 0;
						static int detectedConturesCount = 0;

						detectedConturesAvg += static_cast<int>(m_mDetectedContures.size());
						if (++detectedConturesCount > DETECTED_CONTURES_SMOOTH_SIZE)
						{
							detectedContures = static_cast<int>(floor(detectedConturesAvg / DETECTED_CONTURES_SMOOTH_SIZE));
							detectedConturesAvg = 0;
							detectedConturesCount = 0;
						}

						// Check for color noise and if the color can be found at all.
						switch (detectedContures)
						{
						case 0:
						{
							ImGui::ColorButton(colorRed, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Could not detect tracking color! Place your device in view of the tracker. "
								"If it already is, then your color settings are not correctly set up."
							);
							bHasIssues = true;
							break;
						}
						case 1:
						case 2:
						{
							// Optimal
							break;
						}
						default:
						{
							ImGui::ColorButton(colorRed, true);
							if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip
							ImGui::SameLine();
							ImGui::TextWrapped(
								"Color noise/collisions detected! "
								"The tracker could track different objects instead of the device you want to track! "
								"Enable 'Show color collisions' to show color collisions on screen. "
								"Adjust your color settings to avoid color noise."
							);
							bHasIssues = true;
							break;
						}
						}
					}

					if (!bHasIssues)
					{
						ImGui::ColorButton(colorGreen, true);
						if (ImGui::IsItemHovered()) ImGui::SetTooltip(""); // Disable color tooltip

						ImGui::SameLine();
						ImGui::TextWrapped("No issues detected.");
					}
				}
			}
			ImGui::EndGroup();
			if (ImGui::IsItemVisible())
				lastWindowVec = ImGui::GetItemRectSize();

            ImGui::End();
        }
        

    } break;

    case eMenuState::autoConfig:
    {
		if (!m_bAlignPinned)
		{
			setState(eMenuState::manualConfig);
			break;
		}

		if (m_video_buffer_state == nullptr)
		{
			m_iDetectingFailReason = eDetectionFailReason::failreason_unknown;
			setState(eMenuState::detection_fail_pre);
			break;
		}

        PSMTrackingColorType new_color =
            static_cast<PSMTrackingColorType>(
            (m_masterTrackingColorType + 1) % PSMTrackingColorType_Custom0); //PSMTrackingColorType_MaxColorTypes

		ImVec2 dispSize = ImGui::GetIO().DisplaySize;
		int img_x = (static_cast<int>(m_mAlignPosition[0]) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
		int img_y = (static_cast<int>(m_mAlignPosition[1]) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
		cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

		TrackerColorPreset preset = getColorPreset();
		preset.hue_center = hsv_pixel[0];
		preset.saturation_center = hsv_pixel[1];
		preset.value_center = hsv_pixel[2];

		auto_adjust_color_sensitivity(preset, m_masterControllerView->ControllerType != PSMController_Virtual && !is_tracker_virtual());
		request_tracker_set_color_preset(m_masterTrackingColorType, preset);
		request_set_controller_tracking_color(m_masterControllerView, new_color);

        if (new_color == PSMTrackingColorType_Magenta) 
		{
			if (m_bAutoChangeController)
			{
				setState(eMenuState::changeController);
			}
			else if (m_bAutoChangeTracker)
			{
				setState(eMenuState::changeTracker);
			}
			else 
			{
				setState(eMenuState::manualConfig);
			}
        }
		else
		{
			setState(eMenuState::autoConfig_wait1);
		}

        m_masterTrackingColorType = new_color;
    } break;

	case eMenuState::detection_init:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(600, 150));
		ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

		if (!m_areAllControllerStreamsActive)
		{
			ImGui::Text("Wait for controller streams to be active...");
			break;
		}

		m_videoDisplayMode = eVideoDisplayMode::mode_bgr;
		m_bTurnOnAllControllers = false;
		m_bColorCollsionShow = false;
		m_bProjectionBlacklistedShow = false;
		request_turn_on_all_tracking_bulbs(false);

		m_bDetectingCancel = false;

		int stable_controllers = 0;
		int stable_total_controllers = 0;

		if (m_bAutoChangeTracker || m_bAutoChangeController)
		{
			stable_total_controllers = static_cast<int>(m_controllerViews.size());

			for (int i = 0; i < m_controllerViews.size(); i++)
			{
				PSMController *controllerView = m_controllerViews[i];

				bool bIsStable;
				bool bCanBeStabilized = (PSM_GetIsControllerStable(controllerView->ControllerID, &bIsStable) == PSMResult_Success);

				if (bCanBeStabilized)
				{
					if (bIsStable)
					{
						stable_controllers++;
					}
				}
				else
				{
					stable_controllers++;
				}
			}
		}
		else
		{
			stable_total_controllers = 1;

			bool bIsStable;
			bool bCanBeStabilized = (PSM_GetIsControllerStable(m_masterControllerView->ControllerID, &bIsStable) == PSMResult_Success);

			if (bCanBeStabilized)
			{
				if (bIsStable)
				{
					stable_controllers++;
				}
			}
			else
			{
				stable_controllers++;
			}
		}

		ImGui::TextWrapped(
			"Place all controllers in the middle of your play space so all trackers can see them. "
			"Do not obscure the controller bulb while the sampling process is running otherwise the "
			"sampled colors might be inaccurate!"
		);

		ImGui::Spacing();

		ImGui::Text("Stable controllers: %d / %d", stable_controllers, stable_total_controllers);

		if (stable_controllers == stable_total_controllers)
		{
			if (ImGui::Button("Start Sampling Colors"))
			{
				if (m_bAutoChangeTracker)
				{
					m_iDetectingControllersLeft = static_cast<int>(stable_total_controllers * tracker_count);
				}
				else if (m_bAutoChangeController)
				{
					m_iDetectingControllersLeft = static_cast<int>(stable_total_controllers);
				}
				else
				{
					m_iDetectingControllersLeft = 1;
				}

				m_iDetectingExposure = k_color_autodetect_probe_step;
				setState(eMenuState::detection_exposure_adjust);
			}
		}
		else
		{
			if (ImGui::Button("Force Start Sampling Colors"))
			{
				if (m_bAutoChangeTracker)
				{
					m_iDetectingControllersLeft = static_cast<int>(stable_total_controllers * tracker_count);
				}
				else if(m_bAutoChangeController)
				{
					m_iDetectingControllersLeft = static_cast<int>(stable_total_controllers);
				}
				else 
				{
					m_iDetectingControllersLeft = 1;
				}

				m_iDetectingExposure = k_color_autodetect_probe_step;
				setState(eMenuState::detection_exposure_adjust);
			}
		}

		ImGui::SameLine();

		if (ImGui::Button("Cancel"))
		{
			setState(eMenuState::manualConfig);
		}


		ImGui::End();
		break;
	}
	case eMenuState::detection_exposure_adjust:
	{
		m_bDetectingColors = true;

		switch (m_iDetectingAdjustMethod)
		{
		case eDetectionAdjustMethod::adjust_exposure:
		{
			request_tracker_set_exposure(m_iDetectingExposure);
			request_tracker_set_gain(32);
			break;
		}
		case eDetectionAdjustMethod::adjust_gain:
		{
			request_tracker_set_exposure(32);
			request_tracker_set_gain(m_iDetectingExposure);
			break;
		}
		}

		setState(eMenuState::detection_exposure_wait1);
		break;
	}
	case eMenuState::detection_exposure_wait1:
	{
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::detection_exposure_wait2);
		break;
	}
	case eMenuState::detection_exposure_wait2:
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::detection_get_red);
		}
		break;
	}
	case eMenuState::detection_get_red:
	{
		TrackerColorPreset preset = getColorPreset();
		preset.hue_center = 0;
		preset.hue_range = 25;
		preset.saturation_center = 255;
		preset.saturation_range = 125;
		preset.value_center = 255;
		preset.value_range = 125;

		request_tracker_set_color_preset(PSMTrackingColorType_Red, preset);

		request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType::PSMTrackingColorType_Red);
		m_masterTrackingColorType = PSMTrackingColorType::PSMTrackingColorType_Red;

		setState(eMenuState::detection_get_red_wait1);
		break;
	}
	case eMenuState::detection_get_red_wait1:
	{
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::detection_get_red_wait2);
		break;
	}
	case eMenuState::detection_get_red_wait2:
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::detection_get_red_done);
		}
		break;
	}
	case eMenuState::detection_get_red_done:
	{
		if (m_video_buffer_state == nullptr)
		{
			m_iDetectingFailReason = eDetectionFailReason::failreason_unknown;
			setState(eMenuState::detection_fail_pre);
			break;
		}

		*m_video_buffer_state->detectionMaskedBuffer = cv::Scalar(0, 0, 0);

		cv::threshold(*m_video_buffer_state->maskedBuffer, *m_video_buffer_state->maskedBuffer, 0, 255, CV_THRESH_BINARY);

		cv::bitwise_or(
			*m_video_buffer_state->maskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer);

		cv::threshold(*m_video_buffer_state->detectionMaskedBuffer, *m_video_buffer_state->detectionMaskedBuffer, 0, 255, CV_THRESH_BINARY);

		setState(eMenuState::detection_get_green);
		break;
	}
	case eMenuState::detection_get_green:
	{
		TrackerColorPreset preset = getColorPreset();
		preset.hue_center = 60;
		preset.hue_range = 25;
		preset.saturation_center = 255;
		preset.saturation_range = 125;
		preset.value_center = 255;
		preset.value_range = 125;

		request_tracker_set_color_preset(PSMTrackingColorType_Green, preset);

		request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType::PSMTrackingColorType_Green);
		m_masterTrackingColorType = PSMTrackingColorType::PSMTrackingColorType_Green;

		setState(eMenuState::detection_get_green_wait1);
		break;
	}
	case eMenuState::detection_get_green_wait1:
	{
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::detection_get_green_wait2);
		break;
	}
	case eMenuState::detection_get_green_wait2: 
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::detection_get_green_done);
		}
		break;
	}
	case eMenuState::detection_get_green_done:
	{
		if (m_video_buffer_state == nullptr)
		{
			m_iDetectingFailReason = eDetectionFailReason::failreason_unknown;
			setState(eMenuState::detection_fail_pre);
			break;
		}

		cv::threshold(*m_video_buffer_state->maskedBuffer, *m_video_buffer_state->maskedBuffer, 0, 255, CV_THRESH_BINARY);

		cv::bitwise_and(
			*m_video_buffer_state->maskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer);

		cv::threshold(*m_video_buffer_state->detectionMaskedBuffer, *m_video_buffer_state->detectionMaskedBuffer, 0, 255, CV_THRESH_BINARY);

		setState(eMenuState::detection_get_blue);
		break;
	}
	case eMenuState::detection_get_blue:
	{
		TrackerColorPreset preset = getColorPreset();
		preset.hue_center = 120;
		preset.hue_range = 25;
		preset.saturation_center = 255;
		preset.saturation_range = 125;
		preset.value_center = 255;
		preset.value_range = 125;

		request_tracker_set_color_preset(PSMTrackingColorType_Blue, preset);

		request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType::PSMTrackingColorType_Blue);
		m_masterTrackingColorType = PSMTrackingColorType::PSMTrackingColorType_Blue;

		setState(eMenuState::detection_get_blue_wait1);
		break;
	}
	case eMenuState::detection_get_blue_wait1:
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::detection_get_blue_wait2);
		break;
	case eMenuState::detection_get_blue_wait2:
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::detection_get_blue_done);
		}
		break;
	}
	case eMenuState::detection_get_blue_done:
	{
		if (m_video_buffer_state == nullptr)
		{
			m_iDetectingFailReason = eDetectionFailReason::failreason_unknown;
			setState(eMenuState::detection_fail_pre);
			break;
		}

		cv::threshold(*m_video_buffer_state->maskedBuffer, *m_video_buffer_state->maskedBuffer, 0, 255, CV_THRESH_BINARY);

		cv::bitwise_and(
			*m_video_buffer_state->maskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer,
			*m_video_buffer_state->detectionMaskedBuffer);

		cv::threshold(*m_video_buffer_state->detectionMaskedBuffer, *m_video_buffer_state->detectionMaskedBuffer, 0, 255, CV_THRESH_BINARY);

		setState(eMenuState::detection_change_color_wait1);
		request_set_controller_tracking_color(m_masterControllerView, PSMTrackingColorType_Magenta);
		m_masterTrackingColorType = PSMTrackingColorType_Magenta;
		break;
	}
	case eMenuState::detection_change_color:
	{
		if (m_video_buffer_state == nullptr)
		{
			m_iDetectingFailReason = eDetectionFailReason::failreason_unknown;
			setState(eMenuState::detection_fail_pre);
			break;
		}

		cv::cvtColor(*m_video_buffer_state->detectionMaskedBuffer, *m_video_buffer_state->detectionLowerBuffer, cv::COLOR_BGR2GRAY);
		cv::threshold(*m_video_buffer_state->detectionLowerBuffer, *m_video_buffer_state->detectionLowerBuffer, 0, 255, CV_THRESH_BINARY);

		if (!m_bDetectingCancel)
		{
			std::vector<std::vector<int>> contures;
			get_contures_lower(1, 4, contures);

			// We can only change PSmoves and PS4 controllers colors. Abort on others.
			if (m_masterControllerView->ControllerType != PSMControllerType::PSMController_Move &&
				m_masterControllerView->ControllerType != PSMControllerType::PSMController_DualShock4)
			{
				m_iDetectingFailReason = eDetectionFailReason::failreason_unsupported_controller;
				setState(eMenuState::detection_fail_pre);
				break;
			}

			// We didnt got any results?
			// Controller either blocked by something or exposure too low.
			// Try adjusting exposure first.
			if (contures.size() == 0)
			{
				// If its a virtual tracker, don't even bother setting gain/exposure, just fail.
				if (is_tracker_virtual())
				{
					m_iDetectingFailReason = eDetectionFailReason::failreason_unsupported_tracker;
					setState(eMenuState::detection_fail_pre);
				}
				else
				{
					if (m_iDetectingAdjustMethod == eDetectionAdjustMethod::adjust_keep)
					{
						m_iDetectingFailReason = eDetectionFailReason::failreason_no_detection;
						setState(eMenuState::detection_fail_pre);
					}
					else
					{
						m_iDetectingExposure += k_color_autodetect_probe_step;

						if (m_iDetectingExposure >= k_color_autodetect_probe_max)
						{
							m_iDetectingFailReason = eDetectionFailReason::failreason_no_detection;
							setState(eMenuState::detection_fail_pre);
						}
						else
						{
							setState(eMenuState::detection_exposure_adjust);
						}
					}
					
				}
				break;
			}

			// We dont want to search for custom colors presets
			PSMTrackingColorType new_color =
				static_cast<PSMTrackingColorType>(
				(m_masterTrackingColorType + 1) % PSMTrackingColorType_Custom0); //PSMTrackingColorType_MaxColorTypes

			ImVec2 dispSize = ImGui::GetIO().DisplaySize;
			int img_x = (contures[0][0] * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
			int img_y = (contures[0][1] * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);
			cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->hsvBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

			TrackerColorPreset preset = getColorPreset();
			preset.hue_center = hsv_pixel[0];
			preset.hue_range = 10.f;
			preset.saturation_center = hsv_pixel[1];
			preset.saturation_range = 32.f;
			preset.value_center = hsv_pixel[2];
			preset.value_range = 32.f;

			auto_adjust_color_sensitivity(preset, m_masterControllerView->ControllerType != PSMController_Virtual && !is_tracker_virtual());
			request_tracker_set_color_preset(m_masterTrackingColorType, preset);
			request_set_controller_tracking_color(m_masterControllerView, new_color);

			if (new_color == PSMTrackingColorType_Magenta)
			{
				if (--m_iDetectingControllersLeft > 0)
				{
					setState(eMenuState::changeController);
				}
				else
				{
					setState(eMenuState::detection_finish);
				}
			}
			else
			{
				setState(eMenuState::detection_change_color_wait1);
			}

			m_masterTrackingColorType = new_color;
		}
		else
		{
			m_bDetectingCancel = false;
			m_iDetectingFailReason = eDetectionFailReason::failreason_canceled;
			setState(eMenuState::detection_fail_pre);
		}
		break;
	}
	case eMenuState::detection_change_color_wait1:
	{
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::detection_change_color_wait2);
		break;
	}
	case eMenuState::detection_change_color_wait2:
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::detection_change_color);
		}
		break;
	}
	case eMenuState::detection_fail_pre:
	{
		if (m_iDetectingAdjustMethod != eDetectionAdjustMethod::adjust_keep)
		{
			request_tracker_set_exposure(32);
			request_tracker_set_gain(32);
		}

		setState(eMenuState::detection_fail);
		break;
	}
	case eMenuState::detection_fail:
	{
		m_bDetectingColors = false;
		
		switch (m_iDetectingFailReason)
		{
		case eDetectionFailReason::failreason_no_detection:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 175));
			ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

			ImGui::TextWrapped("Color sampling failed!");
			ImGui::TextWrapped("Unable to find controller #%d on tracker #%d!", m_masterControllerView->ControllerID, m_trackerView->tracker_info.tracker_id);
		 	ImGui::TextWrapped("Make sure the controller bulb is not being obscured during the sampling process.");
			ImGui::TextWrapped("Otherwise use the manual color detection method instead.");

			if (ImGui::Button("Try Again"))
			{
				m_iDetectingExposure = k_color_autodetect_probe_step;
				setState(eMenuState::detection_exposure_adjust);
			}

			ImGui::SameLine();

			if (ImGui::Button("Go Back"))
			{
				setState(eMenuState::manualConfig);
			}

			ImGui::End();
			break;
		}
		case eDetectionFailReason::failreason_canceled:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 100));
			ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

			ImGui::Text("Color sampling aborted!");

			if (ImGui::Button("Go Back"))
			{
				setState(eMenuState::manualConfig);
			}

			ImGui::End();
			break;
		}
		case eDetectionFailReason::failreason_unsupported_controller:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 150));
			ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

			ImGui::TextWrapped("Color sampling failed!");
			ImGui::TextWrapped("Unable to change color on virtual controllers!");
			ImGui::TextWrapped("Please use the manual color detection instead.");

			if (ImGui::Button("Go Back"))
			{
				setState(eMenuState::manualConfig);
			}

			ImGui::End();
			break;
		}
		case eDetectionFailReason::failreason_unsupported_tracker:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 200));
			ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

			ImGui::TextWrapped("Color sampling failed!");
			ImGui::TextWrapped("Unable to find controller #%d on tracker #%d!", m_masterControllerView->ControllerID, m_trackerView->tracker_info.tracker_id);
			ImGui::TextWrapped("Could not automatically adjust exposure/gain on virtual trackers!");
			ImGui::TextWrapped("Please adjust exposure/gain on this tracker manually and try again.");
			ImGui::TextWrapped("Otherwise use the manual color detection method instead.");
		
			if (ImGui::Button("Try Again"))
			{
				m_iDetectingExposure = k_color_autodetect_probe_step;
				setState(eMenuState::detection_exposure_adjust);
			}

			ImGui::SameLine();

			if (ImGui::Button("Go Back"))
			{
				setState(eMenuState::manualConfig);
			}

			ImGui::End();
			break;
		}
		default:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 150));
			ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

			ImGui::TextWrapped("Color sampling on controller #%d and tracker #%d failed!", m_masterControllerView->ControllerID, m_trackerView->tracker_info.tracker_id);

			if (ImGui::Button("Try Again"))
			{
				m_iDetectingExposure = k_color_autodetect_probe_step;
				setState(eMenuState::detection_exposure_adjust);
			}

			ImGui::SameLine();

			if (ImGui::Button("Go Back"))
			{
				setState(eMenuState::manualConfig);
			}

			ImGui::End();
			break;
		}
		}

		
		break;
	}
	case eMenuState::detection_finish:
	{
		m_bDetectingColors = false;

		ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(600, 100));
		ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

		ImGui::TextWrapped("Color sampling finished.");
		ImGui::TextWrapped("Controller colors and tracker exposure/gain have been automatically adjusted!");

		if (ImGui::Button(" OK "))
		{
			setState(eMenuState::manualConfig);
		}

		ImGui::End();
		break;
	}



    case eMenuState::autoConfig_wait1:
	{
		m_lastDetection = std::chrono::high_resolution_clock::now();
		setState(eMenuState::autoConfig_wait2);
		break;
	}
    case eMenuState::autoConfig_wait2:
	{
		// $TODO: Wait for frame ready instead of waiting
		std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> timeSinceLast = now - m_lastDetection;
		if (timeSinceLast.count() > k_auto_calib_sleep)
		{
			setState(eMenuState::autoConfig);
		}
		break;
	}
    case eMenuState::changeController:
		if (m_bDetectingColors)
		{
			setState(eMenuState::detection_exposure_adjust);
		}
		else
		{
			setState(eMenuState::manualConfig);
		}
		request_change_controller(1);

		break;
    case eMenuState::changeTracker:
		if (m_bDetectingColors)
		{
			setState(eMenuState::detection_exposure_adjust);
		}
		else
		{
			setState(eMenuState::manualConfig);
		}

        request_change_tracker(1);
        break;
    case eMenuState::pendingTrackerStartStreamRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::waitingForStreamStartResponse:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

        ImGui::Text("Waiting for device stream to start...");

        ImGui::End();
    } break;

    case eMenuState::failedTrackerStartStreamRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedControllerStartRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoCollapse);

        if (m_menuState == eMenuState::failedTrackerStartStreamRequest)
        {
            ImGui::Text("Failed to start tracker stream!");
        }
        else if (m_menuState == eMenuState::failedHmdStartRequest)
        {
            ImGui::Text("Failed to start controller stream!");
        }
        else
        {
            ImGui::Text("Failed to start controller stream!");
        }

        if (ImGui::Button(" OK "))
        {
            request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ColorCalibration::setState(
    AppStage_ColorCalibration::eMenuState newState)
{
    if (newState != m_menuState)
    {
		m_menuState = newState;
    }
}

void AppStage_ColorCalibration::request_start_controller_streams()
{
    for (PSMController *controllerView : m_controllerViews)
    {
        ++m_pendingControllerStartCount;

        PSMRequestID request_id;
        PSM_StartControllerDataStreamAsync(controllerView->ControllerID, PSMStreamFlags_defaultStreamOptions | PSMStreamFlags_includeRawSensorData | PSMStreamFlags_includeCalibratedSensorData, &request_id);
        PSM_RegisterCallback(request_id, &AppStage_ColorCalibration::handle_start_controller_response, this);
    }

    // Start receiving data from the controller
    setState(AppStage_ColorCalibration::pendingControllerStartRequest);
}

void AppStage_ColorCalibration::handle_start_controller_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
//    const ClientPSMoveAPI::t_request_id request_id = response_message->request_id;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            --thisPtr->m_pendingControllerStartCount;

            if (thisPtr->m_pendingControllerStartCount <= 0)
            {
                thisPtr->m_areAllControllerStreamsActive= true;
                thisPtr->setState(AppStage_ColorCalibration::waitingForStreamStartResponse);
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ColorCalibration::failedControllerStartRequest);
        } break;
    }
}

void AppStage_ColorCalibration::request_set_controller_tracking_color(
    PSMController *controllerView,
    PSMTrackingColorType tracking_color)
{
    unsigned char r, g, b;

    switch (tracking_color)
    {
    case PSMTrackingColorType::PSMTrackingColorType_Magenta:
        r = 0xFF; g = 0x00; b = 0xFF;
        break;
    case PSMTrackingColorType::PSMTrackingColorType_Cyan:
        r = 0x00; g = 0xFF; b = 0xFF;
        break;
    case PSMTrackingColorType::PSMTrackingColorType_Yellow:
        r = 0xFF; g = 0xFF; b = 0x00;
        break;
    case PSMTrackingColorType::PSMTrackingColorType_Red:
        r = 0xFF; g = 0x00; b = 0x00;
        break;
    case PSMTrackingColorType::PSMTrackingColorType_Green:
        r = 0x00; g = 0xFF; b = 0x00;
        break;
    case PSMTrackingColorType::PSMTrackingColorType_Blue:
        r = 0x00; g = 0x00; b = 0xFF;
        break;
    default:
		r = 0x00; g = 0x00; b = 0x00;
		break;
    }

    PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, r, g, b);
}

void AppStage_ColorCalibration::request_start_hmd_stream()
{
    // Start receiving data from the controller
    setState(AppStage_ColorCalibration::pendingHmdStartRequest);

    PSMRequestID requestId;
    PSM_StartHmdDataStreamAsync(m_hmdView->HmdID, PSMStreamFlags_defaultStreamOptions | PSMStreamFlags_includeRawSensorData | PSMStreamFlags_includeCalibratedSensorData, &requestId);
    PSM_RegisterCallback(requestId, AppStage_ColorCalibration::handle_start_hmd_response, this);
}

void AppStage_ColorCalibration::handle_start_hmd_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);
    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            thisPtr->m_isHmdStreamActive = true;
            thisPtr->setState(AppStage_ColorCalibration::waitingForStreamStartResponse);
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ColorCalibration::failedControllerStartRequest);
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_start_stream()
{
    if (m_menuState != AppStage_ColorCalibration::pendingTrackerStartStreamRequest)
    {
        setState(AppStage_ColorCalibration::pendingTrackerStartStreamRequest);

        // Tell the psmove service that we want to start streaming data from the tracker
        PSMRequestID requestID;
        PSM_StartTrackerDataStreamAsync(m_trackerView->tracker_info.tracker_id, &requestID);
        PSM_RegisterCallback(requestID, AppStage_ColorCalibration::handle_tracker_start_stream_response, this);
    }
}

void AppStage_ColorCalibration::handle_tracker_start_stream_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            PSMTracker *trackerView = thisPtr->m_trackerView;

            // Open the shared memory that the video stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSMResult_Success)
            {
                thisPtr->allocate_video_buffers();
            }

            // Now that the tracker stream is started, start the controller stream
            if (thisPtr->m_hmdView != nullptr)
            {
                thisPtr->request_start_hmd_stream();
            }
            else
            {
                thisPtr->request_start_controller_streams();
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ColorCalibration::failedTrackerStartStreamRequest);
        } break;
    }
}

void AppStage_ColorCalibration::allocate_video_buffers()
{
    m_video_buffer_state = new VideoBufferState(m_trackerView);
}

void AppStage_ColorCalibration::release_video_buffers()
{
    delete m_video_buffer_state;
    m_video_buffer_state = nullptr;
}

void AppStage_ColorCalibration::request_tracker_set_frame_width(double value)
{
	// Tell the psmove service that we want to change frame width.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_FRAME_WIDTH);
	request->mutable_request_set_tracker_frame_width()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
	request->mutable_request_set_tracker_frame_width()->set_value(static_cast<float>(value));
	request->mutable_request_set_tracker_frame_width()->set_save_setting(true);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_frame_width_response, this);

	// Exit and re-enter Color Calibration
	m_app->getAppStage<AppStage_TrackerSettings>()->gotoControllerColorCalib(true);

	request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
}

void AppStage_ColorCalibration::handle_tracker_set_frame_width_response(
	const PSMResponseMessage *response,
	void *userdata)
{
	PSMResult ResultCode = response->result_code;
	PSMResponseHandle response_handle = response->opaque_response_handle;
	AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
		thisPtr->m_trackerFrameWidth = response->result_set_tracker_frame_width().new_frame_width();
	} break;
	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		//###HipsterSloth $TODO - Replace with C_API style log
		//CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker frame width!";
	} break;
	}
}

void AppStage_ColorCalibration::request_tracker_set_frame_rate(double value)
{
    // Tell the psmove service that we want to change frame rate.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_FRAME_RATE);
    request->mutable_request_set_tracker_frame_rate()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_frame_rate()->set_value(static_cast<float>(value));
    request->mutable_request_set_tracker_frame_rate()->set_save_setting(true);

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_frame_rate_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_frame_rate_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerFrameRate = response->result_set_tracker_frame_rate().new_frame_rate();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker frame rate!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_exposure(double value)
{
    // Tell the psmove service that we want to change exposure.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_EXPOSURE);
    request->mutable_request_set_tracker_exposure()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_exposure()->set_value(static_cast<float>(value));
    request->mutable_request_set_tracker_exposure()->set_save_setting(true);

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_exposure_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_exposure_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerExposure = response->result_set_tracker_exposure().new_exposure();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker exposure!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_gain(double value)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_GAIN);
    request->mutable_request_set_tracker_gain()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_gain()->set_value(static_cast<float>(value));
    request->mutable_request_set_tracker_gain()->set_save_setting(true);

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_gain_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_gain_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerGain = response->result_set_tracker_gain().new_gain();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker gain!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_option(
    TrackerOption &option, 
    int new_option_index)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_OPTION);
    request->mutable_request_set_tracker_option()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_set_tracker_option()->set_option_name(option.option_name);
    request->mutable_request_set_tracker_option()->set_option_index(new_option_index);

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_option_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_option_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            int result_option_index = response->result_set_tracker_option().new_option_index();
            std::string option_name = response->result_set_tracker_option().option_name();

            // Find the option with the matching option_name
            auto it = std::find_if(
                thisPtr->m_trackerOptions.begin(),
                thisPtr->m_trackerOptions.end(),
                [&option_name](const TrackerOption &option) { 
                    return option.option_name == option_name; 
                });

            if (it != thisPtr->m_trackerOptions.end())
            {
                it->option_index = result_option_index;
            }
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker gain!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_set_projectionblacklist(
	CommonDeviceBlacklistProjection projection_blacklisted[eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS])
{
	// Tell the psmove service that we want to change gain.
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_PROJECTIONBLACKLIST);
	request->mutable_request_set_tracker_projection_blacklist()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
	PSMoveProtocol::ProjectionBlacklistList *proj_blacklist = request->mutable_request_set_tracker_projection_blacklist()->mutable_projection_blacklist();

	for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
	{
		switch (i)
		{
		case 0: {
			proj_blacklist->mutable__1()->set_x(projection_blacklisted[i].x);
			proj_blacklist->mutable__1()->set_y(projection_blacklisted[i].y);
			proj_blacklist->mutable__1()->set_w(projection_blacklisted[i].w);
			proj_blacklist->mutable__1()->set_h(projection_blacklisted[i].h);
			break;
		}
		case 1: {
			proj_blacklist->mutable__2()->set_x(projection_blacklisted[i].x);
			proj_blacklist->mutable__2()->set_y(projection_blacklisted[i].y);
			proj_blacklist->mutable__2()->set_w(projection_blacklisted[i].w);
			proj_blacklist->mutable__2()->set_h(projection_blacklisted[i].h);
			break;
		}
		case 2: {
			proj_blacklist->mutable__3()->set_x(projection_blacklisted[i].x);
			proj_blacklist->mutable__3()->set_y(projection_blacklisted[i].y);
			proj_blacklist->mutable__3()->set_w(projection_blacklisted[i].w);
			proj_blacklist->mutable__3()->set_h(projection_blacklisted[i].h);
			break;
		}
		case 3: {
			proj_blacklist->mutable__4()->set_x(projection_blacklisted[i].x);
			proj_blacklist->mutable__4()->set_y(projection_blacklisted[i].y);
			proj_blacklist->mutable__4()->set_w(projection_blacklisted[i].w);
			proj_blacklist->mutable__4()->set_h(projection_blacklisted[i].h);
			break;
		}
		case 4: {
			proj_blacklist->mutable__5()->set_x(projection_blacklisted[i].x);
			proj_blacklist->mutable__5()->set_y(projection_blacklisted[i].y);
			proj_blacklist->mutable__5()->set_w(projection_blacklisted[i].w);
			proj_blacklist->mutable__5()->set_h(projection_blacklisted[i].h);
			break;
		}
		}
	}

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ColorCalibration::request_tracker_set_color_preset(
    PSMTrackingColorType color_type,
    TrackerColorPreset &color_preset)
{
    // Tell the psmove service that we want to change gain.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_COLOR_PRESET);
    request->mutable_request_set_tracker_color_preset()->set_tracker_id(m_trackerView->tracker_info.tracker_id);

    if (m_hmdView != nullptr)
    {
        request->mutable_request_set_tracker_color_preset()->set_device_id(m_overrideHmdId);
        request->mutable_request_set_tracker_color_preset()->set_device_category(
            PSMoveProtocol::Request_RequestSetTrackerColorPreset_DeviceCategory_HMD);
    }
    else
    {
        request->mutable_request_set_tracker_color_preset()->set_device_id(m_overrideControllerId);
        request->mutable_request_set_tracker_color_preset()->set_device_category(
            PSMoveProtocol::Request_RequestSetTrackerColorPreset_DeviceCategory_CONTROLLER);
    }

    {
        PSMoveProtocol::TrackingColorPreset* tracking_color_preset =
            request->mutable_request_set_tracker_color_preset()->mutable_color_preset();

        tracking_color_preset->set_color_type(static_cast<PSMoveProtocol::TrackingColorType>(color_type));
        tracking_color_preset->set_hue_center(color_preset.hue_center);
        tracking_color_preset->set_hue_range(color_preset.hue_range);
        tracking_color_preset->set_saturation_center(color_preset.saturation_center);
        tracking_color_preset->set_saturation_range(color_preset.saturation_range);
        tracking_color_preset->set_value_center(color_preset.value_center);
        tracking_color_preset->set_value_range(color_preset.value_range);
    }

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_set_color_preset_response, this);
}

void AppStage_ColorCalibration::handle_tracker_set_color_preset_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    switch (response->result_code)
    {
    case PSMResult_Success:
        {
            const PSMResponseHandle response_handle = response->opaque_response_handle;
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            const PSMoveProtocol::TrackingColorPreset &srcPreset= response->result_set_tracker_color_preset().new_color_preset();
            const PSMTrackingColorType color_type = static_cast<PSMTrackingColorType>(srcPreset.color_type());

            AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);
            AppStage_ColorCalibration::TrackerColorPreset &targetPreset= thisPtr->m_colorPresets[color_type];

            targetPreset.hue_center= srcPreset.hue_center();
            targetPreset.hue_range= srcPreset.hue_range();
            targetPreset.saturation_center= srcPreset.saturation_center();
            targetPreset.saturation_range= srcPreset.saturation_range();
            targetPreset.value_center= srcPreset.value_center();
            targetPreset.value_range= srcPreset.value_range();
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to set the tracker presets!";
        } break;
    }
}

void AppStage_ColorCalibration::request_tracker_get_settings()
{
    // Tell the psmove service that we want to change exposure.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_GET_TRACKER_SETTINGS);
    request->mutable_request_get_tracker_settings()->set_tracker_id(m_trackerView->tracker_info.tracker_id);

    if (m_overrideHmdId != -1)
    {
        request->mutable_request_get_tracker_settings()->set_device_id(m_overrideHmdId);
        request->mutable_request_get_tracker_settings()->set_device_category(PSMoveProtocol::Request_RequestGetTrackerSettings_DeviceCategory_HMD);
    }
    else
    {
        request->mutable_request_get_tracker_settings()->set_device_id(m_overrideControllerId);
        request->mutable_request_get_tracker_settings()->set_device_category(PSMoveProtocol::Request_RequestGetTrackerSettings_DeviceCategory_CONTROLLER);
    }

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_get_settings_response, this);
}

void AppStage_ColorCalibration::handle_tracker_get_settings_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_ColorCalibration *thisPtr = static_cast<AppStage_ColorCalibration *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            thisPtr->m_trackerFrameWidth = response->result_tracker_settings().frame_width();
            thisPtr->m_trackerFrameRate = response->result_tracker_settings().frame_rate();
            thisPtr->m_trackerExposure = response->result_tracker_settings().exposure();
            thisPtr->m_trackerGain = response->result_tracker_settings().gain();
			
            thisPtr->m_trackerOptions.clear();
            for (auto it = response->result_tracker_settings().option_sets().begin();
                it != response->result_tracker_settings().option_sets().end();
                ++it)
            {
                const PSMoveProtocol::OptionSet &srcOption = *it;
                AppStage_ColorCalibration::TrackerOption destOption;

                destOption.option_index = srcOption.option_index();
                destOption.option_name = srcOption.option_name();
                
                // Copy the option strings into the destOption
                std::for_each(
                    srcOption.option_strings().begin(), 
                    srcOption.option_strings().end(), 
                    [&destOption](const std::string &option_string) { 
                        destOption.option_strings.push_back(option_string); 
                    });

                thisPtr->m_trackerOptions.push_back(destOption);
            }

            for (auto it = response->result_tracker_settings().color_presets().begin();
                it != response->result_tracker_settings().color_presets().end();
                ++it)
            {
                const PSMoveProtocol::TrackingColorPreset &srcPreset = *it;
                const PSMTrackingColorType client_color= 
                    static_cast<PSMTrackingColorType>(srcPreset.color_type());

                AppStage_ColorCalibration::TrackerColorPreset &destPreset = thisPtr->m_colorPresets[client_color];
                destPreset.hue_center= srcPreset.hue_center();
                destPreset.hue_range= srcPreset.hue_range();
                destPreset.saturation_center = srcPreset.saturation_center();
                destPreset.saturation_range = srcPreset.saturation_range();
                destPreset.value_center = srcPreset.value_center();
                destPreset.value_range = srcPreset.value_range();
            }

			for (int i = 0; i < eCommonBlacklistProjection::MAX_BLACKLIST_PROJECTIONS; ++i)
			{
				const PSMoveProtocol::ProjectionBlacklistList proj_blacklist = response->result_tracker_settings().projection_blacklist();

				switch (i)
				{
				case 0: {
					thisPtr->m_blacklisted_projection[i].x = proj_blacklist._1().x();
					thisPtr->m_blacklisted_projection[i].y = proj_blacklist._1().y();
					thisPtr->m_blacklisted_projection[i].w = proj_blacklist._1().w();
					thisPtr->m_blacklisted_projection[i].h = proj_blacklist._1().h();
					break;
				}
				case 1: {
					thisPtr->m_blacklisted_projection[i].x = proj_blacklist._2().x();
					thisPtr->m_blacklisted_projection[i].y = proj_blacklist._2().y();
					thisPtr->m_blacklisted_projection[i].w = proj_blacklist._2().w();
					thisPtr->m_blacklisted_projection[i].h = proj_blacklist._2().h();
					break;
				}
				case 2: {
					thisPtr->m_blacklisted_projection[i].x = proj_blacklist._3().x();
					thisPtr->m_blacklisted_projection[i].y = proj_blacklist._3().y();
					thisPtr->m_blacklisted_projection[i].w = proj_blacklist._3().w();
					thisPtr->m_blacklisted_projection[i].h = proj_blacklist._3().h();
					break;
				}
				case 3: {
					thisPtr->m_blacklisted_projection[i].x = proj_blacklist._4().x();
					thisPtr->m_blacklisted_projection[i].y = proj_blacklist._4().y();
					thisPtr->m_blacklisted_projection[i].w = proj_blacklist._4().w();
					thisPtr->m_blacklisted_projection[i].h = proj_blacklist._4().h();
					break;
				}
				case 4: {
					thisPtr->m_blacklisted_projection[i].x = proj_blacklist._5().x();
					thisPtr->m_blacklisted_projection[i].y = proj_blacklist._5().y();
					thisPtr->m_blacklisted_projection[i].w = proj_blacklist._5().w();
					thisPtr->m_blacklisted_projection[i].h = proj_blacklist._5().h();
					break;
				}
				}
			}
			
        } break;
    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            //###HipsterSloth $TODO - Replace with C_API style log
            //CLIENT_LOG_INFO("AppStage_ColorCalibration") << "Failed to get the tracker settings!";
        } break;
    }
}

void AppStage_ColorCalibration::request_save_default_tracker_profile()
{
    // Tell the psmove service that we want to save the current trackers profile.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SAVE_TRACKER_PROFILE);
    request->mutable_request_save_tracker_profile()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_save_tracker_profile()->set_controller_id(m_overrideControllerId);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ColorCalibration::request_apply_default_tracker_profile()
{
    // Tell the psmove service that we want to apply the saved default profile to the current tracker.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_APPLY_TRACKER_PROFILE);
    request->mutable_request_save_tracker_profile()->set_tracker_id(m_trackerView->tracker_info.tracker_id);
    request->mutable_request_save_tracker_profile()->set_controller_id(m_overrideControllerId);

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_ColorCalibration::handle_tracker_get_settings_response, this);
}

void AppStage_ColorCalibration::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

    release_video_buffers();

    for (PSMController *controllerView : m_controllerViews)
    {
        PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);

        if (m_areAllControllerStreamsActive)
        {
            PSMRequestID request_id;
            PSM_StopControllerDataStreamAsync(controllerView->ControllerID, &request_id);
            PSM_EatResponse(request_id);
        }

        PSM_FreeControllerListener(controllerView->ControllerID);
    }
    m_controllerViews.clear();

    m_masterControllerView = nullptr;
    m_areAllControllerStreamsActive= false;
    m_lastMasterControllerSeqNum= -1;


    if (m_hmdView != nullptr)
    {
        if (m_isHmdStreamActive)
        {
            PSMRequestID request_id;
            PSM_StopHmdDataStreamAsync(m_hmdView->HmdID, &request_id);
            PSM_EatResponse(request_id);
        }

        PSM_FreeHmdListener(m_hmdView->HmdID);
        m_hmdView = nullptr;
        m_isHmdStreamActive = false;
        m_lastHmdSeqNum = -1;
    }

    if (m_trackerView != nullptr)
    {
        PSM_CloseTrackerVideoStream(m_trackerView->tracker_info.tracker_id);

        PSMRequestID request_id;
        PSM_StopTrackerDataStreamAsync(m_trackerView->tracker_info.tracker_id, &request_id);
        PSM_EatResponse(request_id);

        PSM_FreeTrackerListener(m_trackerView->tracker_info.tracker_id);
        m_trackerView = nullptr;
    }
}

void AppStage_ColorCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_ColorCalibration::request_turn_on_all_tracking_bulbs(bool bEnabled)
{
    assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());
    for (int list_index= 0; list_index < m_controllerViews.size(); ++list_index)
    {
        PSMController *controllerView= m_controllerViews[list_index];

        if (controllerView == m_masterControllerView)
            continue;

        if (bEnabled)
        {
            request_set_controller_tracking_color(controllerView, m_controllerTrackingColorTypes[list_index]);
        }
        else
        {
            PSM_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);
        }
    }
}

void AppStage_ColorCalibration::request_change_controller(int step)
{
    assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());

    {
        PSMController *controllerView = m_controllerViews[m_overrideControllerId];

        if (controllerView == m_masterControllerView) {
            PSM_SetControllerLEDOverrideColor(m_masterControllerView->ControllerID, 0, 0, 0);
            if (m_overrideControllerId + step < static_cast<int>(m_controllerViews.size()) && m_overrideControllerId + step >= 0) {
                m_overrideControllerId = m_overrideControllerId + step;
                m_masterControllerView = m_controllerViews[m_overrideControllerId];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            }
            else if (step > 0) {
                m_overrideControllerId = 0;
                m_masterControllerView = m_controllerViews[0];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);

                if (m_bAutoChangeTracker)
					setState(eMenuState::changeTracker);
            }
            else {
                m_overrideControllerId = static_cast<int>(m_controllerViews.size()) -1;
                m_masterControllerView = m_controllerViews[m_overrideControllerId];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            }
        }
    }
    m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedControllerIndex(m_overrideControllerId);
}

void AppStage_ColorCalibration::request_change_tracker(int step)
{
	m_app->getAppStage<AppStage_ColorCalibration>()->
		set_autoConfig(m_bAutoChangeColor, m_bAutoChangeController, m_bAutoChangeTracker);

	m_app->getAppStage<AppStage_ColorCalibration>()->
		set_autoDetection(m_bDetectingColors, m_iDetectingControllersLeft, m_iDetectingExposure, m_iDetectingAdjustMethod);

    if (tracker_index + step < tracker_count && tracker_index + step >= 0)
    {
        m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(tracker_index + step);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
    else if (step > 0)
    {
        m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(0);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
    else
    {
        m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(tracker_count -1);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
}

void AppStage_ColorCalibration::auto_adjust_color_sensitivity(TrackerColorPreset &preset, bool isPSmoveDevice)
{
	// Additional color details
	// MAGENTA requires some HEU_RANGE adjustments to track propperly otherwise it becomes too blocky. None of the other colors seems to be near it at all anyways?
	// GREEN is very VALUE_CENTER sensitive compared to other colors since camera chips have more green sensors. Likes to collide with CYAN if VALUE_RANGE is higher.
	// CYAN is one of the worest colors too track. HEU_RANGE between BLUE and GREEN is very dense too CYAN and collides alot.
	// RED can collide with YELLOW edges on higher VALUE_RANGE

	//###Externet $TODO Probably make it more automated. This is too hardcoded.

	float hueRangeMulti = 1.0;
	float saturationRangeMulti = 1.0f;

	// Only fine tune known psmoves and pseyes because it could cause issues with non-psmove hardware
	if (isPSmoveDevice)
	{
		switch (m_masterTrackingColorType)
		{
		case PSMTrackingColorType_Magenta:
		{
			if (m_bColorCollisionPrevent)
			{
				hueRangeMulti = 1.0f;
			}
			else
			{
				hueRangeMulti = 1.5f; // Improve color detection. No other color near it?!
			}
			break;
		}
		case PSMTrackingColorType_Cyan:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Blue/Green collsion prevention
			break;
		}
		case PSMTrackingColorType_Yellow:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Red collision prevention

			saturationRangeMulti = 2.0f;
			break;
		}
		case PSMTrackingColorType_Red:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Yellow collision prevention
			break;
		}
		case PSMTrackingColorType_Green:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Cyan collision prevention
			break;
		}
		case PSMTrackingColorType_Blue:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Cyan collision prevention
			break;
		}
		default:
		{
			// Custom color
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f;
			break;
		}
		}
	}
	else
	{
		switch (m_masterTrackingColorType)
		{
		case PSMTrackingColorType_Magenta:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f;
			break;
		}
		case PSMTrackingColorType_Cyan:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Blue/Green collsion prevention
			break;
		}
		case PSMTrackingColorType_Yellow:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Red collision prevention
			break;
		}
		case PSMTrackingColorType_Red:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Yellow collision prevention
			break;
		}
		case PSMTrackingColorType_Green:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Cyan collision prevention
			break;
		}
		case PSMTrackingColorType_Blue:
		{
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f; // Cyan collision prevention
			break;
		}
		default:
		{
			// Custom color
			if (m_bColorCollisionPrevent)
				hueRangeMulti = 0.5f;
			break;
		}
		}
	}

	switch (m_iColorSensitivity)
	{
	case sensitivity_normal:
	{
		preset.hue_range = 10.f * hueRangeMulti;
		preset.saturation_range = (32.f * saturationRangeMulti);
		preset.value_range = 32.f;
		break;
	}
	case sensitivity_high:
	{
		preset.hue_range = 10.f * hueRangeMulti;
		preset.saturation_range = (32.f * saturationRangeMulti) + (16.f / saturationRangeMulti);
		preset.value_range = 32.f + 16.f;
		break;
	}
	case sensitivity_aggressive:
	{
		preset.hue_range = 10.f * hueRangeMulti;
		preset.saturation_range = (32.f * saturationRangeMulti) + (32.f / saturationRangeMulti);
		preset.value_range = 32.f + 32.f;

		break;
	}
	case sensitivity_extreme:
	{
		preset.hue_range = 10.f * hueRangeMulti;
		preset.saturation_range = (32.f * saturationRangeMulti) + (48.f / saturationRangeMulti);
		preset.value_range = 32.f + 48.f;
		break;
	}
	}

	preset.saturation_center = std::max(preset.saturation_center, 0.f);
	preset.value_center = std::max(preset.value_center, 0.f);
}

void AppStage_ColorCalibration::get_contures_lower(int type, int min_points_in_contour, std::vector<std::vector<int>> &contures)
{
	if (min_points_in_contour < 1)
		min_points_in_contour = 1;

	if (m_trackerView == nullptr || m_video_buffer_state == nullptr)
	{
		return;
	}

	const int frameWidth = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.x);
	const int frameHeight = static_cast<int>(m_trackerView->tracker_info.tracker_screen_dimensions.y);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Point2f> biggest_contour_f;

	if (type == 0)
	{
		cv::findContours(*m_video_buffer_state->gsLowerBuffer,
			contours,
			CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE);  // CV_CHAIN_APPROX_NONE?
	}
	else
	{
		cv::findContours(*m_video_buffer_state->detectionLowerBuffer,
			contours,
			CV_RETR_EXTERNAL,
			CV_CHAIN_APPROX_SIMPLE);  // CV_CHAIN_APPROX_NONE?
	}

	struct ContourInfo
	{
		int contour_index;
		double contour_area;
	};
	std::vector<ContourInfo> sorted_contour_list;

	// Compute the area of each contour
	int contour_index = 0;
	for (auto it = contours.begin(); it != contours.end(); ++it)
	{
		const double contour_area = cv::contourArea(*it);
		const ContourInfo contour_info = { contour_index, contour_area };

		sorted_contour_list.push_back(contour_info);
		++contour_index;
	}


	// Sort the list of contours by area, largest to smallest
	if (sorted_contour_list.size() > 1)
	{
		std::sort(
			sorted_contour_list.begin(), sorted_contour_list.end(),
			[](const ContourInfo &a, const ContourInfo &b) {
			return b.contour_area < a.contour_area;
		});
	}

	const int max_contour_count = 32;

	std::vector<std::vector<cv::Point>> out_biggest_N_contours;

	// Copy up to N valid contours
	for (auto it = sorted_contour_list.begin();
		it != sorted_contour_list.end() && static_cast<int>(out_biggest_N_contours.size()) < max_contour_count;
		++it)
	{
		const ContourInfo &contour_info = *it;
		std::vector<cv::Point> &contour = contours[contour_info.contour_index];

		if (contour.size() >= min_points_in_contour)
		{
			// Remove any points in contour on edge of camera/ROI
			// TODO: Contours touching image border will be clipped,
			// so this might not be necessary.
			std::vector<cv::Point>::iterator it = contour.begin();
			while (it != contour.end())
			{
				if (it->x == 0 || it->x == (frameWidth - 1) || it->y == 0 || it->y == (frameHeight - 1))
				{
					it = contour.erase(it);
				}
				else
				{
					++it;
				}
			}

			// Add cleaned up contour to the output list
			out_biggest_N_contours.push_back(contour);
		}
	}

	contures.clear();

	for (std::vector<cv::Point> contour : out_biggest_N_contours)
	{
		cv::Moments mu(cv::moments(contour));
		cv::Point2f massCenter;

		// mu.m00 is zero for contours of zero area.
		// Fallback to standard centroid in this case.

		if (!is_double_nearly_zero(mu.m00))
		{
			massCenter = cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
		}
		else
		{
			massCenter.x = 0.f;
			massCenter.y = 0.f;

			for (const cv::Point &int_point : contour)
			{
				massCenter.x += static_cast<float>(int_point.x);
				massCenter.y += static_cast<float>(int_point.y);
			}

			if (contour.size() > 1)
			{
				const float N = static_cast<float>(contour.size());

				massCenter.x /= N;
				massCenter.y /= N;
			}
		}

		std::vector<int> pos;
		pos.push_back(static_cast<int>(massCenter.x * (ImGui::GetIO().DisplaySize.x / static_cast<float>(frameWidth))));
		pos.push_back(static_cast<int>(massCenter.y * (ImGui::GetIO().DisplaySize.y / static_cast<float>(frameHeight))));
		contures.push_back(pos);
	}
}

// Virtual trackers have a common device path "VirtualTracker_#"
// ###Externet $TODO: Add better virtual tracker check. Probably should do that after changing protocols.
bool AppStage_ColorCalibration::is_tracker_virtual()
{
	if (m_trackerView != nullptr && m_trackerView->tracker_info.tracker_type == PSMTracker_PS3Eye)
	{
		return m_trackerView->tracker_info.device_path[0] == 'V';
	}

	return false;
}
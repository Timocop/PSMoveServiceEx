//-- inludes -----
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"
#include "AssetManager.h"

#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_HMDAccelerometerCalibration::APP_STAGE_NAME = "HMDAcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 3000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- definitions -----
struct HMDAccelerometerPoseSamples
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PSMVector3f raw_accelerometer_samples[k_max_accelerometer_samples];
    PSMVector3f raw_average_gravity;
	float raw_variance; // Max raw sensor variance (raw_sensor_units^2)
    int sample_count;

    void clear()
    {        
        sample_count= 0;
		raw_average_gravity = *k_psm_float_vector3_zero;
		raw_variance = 0.f;
    }

    void computeStatistics()
    {
		const float N = static_cast<float>(k_max_accelerometer_samples);

		// Compute both mean of signed and unsigned samples
		PSMVector3f mean_acc_abs_error = *k_psm_float_vector3_zero;
        raw_average_gravity = *k_psm_float_vector3_zero;
        for (int sample_index= 0; sample_index < k_max_accelerometer_samples; ++sample_index)
        {
			PSMVector3f signed_error_sample = raw_accelerometer_samples[sample_index];
			PSMVector3f unsigned_error_sample = PSM_Vector3fAbs(&signed_error_sample);

			mean_acc_abs_error = PSM_Vector3fAdd(&mean_acc_abs_error, &unsigned_error_sample);
            raw_average_gravity= PSM_Vector3fAdd(&raw_average_gravity, &signed_error_sample);
        }
		mean_acc_abs_error = PSM_Vector3fUnsafeScalarDivide(&mean_acc_abs_error, N);
        raw_average_gravity= PSM_Vector3fUnsafeScalarDivide(&raw_average_gravity, N);

		// Compute the variance of the (unsigned) sample error, where "error" = abs(accelerometer_sample)
		PSMVector3f var_accelerometer = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			PSMVector3f unsigned_error_sample = PSM_Vector3fAbs(&raw_accelerometer_samples[sample_index]);
			PSMVector3f diff_from_mean = PSM_Vector3fSubtract(&unsigned_error_sample, &mean_acc_abs_error);
			PSMVector3f diff_from_mean_sqrd= PSM_Vector3fSquare(&diff_from_mean);

			var_accelerometer = PSM_Vector3fAdd(&var_accelerometer, &diff_from_mean_sqrd);
		}
		var_accelerometer = PSM_Vector3fUnsafeScalarDivide(&var_accelerometer, N - 1);

		// Use the max variance of all three axes (should be close)
		raw_variance = PSM_Vector3fMaxValue(&var_accelerometer);
    }
};

//-- private methods -----
static void request_set_hmd_accelerometer_calibration(
    const int controller_id,
	const PSMVector3f &raw_bias,
    const float raw_variance);
static void drawHMD(PSMHeadMountedDisplay *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDAccelerometerCalibration::AppStage_HMDAccelerometerCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDAccelerometerCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_hmdView(nullptr)
    , m_isHMDStreamActive(false)
    , m_lastHMDSeqNum(-1)
	, m_playspaceYawOffset(0.f)
    , m_noiseSamples(new HMDAccelerometerPoseSamples)
{
}

AppStage_HMDAccelerometerCalibration::~AppStage_HMDAccelerometerCalibration()
{
	if (m_noiseSamples != nullptr)
	{
		delete m_noiseSamples;
		m_noiseSamples = nullptr;
	}
}

void AppStage_HMDAccelerometerCalibration::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const AppStage_HMDSettings::HMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
	m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);

    m_noiseSamples->clear();

    // Initialize the controller state
    assert(hmdInfo->HmdID != -1);
    assert(m_hmdView == nullptr);
	PSM_AllocateHmdListener(hmdInfo->HmdID);
	m_hmdView = PSM_GetHmd(hmdInfo->HmdID);

    m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastHMDSeqNum = -1;

    // Start streaming in controller data
    assert(!m_isHMDStreamActive);

	PSMRequestID requestId;
	PSM_StartHmdDataStreamAsync(
		m_hmdView->HmdID, 
		PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includeRawSensorData, 
		&requestId); // turns on tracking lights
	PSM_RegisterCallback(requestId, &AppStage_HMDAccelerometerCalibration::handle_acquire_hmd, this);

	request_playspace_info();
}

void AppStage_HMDAccelerometerCalibration::exit()
{
    assert(m_hmdView != nullptr);
    PSM_FreeHmdListener(m_hmdView->HmdID);
    m_hmdView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_HMDAccelerometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isHMDStreamActive && m_hmdView->OutputSequenceNum != m_lastHMDSeqNum)
    {
        switch(m_hmdView->HmdType)
        {
		case PSMHmd_Morpheus:
            {
				const PSMMorpheusRawSensorData &rawSensorData =
					m_hmdView->HmdState.MorpheusState.RawSensorData;
                const PSMMorpheusCalibratedSensorData &calibratedSensorData =
                    m_hmdView->HmdState.MorpheusState.CalibratedSensorData;

				m_lastRawAccelerometer = rawSensorData.Accelerometer;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        case PSMHmd_Virtual:
            {
                m_lastRawAccelerometer = {0, 0, 0};
                m_lastCalibratedAccelerometer = {0, 0, 0};
            } break;
        default:
            assert(0 && "unreachable");
        }

        m_lastHMDSeqNum = m_hmdView->OutputSequenceNum;
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
		{
		} break;
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    m_app->getOrbitCamera()->resetOrientation();
					m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
                    m_menuState = AppStage_HMDAccelerometerCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_HMDAccelerometerCalibration::placeHMD;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::placeHMD:
        {
        } break;
    case eCalibrationMenuState::measureNoise:
        {
            if (bControllerDataUpdatedThisFrame && m_noiseSamples->sample_count < k_max_accelerometer_samples)
            {
                // Store the new sample
                m_noiseSamples->raw_accelerometer_samples[m_noiseSamples->sample_count] = PSM_Vector3iCastToFloat(&m_lastRawAccelerometer);
                ++m_noiseSamples->sample_count;

                // See if we filled all of the samples for this pose
                if (m_noiseSamples->sample_count >= k_max_accelerometer_samples)
                {
                    // Compute the average gravity value in this pose.
                    // This assumes that the acceleration noise has a Gaussian distribution.
                    m_noiseSamples->computeStatistics();

                    // Tell the service what the new calibration constraints are
                    request_set_hmd_accelerometer_calibration(
                        m_hmdView->HmdID,
                        m_noiseSamples->raw_average_gravity,
						m_noiseSamples->raw_variance);

                    m_menuState = AppStage_HMDAccelerometerCalibration::measureComplete;
                }
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDAccelerometerCalibration::render()
{
    const float modelScale = 18.f;
    glm::mat4 hmdTransform;

    switch(m_hmdView->HmdType)
    {
    case PSMHmd_Morpheus:
    case PSMHmd_Virtual:
        hmdTransform = glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale));
        break;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::placeHMD:
        {
			const float sampleScale = 0.2f;
			glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller model in the pose we want the user place it in
            drawHMD(m_hmdView, hmdTransform);

			// Draw the current raw accelerometer direction
			{
				glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
				glm::vec3 m_end = psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&m_lastRawAccelerometer));

				drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
				drawTextAtWorldPosition(sampleTransform, m_end, "A");
			}
        } break;
    case eCalibrationMenuState::measureNoise:
    case eCalibrationMenuState::measureComplete:
        {
            const float sampleScale = 0.2f;
            glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller in the middle            
            drawHMD(m_hmdView, hmdTransform);

            // Draw the sample point cloud around the origin
            drawPointCloud(sampleTransform, glm::vec3(1.f, 1.f, 1.f), 
                reinterpret_cast<float *>(m_noiseSamples->raw_accelerometer_samples), 
                m_noiseSamples->sample_count);

            // Draw the current raw accelerometer direction
            {
                glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
                glm::vec3 m_end = psm_vector3f_to_glm_vec3(PSM_Vector3iCastToFloat(&m_lastRawAccelerometer));

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "A");
            }
        } break;
    case eCalibrationMenuState::test:
        {
            const float sensorScale = 200.f;
            glm::mat4 sensorTransform = glm::scale(glm::mat4(1.f), glm::vec3(sensorScale, sensorScale, sensorScale));

			drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)), 20.f);

            drawHMD(m_hmdView, hmdTransform);
            drawTransformedAxes(hmdTransform, 20.f);

            // Draw the current filtered accelerometer direction
            {
                const float accel_g = PSM_Vector3fLength(&m_lastCalibratedAccelerometer);
                glm::vec3 m_start = glm::vec3(0.f);
                glm::vec3 m_end = psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(sensorTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sensorTransform, m_end, "A(%.1fg)", accel_g);
            }
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDAccelerometerCalibration::renderUI()
{
	static float waitCount;

    const float k_panel_width = 500;
    const char *k_window_title = "HMD Accelerometer Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::Begin(k_window_title, nullptr, window_flags);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
			ImGui::Text("Waiting for server response...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
			ImGui::End();
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
            ImGui::Text("Waiting for hmd stream to start...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed to start hmd stream!");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::placeHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			switch(m_hmdView->HmdType)
			{
			case PSMHmd_Morpheus:
            case PSMHmd_Virtual:
				ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
				ImGui::TextWrapped(
					"Place the HMD on a flat, level surface. Do not disturb the HMD while sampling is in progress!");
				break;
			}

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "Start Sampling"))
            {
                m_menuState = eCalibrationMenuState::measureNoise;
            }
            
			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureNoise:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            float sampleFraction =
                static_cast<float>(m_noiseSamples->sample_count)
                / static_cast<float>(k_max_accelerometer_samples);

			waitCount += 0.025f;
			switch ((int)floorf(waitCount))
			{
			case 0:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate()->getImTextureId(), ImVec2(32, 32));
				break;
			case 1:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				break;
			default:
				ImGui::Image(AssetManager::getInstance()->getIconUpdate2()->getImTextureId(), ImVec2(32, 32));
				waitCount = 0;
				break;
			}

			ImGui::SameLine();
            ImGui::Text(
				"Sampling accelerometer...\n"
				"Please do not touch the HMD and keep it steady!"
			);
			ImGui::Separator();
            ImGui::ProgressBar(sampleFraction, ImVec2(-1, 0));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconCheck()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::TextWrapped(
                "Sampling complete!\n"
                "Press OK to continue or Redo to resample.");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }
            
			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconUpdate(), "Redo"))
            {
                // Reset the sample info for the current pose
                m_noiseSamples->clear();
                m_menuState = eCalibrationMenuState::placeHMD;
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin("Test Accelerometer", nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of HMD ID #%d", m_hmdView->HmdID);
            }
            else
            {
				ImGui::Image(AssetManager::getInstance()->getIconCheck()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
                ImGui::Text("Calibration of HMD ID #%d complete!", m_hmdView->HmdID);
            }

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconLeft(), "Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
static void request_set_hmd_accelerometer_calibration(
    const int controller_id,
	const PSMVector3f &raw_average_gravity,
    const float variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_ACCELEROMETER_CALIBRATION);

    PSMoveProtocol::Request_RequestSetHMDAccelerometerCalibration *calibration =
        request->mutable_set_hmd_accelerometer_calibration_request();

    calibration->set_hmd_id(controller_id);
	calibration->mutable_raw_average_gravity()->set_i(raw_average_gravity.x);
	calibration->mutable_raw_average_gravity()->set_j(raw_average_gravity.y);
	calibration->mutable_raw_average_gravity()->set_k(raw_average_gravity.z);
    calibration->set_raw_variance(variance);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDAccelerometerCalibration::request_playspace_info()
{
	if (m_menuState != AppStage_HMDAccelerometerCalibration::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_HMDAccelerometerCalibration::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_HMDAccelerometerCalibration::handle_playspace_info_response, this);
	}
}

void AppStage_HMDAccelerometerCalibration::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDAccelerometerCalibration *thisPtr = static_cast<AppStage_HMDAccelerometerCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_playspaceYawOffset = response->result_get_playspace_offsets().playspace_orientation_yaw();

		thisPtr->m_menuState = AppStage_HMDAccelerometerCalibration::waitingForStreamStartResponse;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_HMDAccelerometerCalibration::failedStreamStart;
	} break;
	}
}

void AppStage_HMDAccelerometerCalibration::handle_acquire_hmd(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_HMDAccelerometerCalibration *thisPtr = reinterpret_cast<AppStage_HMDAccelerometerCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isHMDStreamActive = true;
        thisPtr->m_lastHMDSeqNum = -1;
        // Wait for the first HMD packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_HMDAccelerometerCalibration::failedStreamStart;
    }
}

void AppStage_HMDAccelerometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
	if (m_isHMDStreamActive)
	{
		PSMRequestID request_id;
		PSM_StopHmdDataStreamAsync(m_hmdView->HmdID, &request_id);
		PSM_EatResponse(request_id);

		m_isHMDStreamActive = false;
		m_app->setAppStage(app_stage_name);
	}
	else
	{
		m_app->setAppStage(app_stage_name);
	}
}

//-- private methods -----
static void drawHMD(PSMHeadMountedDisplay *hmdView, const glm::mat4 &transform)
{
    switch(hmdView->HmdType)
    {
    case PSMHmd_Morpheus:
        drawMorpheusModel(transform, true, false, glm::vec3(1.f, 1.f, 1.f));
        break;
    case PSMHmd_Virtual:
        drawVirtualHMDModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

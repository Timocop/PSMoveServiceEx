//-- inludes -----
#include "AppStage_HMDGyroscopeCalibration.h"
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
const char *AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME = "HMDGyroscopeCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 3000.f;
const int k_desired_noise_sample_count = 1000;

//-- definitions -----
struct GyroscopeNoiseSamples
{
	PSMVector3i raw_gyro_samples[k_desired_noise_sample_count];
	PSMVector3f raw_omega_samples[k_desired_noise_sample_count];
	PSMVector3f drift_rotation;
	std::chrono::time_point<std::chrono::high_resolution_clock> sampleStartTime;
	int sample_count;

	PSMVector3f raw_bias; // The average bias in the raw gyro measurement per frame
	float angular_drift_variance; // Max sensor variance (raw_sensor_units/s/s for DS4, rad/s/s for PSMove)
	float angular_drift_rate; // Max drift rate (raw_sensor_units/s for DS4, rad/s for PSMove)

	void clear()
	{
		drift_rotation = *k_psm_float_vector3_zero;
		sample_count = 0;
		raw_bias = *k_psm_float_vector3_zero;
		angular_drift_variance = 0.f;
		angular_drift_rate = 0.f;
	}

	void computeStatistics(std::chrono::duration<float, std::milli> sampleDurationMilli)
	{
		const float sampleDurationSeconds = sampleDurationMilli.count() / 1000.f;
		const float N = static_cast<float>(sample_count);

		// Compute the mean of the raw gyro measurements
		// If this is a non-zero value then the gyro has some per frame
		// bias we need to subtract off.
		raw_bias = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			PSMVector3f raw_sample = PSM_Vector3iCastToFloat(&raw_gyro_samples[sample_index]);

			raw_bias = PSM_Vector3fAdd(&raw_bias, &raw_sample);
		}
		raw_bias = PSM_Vector3fUnsafeScalarDivide(&raw_bias, N);

		// Compute the mean of the error samples, where "error" = abs(omega_sample)
		// If the gyro has little to no bias then the  mean of the signed omega samples 
		// would be very close to zero since the the gyro at rest over a short period has mean-zero noise
		PSMVector3f mean_omega_error = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			PSMVector3f error_sample = PSM_Vector3fAbs(&raw_omega_samples[sample_index]);

			mean_omega_error = PSM_Vector3fAdd(&mean_omega_error, &error_sample);
		}
		mean_omega_error = PSM_Vector3fUnsafeScalarDivide(&mean_omega_error, N);

		// Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
		PSMVector3f var_omega = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			PSMVector3f error_sample = PSM_Vector3fAbs(&raw_omega_samples[sample_index]);
			PSMVector3f diff_from_mean = PSM_Vector3fSubtract(&error_sample, &mean_omega_error);
			PSMVector3f diff_from_mean_sqrd = PSM_Vector3fSquare(&diff_from_mean);

			var_omega = PSM_Vector3fAdd(&var_omega, &diff_from_mean_sqrd);
		}
		var_omega = PSM_Vector3fUnsafeScalarDivide(&var_omega, N - 1);

		// Use the max variance of all three axes (should be close)
		angular_drift_variance = PSM_Vector3fMaxValue(&var_omega);

		// Compute the max drift rate we got across a three axis
		PSMVector3f drift_rate = PSM_Vector3fUnsafeScalarDivide(&drift_rotation, sampleDurationSeconds);
		PSMVector3f drift_rate_abs = PSM_Vector3fAbs(&drift_rate);
		angular_drift_rate = PSM_Vector3fMaxValue(&drift_rate_abs);
	}
};

//-- private methods -----
static void drawHMD(PSMHeadMountedDisplay *hmdView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDGyroscopeCalibration::AppStage_HMDGyroscopeCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDGyroscopeCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_hmdView(nullptr)
    , m_isHMDStreamActive(false)
    , m_lastHMDSeqNum(-1)
	, m_playspaceYawOffset(0.f)
    , m_lastRawGyroscope()
    , m_gyroNoiseSamples(new GyroscopeNoiseSamples)
{
}

AppStage_HMDGyroscopeCalibration::~AppStage_HMDGyroscopeCalibration()
{
	if (m_gyroNoiseSamples != nullptr)
	{
		delete m_gyroNoiseSamples;
		m_gyroNoiseSamples = nullptr;
	}
}

void AppStage_HMDGyroscopeCalibration::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const AppStage_HMDSettings::HMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
	m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);

    // Reset all of the sampling state
	m_gyroNoiseSamples->clear();

    // Initialize the hmd state
    assert(hmdInfo->HmdID != -1);
    assert(m_hmdView == nullptr);
	PSM_AllocateHmdListener(hmdInfo->HmdID);
	m_hmdView = PSM_GetHmd(hmdInfo->HmdID);

    m_lastRawGyroscope = *k_psm_int_vector3_zero;
    m_lastHMDSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psm_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;

    // Start streaming in controller data
    assert(!m_isHMDStreamActive);
	PSMRequestID requestId;
	PSM_StartHmdDataStreamAsync(
		m_hmdView->HmdID, 
		PSMStreamFlags_includePositionData | // used for correcting yaw drift
		PSMStreamFlags_includeCalibratedSensorData | 
		PSMStreamFlags_includeRawSensorData, 
		&requestId);
	PSM_RegisterCallback(requestId, &AppStage_HMDGyroscopeCalibration::handle_acquire_hmd, this);

	request_playspace_info();
}

void AppStage_HMDGyroscopeCalibration::exit()
{
    assert(m_hmdView != nullptr);
    PSM_FreeHmdListener(m_hmdView->HmdID);
    m_hmdView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_HMDGyroscopeCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

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

                m_lastRawGyroscope = rawSensorData.Gyroscope;
				m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        case PSMHmd_Virtual:
            {
                m_lastRawGyroscope = {0, 0, 0};
                m_lastCalibratedGyroscope = {0, 0, 0};
                m_lastCalibratedAccelerometer = {0, 0, 0};
            }
            break;
        }

        m_lastHMDSeqNum = m_hmdView->OutputSequenceNum;
        
        if (m_bLastSampleTimeValid)
        {
            sampleTimeDeltaMilli = now - m_lastSampleTime;
        }

        m_lastSampleTime= now;
        m_bLastSampleTimeValid= true;
        
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
                    m_menuState = AppStage_HMDGyroscopeCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_HMDGyroscopeCalibration::waitForStable;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsHmdStable(m_hmdView->HmdID, &bIsStable) == PSMResult_Success;

            if (bCanBeStable && bIsStable)
            {
                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
						m_gyroNoiseSamples->clear();
						m_gyroNoiseSamples->sampleStartTime= now;
                        m_menuState= eCalibrationMenuState::measureBiasAndDrift;
                    }
                }
                else
                {
                    m_bIsStable= true;
                    m_stableStartTime= now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable= false;
                }
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift: // PSMove and DS4
        {
			bool bIsStable= false;
			bool bCanBeStable= PSM_GetIsHmdStable(m_hmdView->HmdID, &bIsStable) == PSMResult_Success;

            if (bCanBeStable && bIsStable)
            {
				// Add sample when there is new data
				if (bControllerDataUpdatedThisFrame)
				{
					const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_gyroNoiseSamples->sampleStartTime;
					const float deltaTimeSeconds = sampleTimeDeltaMilli.count() / 1000.f;
					const PSMVector3f raw_gyro = PSM_Vector3iCastToFloat(&m_lastRawGyroscope);

					// Accumulate the drift total
					if (deltaTimeSeconds > 0.f)
					{
						PSMVector3f lastRawGyro = PSM_Vector3iCastToFloat(&m_lastRawGyroscope);
						m_gyroNoiseSamples->drift_rotation = PSM_Vector3fScaleAndAdd(&lastRawGyro, deltaTimeSeconds, &m_gyroNoiseSamples->drift_rotation);
					}

					// Record the next noise sample
					if (m_gyroNoiseSamples->sample_count < k_desired_noise_sample_count)
					{
						m_gyroNoiseSamples->raw_gyro_samples[m_gyroNoiseSamples->sample_count] = m_lastRawGyroscope;
						m_gyroNoiseSamples->raw_omega_samples[m_gyroNoiseSamples->sample_count] = PSM_Vector3iCastToFloat(&m_lastRawGyroscope);
						++m_gyroNoiseSamples->sample_count;
					}
					else
					{
						// Compute bias and drift statistics
						m_gyroNoiseSamples->computeStatistics(sampleDurationMilli);

						// Update the gyro config on the service
						request_set_gyroscope_calibration(
							m_gyroNoiseSamples->raw_bias,
							m_gyroNoiseSamples->angular_drift_rate,
							m_gyroNoiseSamples->angular_drift_variance);

						m_menuState = eCalibrationMenuState::measureComplete;
					}
				}
            }
            else
            {
                m_bIsStable= false;
                m_menuState = AppStage_HMDGyroscopeCalibration::waitForStable;
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

void AppStage_HMDGyroscopeCalibration::render()
{
	const float modelScale = 18.f;
	glm::mat4 hmdTransform;

	switch (m_hmdView->HmdType)
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
    case eCalibrationMenuState::waitForStable:
        {
            drawHMD(m_hmdView, hmdTransform);

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
	case eCalibrationMenuState::measureComplete:
		{
            drawHMD(m_hmdView, hmdTransform);
        } break;
    case eCalibrationMenuState::test:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)
			PSMQuatf orientation;
			if (PSM_GetHmdOrientation(m_hmdView->HmdID, &orientation) == PSMResult_Success)
			{
				// Compensate for playspace offsets
				const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
				const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(orientation);
				orientation = PSM_QuatfCreate(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

				glm::quat q= psm_quatf_to_glm_quat(orientation);
				glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
				glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

				drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale)), 20.f);

				drawHMD(m_hmdView, worldTransform);
				drawTransformedAxes(worldTransform, 20.f);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDGyroscopeCalibration::renderUI()
{
	static float waitCount;

    const float k_panel_width = 500;
    const char *k_window_title = "HMD Gyroscope Calibration";
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
    case eCalibrationMenuState::waitForStable:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::TextWrapped(
                "Place the HMD on a level surface. Measurement will begin once the HMD is aligned with gravity and stable.");

            if (m_bIsStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
                float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

				ImGui::Separator();
                ImGui::ProgressBar(fraction, ImVec2(-1, 0), " ");
                ImGui::Spacing();
            }
            else
            {
				ImGui::Separator();
				ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), AssetManager::k_imcolor_orange());
				ImGui::SameLine();
				ImGui::TextColored(AssetManager::k_imcolor_orange(), "HMD destabilized! Waiting for stabilization...");
            }

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            const float sampleFraction = 
                static_cast<float>(m_gyroNoiseSamples->sample_count)
                / static_cast<float>(k_desired_noise_sample_count);

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
            ImGui::TextWrapped(
				"Sampling Gyroscope...\n"
				"Please do not touch the HMD and keep it steady!");
			ImGui::Separator();
            ImGui::ProgressBar(sampleFraction, ImVec2(-1, 0));
			ImGui::Separator();

            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

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
                "Press OK to continue or Redo to recalibration.");

			ImGui::Separator();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconCheck(), "OK"))
            {
                m_menuState = eCalibrationMenuState::test;
            }
            
			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconUpdate(), "Redo"))
            {
				m_gyroNoiseSamples->clear();
                m_menuState = eCalibrationMenuState::waitForStable;
            }
            
			ImGui::SameLine();
            if (AssetManager::ImGuiButtonIcon(AssetManager::getInstance()->getIconBan(), "Cancel"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin("Test Gyroscope", nullptr, window_flags);

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
void AppStage_HMDGyroscopeCalibration::request_set_gyroscope_calibration(
	const PSMVector3f &raw_bias,
    const float raw_drift, 
    const float raw_variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_GYROSCOPE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetHMDGyroscopeCalibration *calibration =
        request->mutable_set_hmd_gyroscope_calibration_request();

    calibration->set_hmd_id(m_hmdView->HmdID);

	calibration->mutable_raw_bias()->set_i(raw_bias.x);
	calibration->mutable_raw_bias()->set_j(raw_bias.y);
	calibration->mutable_raw_bias()->set_k(raw_bias.z);
    calibration->set_raw_drift(raw_drift);
    calibration->set_raw_variance(raw_variance);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDGyroscopeCalibration::request_playspace_info()
{
	if (m_menuState != AppStage_HMDGyroscopeCalibration::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_HMDGyroscopeCalibration::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_HMDGyroscopeCalibration::handle_playspace_info_response, this);
	}
}

void AppStage_HMDGyroscopeCalibration::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_HMDGyroscopeCalibration *thisPtr = static_cast<AppStage_HMDGyroscopeCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_playspaceYawOffset = response->result_get_playspace_offsets().playspace_orientation_yaw();

		thisPtr->m_menuState = AppStage_HMDGyroscopeCalibration::waitingForStreamStartResponse;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_HMDGyroscopeCalibration::failedStreamStart;
	} break;
	}
}

void AppStage_HMDGyroscopeCalibration::handle_acquire_hmd(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_HMDGyroscopeCalibration *thisPtr = reinterpret_cast<AppStage_HMDGyroscopeCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isHMDStreamActive = true;
        thisPtr->m_lastHMDSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_HMDGyroscopeCalibration::failedStreamStart;
    }
}

void AppStage_HMDGyroscopeCalibration::request_exit_to_app_stage(const char *app_stage_name)
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

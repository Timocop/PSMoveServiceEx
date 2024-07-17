//-- inludes -----
#include "AppStage_GyroscopeCalibration.h"
#include "AppStage_ControllerSettings.h"
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

//-- statics ----
const char *AppStage_GyroscopeCalibration::APP_STAGE_NAME = "GyroscopeCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 3000.f;
const int k_desired_noise_sample_count = 1000;

const int k_desired_scale_sample_count = 1000;

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
        drift_rotation= *k_psm_float_vector3_zero;
        sample_count= 0;
		raw_bias= *k_psm_float_vector3_zero;
        angular_drift_variance= 0.f;
        angular_drift_rate= 0.f;
    }

    void computeStatistics(std::chrono::duration<float, std::milli> sampleDurationMilli)
    {
        const float sampleDurationSeconds= sampleDurationMilli.count() / 1000.f;
        const float N = static_cast<float>(sample_count);

        // Compute the mean of the raw gyro measurements
        // If this is a non-zero value then the gyro has some per frame
		// bias we need to subtract off.
        raw_bias= *k_psm_float_vector3_zero;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMVector3f raw_sample= PSM_Vector3iCastToFloat(&raw_gyro_samples[sample_index]);

            raw_bias= PSM_Vector3fAdd(&raw_bias, &raw_sample);
        }
        raw_bias= PSM_Vector3fUnsafeScalarDivide(&raw_bias, N);

        // Compute the mean of the error samples, where "error" = abs(omega_sample)
        // If the gyro has little to no bias then the  mean of the signed omega samples 
		// would be very close to zero since the the gyro at rest over a short period has mean-zero noise
        PSMVector3f mean_omega_error= *k_psm_float_vector3_zero;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMVector3f error_sample= PSM_Vector3fAbs(&raw_omega_samples[sample_index]);

            mean_omega_error= PSM_Vector3fAdd(&mean_omega_error, &error_sample);
        }
        mean_omega_error= PSM_Vector3fUnsafeScalarDivide(&mean_omega_error, N);

        // Compute the variance of the (unsigned) sample error, where "error" = abs(omega_sample)
        PSMVector3f var_omega= *k_psm_float_vector3_zero;
        for (int sample_index = 0; sample_index < sample_count; sample_index++)
        {
            PSMVector3f error_sample= PSM_Vector3fAbs(&raw_omega_samples[sample_index]);
            PSMVector3f diff_from_mean= PSM_Vector3fSubtract(&error_sample, &mean_omega_error);
            PSMVector3f diff_from_mean_sqrd= PSM_Vector3fSquare(&diff_from_mean);

            var_omega= PSM_Vector3fAdd(&var_omega, &diff_from_mean_sqrd);
        }
        var_omega= PSM_Vector3fUnsafeScalarDivide(&var_omega, N - 1);

        // Use the max variance of all three axes (should be close)
        angular_drift_variance= PSM_Vector3fMaxValue(&var_omega);

        // Compute the max drift rate we got across a three axis
        PSMVector3f drift_rate= PSM_Vector3fUnsafeScalarDivide(&drift_rotation, sampleDurationSeconds);
        PSMVector3f drift_rate_abs= PSM_Vector3fAbs(&drift_rate);
        angular_drift_rate= PSM_Vector3fMaxValue(&drift_rate_abs);
    }
};

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_GyroscopeCalibration::AppStage_GyroscopeCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_GyroscopeCalibration::inactive)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
	, m_playspaceYawOffset(0.f)
    , m_lastRawGyroscope()
    , m_gyroNoiseSamples(new GyroscopeNoiseSamples)
{
}

AppStage_GyroscopeCalibration::~AppStage_GyroscopeCalibration()
{
	if (m_gyroNoiseSamples != nullptr)
	{
		delete m_gyroNoiseSamples;
		m_gyroNoiseSamples = nullptr;
	}
}

void AppStage_GyroscopeCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

	m_menuState = eCalibrationMenuState::inactive;

    // Reset all of the sampling state
    m_gyroNoiseSamples->clear();

    m_lastRawGyroscope = *k_psm_int_vector3_zero;
    m_lastControllerSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_psm_float_vector3_zero;

    m_stableStartTime = std::chrono::time_point<std::chrono::high_resolution_clock>();
    m_bIsStable= false;
	m_bForceControllerStable= false;

	// Initialize the controller state
	assert(controllerInfo->ControllerID != -1);
	assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
	m_controllerView= PSM_GetController(controllerInfo->ControllerID);

	request_playspace_info();
}

void AppStage_GyroscopeCalibration::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    setState(eCalibrationMenuState::inactive);
}

void AppStage_GyroscopeCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
        switch(m_controllerView->ControllerType)
        {
        case PSMController_DualShock4:
            {
                const PSMDS4RawSensorData &rawSensorData =
                    m_controllerView->ControllerState.PSDS4State.RawSensorData;
                const PSMDS4CalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSDS4State.CalibratedSensorData;

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        case PSMController_Move:
            {
                const PSMPSMoveRawSensorData &rawSensorData =
                    m_controllerView->ControllerState.PSMoveState.RawSensorData;
                const PSMPSMoveCalibratedSensorData &calibratedSensorData =
                    m_controllerView->ControllerState.PSMoveState.CalibratedSensorData;

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        }

        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
        
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
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
		{
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                if (m_bBypassCalibration)
                {
                    setState(AppStage_GyroscopeCalibration::test);
                }
                else
                {
                    setState(AppStage_GyroscopeCalibration::waitForStable);
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
			bool bIsStable;
			bool bCanBeStabilized= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStabilized && bIsStable) || m_bForceControllerStable)
            {
                if (m_bIsStable || m_bForceControllerStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
    
                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        m_gyroNoiseSamples->clear();
                        m_gyroNoiseSamples->sampleStartTime= now;
                        setState(eCalibrationMenuState::measureBiasAndDrift);
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
			bool bIsStable;
			bool bCanBeStabilized= PSM_GetIsControllerStable(m_controllerView->ControllerID, &bIsStable) == PSMResult_Success;

            if ((bCanBeStabilized && bIsStable) || m_bForceControllerStable)
            {
				// Add sample when there is new data
				if (bControllerDataUpdatedThisFrame)
				{
					const std::chrono::duration<float, std::milli> sampleDurationMilli = now - m_gyroNoiseSamples->sampleStartTime;
					const float deltaTimeSeconds = sampleTimeDeltaMilli.count() / 1000.f;

					// Accumulate the drift total
					if (deltaTimeSeconds > 0.f)
					{
						PSMVector3f lastRawGyro = PSM_Vector3iCastToFloat(&m_lastRawGyroscope);
						m_gyroNoiseSamples->drift_rotation =
							PSM_Vector3fScaleAndAdd(&lastRawGyro, deltaTimeSeconds, &m_gyroNoiseSamples->drift_rotation);
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

						setState(eCalibrationMenuState::measureComplete);
					}
				}
            }
            else
            {
                m_bIsStable= false;
                setState(AppStage_GyroscopeCalibration::waitForStable);
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

void AppStage_GyroscopeCalibration::render()
{
	const float k_modelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);

            // Draw the current direction of gravity
            {
                const float renderScale = 200.f;
                glm::mat4 renderScaleMatrix = 
                    glm::scale(glm::mat4(1.f), glm::vec3(renderScale, renderScale, renderScale));
                glm::vec3 g= -psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(
                    renderScaleMatrix,
                    glm::vec3(), g, 
                    0.1f, 
                    glm::vec3(0.f, 1.f, 0.f));
                drawTextAtWorldPosition(renderScaleMatrix, g, "G");
            }
        } break;
    case eCalibrationMenuState::measureBiasAndDrift:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            drawController(m_controllerView, scaleAndRotateModelX90);
        } break;
    case eCalibrationMenuState::test:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)  
			PSMQuatf orientation;
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &orientation) == PSMResult_Success)
			{
				// Compensate for playspace offsets
				const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
				const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(orientation);
				orientation = PSM_QuatfCreate(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

				glm::quat q = psm_quatf_to_glm_quat(orientation);
				glm::mat4 worldSpaceOrientation = glm::mat4_cast(q);
				glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(k_modelScale, k_modelScale, k_modelScale));

				drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)), 20.f);

				drawPSMoveModel(worldTransform, glm::vec3(1.f, 1.f, 1.f));
				drawTransformedAxes(worldTransform, 20.f);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_GyroscopeCalibration::renderUI()
{
	static float waitCount;

    const float k_panel_width = 500;
    const char *k_window_title = "Gyroscope Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
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
            ImGui::Text("Waiting for server response...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Failed server request!");

			ImGui::Separator();
            if (ImGui::Button("      OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

            ImGui::SameLine();
            if (ImGui::Button("      Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::waitForStable:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
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
            ImGui::TextWrapped(
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Set the controller down on a level surface.\n" \
                "Measurement will start once the controller is aligned with gravity and stable.");

            if (m_bIsStable || m_bForceControllerStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;
                float fraction = static_cast<float>(stableDuration.count() / k_stabilize_wait_time_ms);

				ImGui::Separator();
                ImGui::ProgressBar(fraction, ImVec2(-1, 0), " ");
            }
            else
            {
				ImGui::Separator();
				ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), ImColor(1.f, 0.5f, 0.f));
				ImGui::SameLine();
				ImGui::TextColored(ImColor(1.f, 0.5f, 0.f), "Controller destabilized! Waiting for stabilization..");
            }

			ImGui::Separator();
            if (ImGui::Button("      Force Continue"))
            {
                m_bForceControllerStable= true;
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
            
			ImGui::SameLine();
            if (ImGui::Button("      Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

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
                "[Step 1 of 2: Measuring gyroscope drift and bias]\n" \
                "Sampling Gyroscope...");
			ImGui::Separator();
            ImGui::ProgressBar(sampleFraction, ImVec2(-1, 0));

			ImGui::Separator();
            if (ImGui::Button("      Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Sampling complete.\n" \
                "Press OK to continue or Redo to recalibration.");

			ImGui::Separator();
            if (ImGui::Button("      OK"))
            {
				PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
                setState(eCalibrationMenuState::test);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
            
			ImGui::SameLine();
            if (ImGui::Button("      Redo"))
            {
                m_gyroNoiseSamples->clear();
                setState(eCalibrationMenuState::waitForStable);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconUpdate()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
           
			ImGui::SameLine();
            if (ImGui::Button("      Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::Begin("Test Orientation", nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->ControllerID);
            }
            else
            {
				ImGui::Image(AssetManager::getInstance()->getIconCheck()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->ControllerID);
            }

			PSMQuatf orientation;
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &orientation) == PSMResult_Success)
			{
				// Compensate for playspace offsets
				const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
				const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(orientation);
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);
				orientation = PSM_QuatfCreate(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

				ImGui::Text("Pitch(x): %.2f, Yaw(y): %.2f, Roll(z): %.2f",
					m_lastCalibratedGyroscope.x * k_radians_to_degreees, 
					m_lastCalibratedGyroscope.y * k_radians_to_degreees,
					m_lastCalibratedGyroscope.z * k_radians_to_degreees);
				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f", 
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}

			if (m_controllerView->ControllerType == PSMController_DualShock4)
			{
				ImGui::TextWrapped(
					"[Press the Options button with controller pointed straight forward\n" \
					 "to recenter the controller]");
			}
			else if (m_controllerView->ControllerType == PSMController_Move)
			{
				ImGui::TextWrapped(
					"[Hold the Select button with controller pointed forward\n" \
					"to recenter the controller]");
			}

			ImGui::Separator();
            if (ImGui::Button("      OK"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

            ImGui::SameLine();
            if (ImGui::Button("      Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

//-- private methods -----
void AppStage_GyroscopeCalibration::setState(eCalibrationMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_GyroscopeCalibration::onExitState(eCalibrationMenuState newState)
{
	switch (m_menuState)
	{
	case eCalibrationMenuState::inactive:
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::waitForStable:
	case eCalibrationMenuState::measureBiasAndDrift:
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_GyroscopeCalibration::onEnterState(eCalibrationMenuState newState)
{
	switch (newState)
	{
	case eCalibrationMenuState::inactive:
		// Reset the orbit camera back to default orientation and scale
		m_app->getOrbitCamera()->reset();
		break;
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::pendingTrackingSpaceSettings:
		// Reset the menu state
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->resetOrientation();
		m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
		break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
		break;
	case eCalibrationMenuState::waitForStable:
		m_bForceControllerStable= false;
		m_bIsStable= false;
		break;
	case eCalibrationMenuState::measureBiasAndDrift:
		break;
	case eCalibrationMenuState::measureComplete:
		// Reset the menu state
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->resetOrientation();
		m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
		break;
	case eCalibrationMenuState::test:
		{
			switch (m_controllerView->ControllerType)
			{
			case PSMController_DualShock4:
				m_controllerView->ControllerState.PSDS4State.bPoseResetButtonEnabled= true;
				break;
			case PSMController_Move:
				m_controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;
				break;
			}

			m_app->setCameraType(_cameraOrbit);
			m_app->getOrbitCamera()->resetOrientation();
			m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
		}
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_GyroscopeCalibration::request_playspace_info()
{
	if (m_menuState != AppStage_GyroscopeCalibration::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_GyroscopeCalibration::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_GyroscopeCalibration::handle_playspace_info_response, this);
	}
}

void AppStage_GyroscopeCalibration::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_GyroscopeCalibration *thisPtr = static_cast<AppStage_GyroscopeCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_playspaceYawOffset = response->result_get_playspace_offsets().playspace_orientation_yaw();

		thisPtr->request_tracking_space_settings();
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_GyroscopeCalibration::failedStreamStart;
	} break;
	}
}

void AppStage_GyroscopeCalibration::request_tracking_space_settings()
{
	if (m_menuState != eCalibrationMenuState::pendingTrackingSpaceSettings)
	{
		setState(eCalibrationMenuState::pendingTrackingSpaceSettings);

		PSMRequestID request_id;
		PSM_GetTrackingSpaceSettingsAsync(&request_id);
		PSM_RegisterCallback(request_id, AppStage_GyroscopeCalibration::handle_tracking_space_settings_response, this);
	}
}

void AppStage_GyroscopeCalibration::handle_tracking_space_settings_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_GyroscopeCalibration *thisPtr = static_cast<AppStage_GyroscopeCalibration *>(userdata);

	switch (response_message->result_code)
	{
	case PSMResult_Success:
		{
			const AppStage_ControllerSettings *controllerSettings =
				thisPtr->m_app->getAppStage<AppStage_ControllerSettings>();
			const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
				controllerSettings->getSelectedControllerInfo();

			assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackingSpace);

			// Save the tracking space settings (used in rendering)
			thisPtr->m_global_forward_degrees = response_message->payload.tracking_space.global_forward_degrees;

			unsigned int stream_flags = 
				PSMStreamFlags_includeRawSensorData |
				PSMStreamFlags_includeCalibratedSensorData;

			if (controllerInfo->ControllerType == PSMController_DualShock4)
			{
				// Need to turn on optical tracking for the DS4 so that we have get optical orientation
				stream_flags |= PSMStreamFlags_includePositionData;
			}

			// Start streaming in controller data
			assert(!thisPtr->m_isControllerStreamActive);
			PSMRequestID request_id;
			PSM_StartControllerDataStreamAsync(thisPtr->m_controllerView->ControllerID, stream_flags, &request_id);
			PSM_RegisterCallback(request_id, &AppStage_GyroscopeCalibration::handle_acquire_controller, thisPtr);

			thisPtr->setState(eCalibrationMenuState::waitingForStreamStartResponse);
		} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(eCalibrationMenuState::failedStreamStart);
		} break;
	}
}

void AppStage_GyroscopeCalibration::request_set_gyroscope_calibration(
	const PSMVector3f &raw_bias,
    const float drift, 
    const float variance)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_GYROSCOPE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetControllerGyroscopeCalibration *calibration =
        request->mutable_set_controller_gyroscope_calibration_request();

    calibration->set_controller_id(m_controllerView->ControllerID);

	calibration->mutable_raw_bias()->set_i(raw_bias.x);
	calibration->mutable_raw_bias()->set_j(raw_bias.y);
	calibration->mutable_raw_bias()->set_k(raw_bias.z);
    calibration->set_drift(drift);
    calibration->set_variance(variance);
	calibration->set_gyro_gain_setting(""); // keep existing gain

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_GyroscopeCalibration::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_GyroscopeCalibration *thisPtr = reinterpret_cast<AppStage_GyroscopeCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->setState(AppStage_GyroscopeCalibration::failedStreamStart);
    }
}

void AppStage_GyroscopeCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
	if (m_isControllerStreamActive)
	{
		PSMRequestID request_id;
		PSM_StopControllerDataStreamAsync(m_controllerView->ControllerID, &request_id);
		PSM_EatResponse(request_id);

		m_isControllerStreamActive = false;
		m_app->setAppStage(app_stage_name);
	}
	else
	{
		m_app->setAppStage(app_stage_name);
	}
}

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform)
{
    switch(controllerView->ControllerType)
    {
    case PSMController_Move:
        drawPSMoveModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case PSMController_DualShock4:
        drawPSDualShock4Model(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

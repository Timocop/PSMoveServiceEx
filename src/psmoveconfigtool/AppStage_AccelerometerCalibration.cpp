//-- inludes -----
#include "AppStage_AccelerometerCalibration.h"
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
#include "PSMoveClient_CAPI.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_AccelerometerCalibration::APP_STAGE_NAME = "AcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 3000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- definitions -----
//-- definitions -----
struct ControllerAccelerometerPoseSamples
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PSMVector3f raw_accelerometer_samples[k_max_accelerometer_samples];
	PSMVector3f raw_average_gravity;
	float raw_variance; // Max raw sensor variance (raw_sensor_units^2)
	int sample_count;

	void clear()
	{
		sample_count = 0;
		raw_average_gravity = *k_psm_float_vector3_zero;
		raw_variance = 0.f;
	}

	void computeStatistics()
	{
		const float N = static_cast<float>(k_max_accelerometer_samples);

		// Compute both mean of signed and unsigned samples
		PSMVector3f mean_acc_abs_error = *k_psm_float_vector3_zero;
		raw_average_gravity = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < k_max_accelerometer_samples; ++sample_index)
		{
			PSMVector3f signed_error_sample = raw_accelerometer_samples[sample_index];
			PSMVector3f unsigned_error_sample = PSM_Vector3fAbs(&signed_error_sample);

			mean_acc_abs_error = PSM_Vector3fAdd(&mean_acc_abs_error, &unsigned_error_sample);
			raw_average_gravity = PSM_Vector3fAdd(&raw_average_gravity, &signed_error_sample);
		}
		mean_acc_abs_error = PSM_Vector3fUnsafeScalarDivide(&mean_acc_abs_error, N);
		raw_average_gravity = PSM_Vector3fUnsafeScalarDivide(&raw_average_gravity, N);

		// Compute the variance of the (unsigned) sample error, where "error" = abs(accelerometer_sample)
		PSMVector3f var_accelerometer = *k_psm_float_vector3_zero;
		for (int sample_index = 0; sample_index < sample_count; sample_index++)
		{
			PSMVector3f unsigned_error_sample = PSM_Vector3fAbs(&raw_accelerometer_samples[sample_index]);
			PSMVector3f diff_from_mean = PSM_Vector3fSubtract(&unsigned_error_sample, &mean_acc_abs_error);
			PSMVector3f diff_from_mean_sqrd = PSM_Vector3fSquare(&diff_from_mean);

			var_accelerometer = PSM_Vector3fAdd(&var_accelerometer, &diff_from_mean_sqrd);
		}
		var_accelerometer = PSM_Vector3fUnsafeScalarDivide(&var_accelerometer, N - 1);

		// Use the max variance of all three axes (should be close)
		raw_variance = PSM_Vector3fMaxValue(&var_accelerometer);
	}
};

//-- private methods -----
static void request_set_accelerometer_calibration(
	const int controller_id,
	const PSMVector3f &raw_average_gravity,
	const float variance);
static void drawController(PSMController *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_AccelerometerCalibration::AppStage_AccelerometerCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_AccelerometerCalibration::inactive)
	, m_testMode(AppStage_AccelerometerCalibration::controllerRelative)
    , m_bBypassCalibration(false)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
	, m_playspaceYawOffset(0.f)
    , m_noiseSamples(new ControllerAccelerometerPoseSamples)
{
}

AppStage_AccelerometerCalibration::~AppStage_AccelerometerCalibration()
{
    delete m_noiseSamples;
}

void AppStage_AccelerometerCalibration::enter()
{
    const AppStage_ControllerSettings *controllerSettings =
        m_app->getAppStage<AppStage_ControllerSettings>();
    const AppStage_ControllerSettings::ControllerInfo *controllerInfo =
        controllerSettings->getSelectedControllerInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    m_noiseSamples->clear();

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(controllerInfo->ControllerID);
    m_controllerView = PSM_GetController(controllerInfo->ControllerID);

	m_lastRawAccelerometer = *k_psm_int_vector3_zero;
	m_lastCalibratedAccelerometer = *k_psm_float_vector3_zero;
    m_lastControllerSeqNum = -1;

    // Start streaming in controller data
    assert(!m_isControllerStreamActive);
	PSMRequestID request_id;
	PSM_StartControllerDataStreamAsync(
		m_controllerView->ControllerID, 
		PSMStreamFlags_includeCalibratedSensorData | PSMStreamFlags_includeRawSensorData, 
		&request_id);
	PSM_RegisterCallback(request_id, &AppStage_AccelerometerCalibration::handle_acquire_controller, this);

	request_playspace_info();
}

void AppStage_AccelerometerCalibration::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_AccelerometerCalibration::update()
{
    bool bControllerDataUpdatedThisFrame = false;

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
        switch(m_controllerView->ControllerType)
        {
        case PSMController_DualShock4:
            {
				const PSMDS4RawSensorData &rawSensorData =
					m_controllerView->ControllerState.PSDS4State.RawSensorData;
				const PSMDS4CalibratedSensorData &calibratedSensorData =
					m_controllerView->ControllerState.PSDS4State. CalibratedSensorData;

				m_lastRawAccelerometer = rawSensorData.Accelerometer;
				m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        case PSMController_Move:
            {
				const PSMPSMoveRawSensorData &rawSensorData =
					m_controllerView->ControllerState.PSMoveState.RawSensorData;
				const PSMPSMoveCalibratedSensorData &calibratedSensorData =
					m_controllerView->ControllerState.PSMoveState.CalibratedSensorData;

				m_lastRawAccelerometer = rawSensorData.Accelerometer;
				m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        default:
            assert(0 && "unreachable");
        }

        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
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
                    m_menuState = AppStage_AccelerometerCalibration::test;
                }
                else
                {
                    m_menuState = AppStage_AccelerometerCalibration::placeController;
                }
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
    case eCalibrationMenuState::placeController:
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
					request_set_accelerometer_calibration(
						m_controllerView->ControllerID,
						m_noiseSamples->raw_average_gravity,
						m_noiseSamples->raw_variance);

                    m_menuState = AppStage_AccelerometerCalibration::measureComplete;
                }
            }
        } break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {
			if (m_controllerView->ControllerType == PSMController_DualShock4 &&
				m_controllerView->ControllerState.PSDS4State.OptionsButton == PSMButtonState_PRESSED)
			{
                PSMRequestID request_id;
				PSM_ResetControllerOrientationAsync(m_controllerView->ControllerID, k_psm_quaternion_identity, &request_id);
                PSM_EatResponse(request_id);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::render()
{
    const float k_modelScale = 18.f;
    glm::mat4 defaultControllerTransform;

    switch(m_controllerView->ControllerType)
    {
    case PSMController_Move:
		defaultControllerTransform= 
			glm::rotate(
				glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)),
				90.f, glm::vec3(1.f, 0.f, 0.f));  
        break;
    case PSMController_DualShock4:
        defaultControllerTransform = glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale));
        break;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingPlayspaceRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::placeController:
        {
			const float sampleScale = 0.05f;
			glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller model in the pose we want the user place it in
            drawController(m_controllerView, defaultControllerTransform);

			// Draw the current raw accelerometer direction
			{
				PSMVector3f lastRawAccel = PSM_Vector3iCastToFloat(&m_lastRawAccelerometer);
				glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
				glm::vec3 m_end = psm_vector3f_to_glm_vec3(lastRawAccel);

				drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
				drawTextAtWorldPosition(sampleTransform, m_end, "A");
			}
        } break;
    case eCalibrationMenuState::measureNoise:
    case eCalibrationMenuState::measureComplete:
        {
            const float sampleScale = 0.05f;
            glm::mat4 sampleTransform = glm::scale(glm::mat4(1.f), glm::vec3(sampleScale, sampleScale, sampleScale));

            // Draw the controller in the middle            
            drawController(m_controllerView, defaultControllerTransform);
			
            // Draw the sample point cloud around the origin
            drawPointCloud(sampleTransform, glm::vec3(1.f, 1.f, 1.f), 
                reinterpret_cast<float *>(m_noiseSamples->raw_accelerometer_samples), 
                m_noiseSamples->sample_count);

            // Draw the current raw accelerometer direction
            {
				PSMVector3f lastRawAccel = PSM_Vector3iCastToFloat(&m_lastRawAccelerometer);
                glm::vec3 m_start = glm::vec3(0.f, 0.f, 0.f);
                glm::vec3 m_end = psm_vector3f_to_glm_vec3(lastRawAccel);

                drawArrow(sampleTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sampleTransform, m_end, "A");
            }
        } break;
    case eCalibrationMenuState::test:
        {
			const float k_sensorScale = 200.f;

			glm::mat4 controllerTransform;
			glm::mat4 sensorTransform;

			switch (m_testMode)
			{
			case eTestMode::controllerRelative:
				{
					controllerTransform = defaultControllerTransform;
					sensorTransform = glm::scale(glm::mat4(1.f), glm::vec3(k_sensorScale, k_sensorScale, k_sensorScale));
				} break;
			case eTestMode::worldRelative:
				{
					// Get the orientation of the controller in world space (OpenGL Coordinate System)            
					glm::quat q;

					switch(m_controllerView->ControllerType)
					{
					case PSMController_Move:
					{
						// Compensate for playspace offsets
						const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
						const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(m_controllerView->ControllerState.PSMoveState.Pose.Orientation);
						q = glm::quat(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());
						break;
					}
					case PSMController_DualShock4:
					{
						// Compensate for playspace offsets
						const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-m_playspaceYawOffset * k_degrees_to_radians, Eigen::Vector3f::UnitY());
						const Eigen::Quaternionf eigen_quat = offset_yaw.inverse() * psm_quatf_to_eigen_quaternionf(m_controllerView->ControllerState.PSDS4State.Pose.Orientation);
						q = glm::quat(eigen_quat.w(), eigen_quat.x(), eigen_quat.y(), eigen_quat.z());

						break;
					}
					}

					glm::mat4 worldSpaceOrientation = glm::mat4_cast(q);
					
					controllerTransform = glm::scale(worldSpaceOrientation, glm::vec3(k_modelScale, k_modelScale, k_modelScale));
					sensorTransform = glm::rotate(
						glm::scale(worldSpaceOrientation, glm::vec3(k_sensorScale, k_sensorScale, k_sensorScale)),
						-90.f, glm::vec3(1.f, 0.f, 0.f));
				} break;
			default:
				assert(0 && "unreachable");
			}

			// Draw the fixed world space axes
			drawTransformedAxes(glm::scale(glm::mat4(1.f), glm::vec3(k_modelScale, k_modelScale, k_modelScale)), 20.f);

			// Draw the controller
            drawController(m_controllerView, controllerTransform);
            drawTransformedAxes(controllerTransform, 20.f);

			// Draw the accelerometer
			{
				const float accel_g = PSM_Vector3fLength(&m_lastCalibratedAccelerometer);
				glm::vec3 m_start = glm::vec3(0.f);
				glm::vec3 m_end = psm_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

				drawArrow(sensorTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 1.f, 1.f));
				drawTextAtWorldPosition(sensorTransform, m_end, "A(%.1fg)", accel_g);
			}

        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_AccelerometerCalibration::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Accelerometer Calibration";
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
			ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
			ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Waiting for server response...");

			ImGui::End();
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to start controller stream!");

            if (ImGui::Button(" OK "))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::placeController:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

			switch(m_controllerView->ControllerType)
			{
			case PSMController_Move:
				ImGui::Text("Stand the controller on a flat, level surface.");
				break;
			case PSMController_DualShock4:
				ImGui::Text("Lay the controller on a flat, level surface.");
				break;
			}

            if (ImGui::Button("Start Sampling"))
            {
                m_menuState = eCalibrationMenuState::measureNoise;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureNoise:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            float sampleFraction =
                static_cast<float>(m_noiseSamples->sample_count)
                / static_cast<float>(k_max_accelerometer_samples);

            ImGui::Text("Sampling accelerometer.");
            ImGui::ProgressBar(sampleFraction, ImVec2(250, 20));

            ImGui::End();
        } break;
    case eCalibrationMenuState::measureComplete:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::TextWrapped(
                "Sampling complete.\n" \
                "Press OK to continue or Redo to resample.");

            if (ImGui::Button(" OK "))
            {
				PSM_SetControllerLEDOverrideColor(m_controllerView->ControllerID, 0, 0, 0);
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }
            ImGui::SameLine();
            if (ImGui::Button("Redo"))
            {
                // Reset the sample info for the current pose
                m_noiseSamples->clear();
                m_menuState = eCalibrationMenuState::placeController;
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin("Test Accelerometer", nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of Controller ID #%d", m_controllerView->ControllerID);
            }
            else
            {
                ImGui::Text("Calibration of Controller ID #%d complete!", m_controllerView->ControllerID);
            }

			if (m_testMode == eTestMode::controllerRelative)
			{
				if (ImGui::Button("World Relative"))
				{
					m_testMode = eTestMode::worldRelative;
				}
			}
			else if (m_testMode == eTestMode::worldRelative)
			{
				if (ImGui::Button("Controller Relative"))
				{
					m_testMode= eTestMode::controllerRelative;
				}
			}				

            if (ImGui::Button(" OK "))
            {
                request_exit_to_app_stage(AppStage_ControllerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

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

//-- private methods -----
static void request_set_accelerometer_calibration(
	const int controller_id,
	const PSMVector3f &raw_average_gravity,
	const float variance)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_ACCELEROMETER_CALIBRATION_EX);

	PSMoveProtocol::Request_RequestSetControllerAccelerometerCalibrationEx *calibration =
		request->mutable_set_controller_accelerometer_calibration_ex_request();

	calibration->set_controller_id(controller_id);
	calibration->mutable_raw_average_gravity()->set_i(raw_average_gravity.x);
	calibration->mutable_raw_average_gravity()->set_j(raw_average_gravity.y);
	calibration->mutable_raw_average_gravity()->set_k(raw_average_gravity.z);
	calibration->set_raw_variance(variance);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_AccelerometerCalibration::request_playspace_info()
{
	if (m_menuState != AppStage_AccelerometerCalibration::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_AccelerometerCalibration::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_AccelerometerCalibration::handle_playspace_info_response, this);
	}
}

void AppStage_AccelerometerCalibration::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_AccelerometerCalibration *thisPtr = static_cast<AppStage_AccelerometerCalibration *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_playspaceYawOffset = response->result_get_playspace_offsets().playspace_orientation_yaw();

		thisPtr->m_menuState = AppStage_AccelerometerCalibration::waitingForStreamStartResponse;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_AccelerometerCalibration::failedStreamStart;
	} break;
	}
}

void AppStage_AccelerometerCalibration::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_AccelerometerCalibration *thisPtr = reinterpret_cast<AppStage_AccelerometerCalibration *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
    }
    else
    {
        thisPtr->m_menuState = AppStage_AccelerometerCalibration::failedStreamStart;
    }
}

void AppStage_AccelerometerCalibration::request_exit_to_app_stage(const char *app_stage_name)
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

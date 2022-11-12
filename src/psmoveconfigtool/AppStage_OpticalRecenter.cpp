//-- inludes -----
#include "AppStage_OpticalRecenter.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"

#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_OpticalRecenter::APP_STAGE_NAME = "OpticalRecenter";
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);  
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- constants -----

//-- definitions -----

//-- private methods -----
static void drawController(PSMController *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_OpticalRecenter::AppStage_OpticalRecenter(App *app)
    : AppStage(app)
    , m_menuState(AppStage_OpticalRecenter::inactive)
    , m_controllerView(nullptr)
    , m_isControllerStreamActive(false)
    , m_lastControllerSeqNum(-1)
	, m_bLastMulticamPositionValid(false)
	, m_bLastMulticamOrientationValid(false)
{	
	m_lastMulticamPositionCm = *k_psm_float_vector3_zero;
	m_lastMulticamOrientation = *k_psm_quaternion_identity;
	m_lastControllerPose = *k_psm_pose_identity;

	m_CenterSample = *k_psm_float_vector3_zero;
	m_ForwardSample = *k_psm_float_vector3_zero;

	memset(&m_trackerList, 0, sizeof(m_trackerList));
}

AppStage_OpticalRecenter::~AppStage_OpticalRecenter()
{
}

void AppStage_OpticalRecenter::enter()
{
	m_app->setCameraType(_cameraOrbit);
	m_menuState = eCalibrationMenuState::inactive;

    // Initialize the controller state
    assert(controllerInfo->ControllerID != -1);
    assert(m_controllerView == nullptr);
	PSM_AllocateControllerListener(m_iControllerId);
	m_controllerView= PSM_GetController(m_iControllerId);

    m_lastControllerSeqNum = -1;
	m_lastMulticamPositionCm = *k_psm_float_vector3_zero;
	m_lastMulticamOrientation = *k_psm_quaternion_identity;
	m_lastControllerPose = *k_psm_pose_identity;
	m_bLastMulticamPositionValid = false;
	m_bLastMulticamOrientationValid = false;

	m_CenterSample = *k_psm_float_vector3_zero;
	m_ForwardSample = *k_psm_float_vector3_zero;

	memset(&m_trackerList, 0, sizeof(m_trackerList));

	// Get a list of trackers first
	request_tracker_list();
}

void AppStage_OpticalRecenter::exit()
{
    assert(m_controllerView != nullptr);
    PSM_FreeControllerListener(m_controllerView->ControllerID);
    m_controllerView = nullptr;
    setState(eCalibrationMenuState::inactive);
}

void AppStage_OpticalRecenter::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isControllerStreamActive && m_controllerView->OutputSequenceNum != m_lastControllerSeqNum)
    {
		PSMRawTrackerData rawTrackerData;
		memset(&rawTrackerData, 0, sizeof(PSMRawTrackerData));

		m_bLastMulticamPositionValid = false;
		m_bLastMulticamOrientationValid = false;

        switch(m_controllerView->ControllerType)
        {
        case PSMController_DualShock4:
            {
                rawTrackerData = m_controllerView->ControllerState.PSDS4State.RawTrackerData;

				if (rawTrackerData.bMulticamOrientationValid)
				{
					m_lastMulticamOrientation = rawTrackerData.MulticamOrientation;
					m_bLastMulticamOrientationValid = true;
				}

				m_lastControllerPose = m_controllerView->ControllerState.PSDS4State.Pose;
            }
            break;
        case PSMController_Move:
            {
				rawTrackerData = m_controllerView->ControllerState.PSMoveState.RawTrackerData;

				m_lastControllerPose = m_controllerView->ControllerState.PSMoveState.Pose;
				m_lastMulticamOrientation = *k_psm_quaternion_identity;
				m_bLastMulticamOrientationValid = true;
            }
            break;
        case PSMController_Virtual:
            {
				rawTrackerData = m_controllerView->ControllerState.VirtualController.RawTrackerData;

				m_lastControllerPose = m_controllerView->ControllerState.VirtualController.Pose;
				m_lastMulticamOrientation = *k_psm_quaternion_identity;
				m_bLastMulticamOrientationValid = true;
            }
            break;
        }

		if (rawTrackerData.bMulticamPositionValid)
		{
			m_lastMulticamPositionCm = rawTrackerData.MulticamPositionCm;
			m_bLastMulticamPositionValid = true;
		}

        m_lastControllerSeqNum = m_controllerView->OutputSequenceNum;
                
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
		{
		} break;
	case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
				setState(AppStage_OpticalRecenter::waitForStablePosition);
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitForStablePosition:
		{
		} break;
    case eCalibrationMenuState::measureOpticalPosition:
        {
            
        } break;
	case eCalibrationMenuState::waitForStableOrientation:
	{
	} break;
	case eCalibrationMenuState::measureOpticalOrientation:
	{
	} break;
    case eCalibrationMenuState::measureComplete:
    case eCalibrationMenuState::test:
        {

        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_OpticalRecenter::render()
{
    const float bigModelScale = 18.f;
    glm::mat4 scaleAndRotateModelX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(bigModelScale, bigModelScale, bigModelScale)),
            90.f, glm::vec3(1.f, 0.f, 0.f));  

	glm::mat4 controllerWorldTransform= glm::mat4(1.f);
	if (m_lastControllerSeqNum != -1 && 
		m_isControllerStreamActive)
	{
		bool bIsTracking= false;
		bool bCanBeTracked= PSM_GetIsControllerTracking(m_controllerView->ControllerID, &bIsTracking) == PSMResult_Success;

		if (bCanBeTracked && bIsTracking)
		{
			PSMPosef psmove_space_pose = {m_lastMulticamPositionCm, m_lastMulticamOrientation};

			if (m_controllerView->ControllerType == PSMController_Move ||
                m_controllerView->ControllerType == PSMController_Virtual)
			{
				psmove_space_pose.Orientation = m_lastControllerPose.Orientation;
			}

			controllerWorldTransform = psm_posef_to_glm_mat4(psmove_space_pose);
		}
	}

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
        {
        } break;
	case eCalibrationMenuState::waitForStablePosition:
	case eCalibrationMenuState::measureOpticalPosition:
	case eCalibrationMenuState::waitForStableOrientation:
	case eCalibrationMenuState::measureOpticalOrientation:
		{
			bool bIsTracking= false;
			bool bCanBeTracked= PSM_GetIsControllerTracking(m_controllerView->ControllerID, &bIsTracking) == PSMResult_Success;

			// Show the controller with optically derived pose
			if (bCanBeTracked && bIsTracking)
			{
				drawController(m_controllerView, controllerWorldTransform);
				drawTransformedAxes(controllerWorldTransform, 10.f);
			}

			drawTransformedAxes(glm::mat4(1.f), 200.f);
			drawTransformeGrid(glm::mat4(1.f), 250.f);
			drawTrackerListSelected(m_trackerList.trackers, m_trackerList.count, bCanBeTracked && bIsTracking);
		} break;
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		{           
			bool bIsTracking= false;
			bool bCanBeTracked= PSM_GetIsControllerTracking(m_controllerView->ControllerID, &bIsTracking) == PSMResult_Success;

            // Show the controller with filtered pose
            if (bCanBeTracked && bIsTracking &&
                m_bLastMulticamPositionValid && m_bLastMulticamOrientationValid)
            {
                drawController(m_controllerView, controllerWorldTransform);
				drawTransformedAxes(controllerWorldTransform, 10.f);
            }

			drawTransformedAxes(glm::mat4(1.f), 200.f);
			drawTransformeGrid(glm::mat4(1.f), 250.f);
			drawTrackerListSelected(m_trackerList.trackers, m_trackerList.count, bCanBeTracked && bIsTracking);

        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_OpticalRecenter::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "Optical Playspace Recenter";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eCalibrationMenuState::pendingTrackerListRequest:
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for server response...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::failedTrackerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed server request!");

            if (ImGui::Button(" OK "))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
	case eCalibrationMenuState::waitForStablePosition:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 175));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Please place the controller on the ground to sample the new playspace center location.");
		ImGui::Text("Click continue to begin the next step.");

		if (ImGui::Button("Continue"))
		{
			setState(eCalibrationMenuState::measureOpticalPosition);
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel"))
		{
			request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;
	case eCalibrationMenuState::measureOpticalPosition:
	{
		bool bIsTracking = false;
		bool bCanBeTracked = PSM_GetIsControllerTracking(m_controllerView->ControllerID, &bIsTracking) == PSMResult_Success;

		// Show the controller with filtered pose
		if (bCanBeTracked && bIsTracking &&
			m_bLastMulticamPositionValid)
		{
			m_CenterSample = m_lastMulticamPositionCm;
			setState(eCalibrationMenuState::waitForStableOrientation);
		}
		else
		{
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_panel_width, 175));
			ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Unalbe to find controller. Make sure the controller is visible to the trackers.");

			if (ImGui::Button("Cancel"))
			{
				request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}

			ImGui::End();
		}

	} break;
	case eCalibrationMenuState::waitForStableOrientation:
	{
		ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
		ImGui::SetNextWindowSize(ImVec2(k_panel_width, 175));
		ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Text("Please place the controller on the ground away from the previous position,");
		ImGui::Text("to sample the new playspace forward orientation.");
		ImGui::Text("Click continue to begin the next step.");

		if (ImGui::Button("Continue"))
		{
			setState(eCalibrationMenuState::measureOpticalOrientation);
		}
		ImGui::SameLine();
		if (ImGui::Button("Cancel"))
		{
			request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
		}

		ImGui::End();
	} break;
	case eCalibrationMenuState::measureOpticalOrientation:
	{

		bool bIsTracking = false;
		bool bCanBeTracked = PSM_GetIsControllerTracking(m_controllerView->ControllerID, &bIsTracking) == PSMResult_Success;

		// Show the controller with filtered pose
		if (bCanBeTracked && bIsTracking &&
			m_bLastMulticamPositionValid)
		{
			m_ForwardSample = m_lastMulticamPositionCm;
			setState(eCalibrationMenuState::measureComplete);
		}
		else
		{
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_panel_width, 175));
			ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Unalbe to find controller. Make sure the controller is visible to the trackers.");

			if (ImGui::Button("Cancel"))
			{
				request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}

			ImGui::End();
		}

	} break;
    case eCalibrationMenuState::measureComplete:
        {
			static const float k_height_to_psmove_bulb_center = 17.7f; // cm - measured base to bulb center distance
		
			// Calc angle
			Eigen::Vector3f forwardSample = Eigen::Vector3f(m_CenterSample.x, 0.f, m_CenterSample.z) - Eigen::Vector3f(m_ForwardSample.x, 0.f, m_ForwardSample.z);
			Eigen::Vector3f forwardSample_norm = forwardSample;
			forwardSample_norm.normalize();

			Eigen::Vector3f target(0.0, 0.0, 1.0);
			Eigen::Quaternionf forwardQuat = Eigen::Quaternionf::Identity();
			forwardQuat.setFromTwoVectors(forwardSample_norm, target);

			Eigen::EulerAnglesf forwardAng = eigen_quaternionf_to_euler_angles(forwardQuat);
			
			// Calc Position
			//const Eigen::Quaternionf offset_yaw = eigen_quaternion_angle_axis(-forwardAng.get_y_degrees() * k_degrees_to_radians, Eigen::Vector3f::UnitY());

			//Eigen::Vector3f offsetSample = Eigen::Vector3f(m_CenterSample.x, 0.f, m_CenterSample.z);
			//offsetSample = eigen_vector3f_clockwise_rotate(offset_yaw, offsetSample);



			m_tracker_settings->setPlayspaceOffsets(
				-forwardAng.get_y_degrees(),
				-m_CenterSample.x,
				-k_height_to_psmove_bulb_center,
				-m_CenterSample.z
			);

			AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, m_iControllerId, -1);
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 160));
            ImGui::Begin("Test Optical Playspace Recenter", nullptr, window_flags);

			PSMQuatf orientation;
			if (PSM_GetControllerOrientation(m_controllerView->ControllerID, &orientation) == PSMResult_Success)
			{
				const Eigen::Quaternionf eigen_quat = psm_quatf_to_eigen_quaternionf(orientation);
				const Eigen::EulerAnglesf euler_angles = eigen_quaternionf_to_euler_angles(eigen_quat);

				ImGui::Text("Attitude: %.2f, Heading: %.2f, Bank: %.2f", 
					euler_angles.get_attitude_degrees(), euler_angles.get_heading_degrees(), euler_angles.get_bank_degrees());
			}

            if (ImGui::Button(" OK "))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
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
void AppStage_OpticalRecenter::setState(eCalibrationMenuState newState)
{
	if (newState != m_menuState)
	{
		onExitState(m_menuState);
		onEnterState(newState);

		m_menuState = newState;
	}
}

void AppStage_OpticalRecenter::onExitState(eCalibrationMenuState newState)
{
	switch (m_menuState)
	{
	case eCalibrationMenuState::inactive:
	case eCalibrationMenuState::pendingTrackerListRequest:
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::waitForStablePosition:
	case eCalibrationMenuState::measureOpticalPosition:
	case eCalibrationMenuState::waitForStableOrientation:
	case eCalibrationMenuState::measureOpticalOrientation:
	case eCalibrationMenuState::measureComplete:
	case eCalibrationMenuState::test:
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_OpticalRecenter::onEnterState(eCalibrationMenuState newState)
{
	switch (newState)
	{
	case eCalibrationMenuState::inactive:
	case eCalibrationMenuState::pendingTrackerListRequest:
	case eCalibrationMenuState::failedTrackerListRequest:
	case eCalibrationMenuState::waitingForStreamStartResponse:
	case eCalibrationMenuState::failedStreamStart:
	case eCalibrationMenuState::waitForStablePosition:
	case eCalibrationMenuState::measureOpticalPosition:
	case eCalibrationMenuState::waitForStableOrientation:
	case eCalibrationMenuState::measureOpticalOrientation:
	case eCalibrationMenuState::measureComplete:
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
		}
		break;
	default:
		assert(0 && "unreachable");
	}
}

void AppStage_OpticalRecenter::request_tracker_list()
{
	if (m_menuState != eCalibrationMenuState::pendingTrackerListRequest)
	{
	 	setState(eCalibrationMenuState::pendingTrackerListRequest);

		// Tell the psmove service that we we want a list of trackers connected to this machine
		PSMRequestID requestId;
		PSM_GetTrackerListAsync(&requestId);
		PSM_RegisterCallback(requestId, AppStage_OpticalRecenter::handle_tracker_list_response, this);
	}
}

void AppStage_OpticalRecenter::handle_tracker_list_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_OpticalRecenter *thisPtr = static_cast<AppStage_OpticalRecenter *>(userdata);

	switch (response_message->result_code)
	{
	case PSMResult_Success:
		{
			assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
			assert(!thisPtr->m_isControllerStreamActive);

			// Save the controller list state (used in rendering)
			thisPtr->m_trackerList = response_message->payload.tracker_list;

			// Start streaming in controller data
            {
			    PSMRequestID request_id;

			    PSM_StartControllerDataStreamAsync(thisPtr->m_controllerView->ControllerID, PSMStreamFlags_includePositionData | PSMStreamFlags_includeRawTrackerData, &request_id);
			    PSM_RegisterCallback(request_id, &AppStage_OpticalRecenter::handle_acquire_controller, thisPtr);
            }

			thisPtr->setState(eCalibrationMenuState::waitingForStreamStartResponse);
		} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
		{
			thisPtr->setState(eCalibrationMenuState::failedTrackerListRequest);
		} break;
	}
}

void AppStage_OpticalRecenter::handle_acquire_controller(
    const PSMResponseMessage *response,
    void *userdata)
{
	AppStage_OpticalRecenter *thisPtr = reinterpret_cast<AppStage_OpticalRecenter *>(userdata);

    if (response->result_code == PSMResult_Success)
    {
        thisPtr->m_isControllerStreamActive = true;
        thisPtr->m_lastControllerSeqNum = -1;
        // Wait for the first controller packet to show up...
        
		// Start off getting getting projection data from selected tracker
		{
			PSMRequestID requestId;
			PSM_SetControllerDataStreamTrackerIndexAsync(thisPtr->m_controllerView->ControllerID, 0, &requestId);
			PSM_EatResponse(requestId);
		}

    }
    else
    {
        thisPtr->setState(AppStage_OpticalRecenter::failedStreamStart);
    }
}

void AppStage_OpticalRecenter::request_exit_to_app_stage(const char *app_stage_name)
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
    case PSMController_Virtual:
        drawVirtualControllerModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}
void AppStage_OpticalRecenter::drawTrackerListSelected(const PSMClientTrackerInfo *trackerList, const int trackerCount, const bool bIsTracking)
{
	glm::mat4 psmove_tracking_space_to_chaperone_space = glm::mat4(1.f);

	// Draw the frustum for each tracking camera.
	// The frustums are defined in PSMove tracking space.
	// We need to transform them into chaperone space to display them along side the HMD.
	for (int tracker_index = 0; tracker_index < trackerCount; ++tracker_index)
	{
		const PSMClientTrackerInfo *trackerInfo = &trackerList[tracker_index];
		const PSMPosef tracker_pose = trackerInfo->tracker_pose;
		const glm::mat4 chaperoneSpaceTransform = psm_posef_to_glm_mat4(tracker_pose);

		PSMFrustum frustum;

		PSM_FrustumSetPose(&frustum, &tracker_pose);

		// Convert the FOV angles to radians for rendering purposes
		frustum.HFOV = trackerInfo->tracker_hfov * k_degrees_to_radians;
		frustum.VFOV = trackerInfo->tracker_vfov * k_degrees_to_radians;
		frustum.zNear = trackerInfo->tracker_znear;
		frustum.zFar = trackerInfo->tracker_zfar;

		drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(tracker_pose.Position), "#%d", tracker_index);

		if (bIsTracking)
		{
			drawTransformedFrustum(psmove_tracking_space_to_chaperone_space, &frustum, k_psmove_frustum_color);
		}
		else
		{
			drawTransformedFrustum(psmove_tracking_space_to_chaperone_space, &frustum, k_psmove_frustum_color_no_track);
		}
		drawTransformedAxes(chaperoneSpaceTransform, 20.f);
	}
}

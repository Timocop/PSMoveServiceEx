//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppSubStage_CalibrateWithMat.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "GeometryUtility.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"
#include "SharedTrackerState.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>
#include <sstream>

const float k_magnetic_bad = 0.2f;
const float k_magnetic_warn = 0.1f;

//-- statics ----
const char *AppStage_ComputeTrackerPoses::APP_STAGE_NAME = "ComputeTrackerPoses";

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static void drawController(const PSMController *controllerView, const glm::mat4 &transform, const PSMTrackingColorType trackingColorType);
static void drawHMD(const PSMHeadMountedDisplay *hmdView, const glm::mat4 &transform, const PSMTrackingColorType trackingColorType);

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_pendingTrackerStartCount(0)
    , m_pendingControllerStartCount(0)
	, m_areAllControllerStreamsActive(false)
	, m_lastControllerSeqNum(-1)
    , m_renderTrackerIndex(0)
    , m_pCalibrateWithMat(new AppSubStage_CalibrateWithMat(this))
    , m_bSkipCalibration(false)
    , m_ShowTrackerVideoId(-1)
    , m_overrideControllerId(-1)
    , m_overrideHmdId(-1)
	, m_triangTargetTrackerId(-1)
	, m_triangPendingTrackerDataIndexChange(false)
	, m_triangSelectedTracker(-1)
	, m_triangShowArrows(true)
	, m_triangShowControllers(false)
	, m_triangShowFrustum(true)
	, m_triangShowTrackerIds(false)
	, m_triangShowBounds(true)
	, m_triangCenter(false)
	, m_hideGoodSamples(false)
	, m_showTrackerFrustum(true)
	, m_currentMag(0.f)
{ 
    m_renderTrackerIter = m_trackerViews.end();
	m_triangInfo.clear();
	m_magneticSamples.clear();
}

AppStage_ComputeTrackerPoses::~AppStage_ComputeTrackerPoses()
{
	if (m_pCalibrateWithMat != nullptr)
	{
		delete m_pCalibrateWithMat;
		m_pCalibrateWithMat = nullptr;
	}
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithController(App *app, PSMControllerID reqeusted_controller_id)
{
    AppStage_ComputeTrackerPoses *appStage= app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = false;
    appStage->m_overrideControllerId = reqeusted_controller_id;
    appStage->m_overrideHmdId = -1;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithHMD(class App *app, PSMHmdID reqeusted_hmd_id)
{
    AppStage_ComputeTrackerPoses *appStage = app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = false;
    appStage->m_overrideControllerId = -1;
    appStage->m_overrideHmdId = reqeusted_hmd_id;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(App *app, PSMControllerID reqeusted_controller_id, PSMHmdID requested_hmd_id)
{
    AppStage_ComputeTrackerPoses *appStage = app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = true;
    appStage->m_overrideControllerId = reqeusted_controller_id;
    appStage->m_overrideHmdId = requested_hmd_id;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enter()
{
	m_pendingTrackerStartCount = 0;
	m_pendingControllerStartCount = 0;
	m_areAllControllerStreamsActive = false;
	m_lastControllerSeqNum = -1;
	
	m_showTrackerFrustum = true;
	m_currentMag = 0.f;

	m_magneticSamples.clear();

	// Only get the controller list if
    // A) No specific controller or HMD was requests
    // B) A specific controller was requested
    if ((m_overrideControllerId == -1 && m_overrideHmdId == -1) || 
        m_overrideControllerId != -1)
    {
        // Kick off this async request chain:
        // controller list request
        // -> controller start request
        // -> hmd list request (if no specific controller was requested)
        // -> hmd start request (if no specific controller was requested)
        // -> tracker list request
        // -> tracker start request
        request_controller_list();
    }
    else
    {
        // Kick off this async request chain 
        // hmd list request
        // -> hmd start request
        // -> tracker list request
        // -> tracker start request
        request_hmd_list();
    }

    m_app->setCameraType(_cameraFixed);
	m_app->getFixedCamera()->resetOrientation();
}

void AppStage_ComputeTrackerPoses::exit()
{
    release_devices();

    setState(eMenuState::inactive);
}

void AppStage_ComputeTrackerPoses::update()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
	case eMenuState::failedTrackerStartRequest:
	case eMenuState::failedControllerMagnetometer:
	case eMenuState::failedControllerUnsupported:
	case eMenuState::failedControllerOffsets:
        break;
    case eMenuState::verifyTrackers:
        update_tracker_video();
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->update();

            if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepSuccess)
            {
                //setState(AppStage_ComputeTrackerPoses::eMenuState::testTracking);

				// Refresh controller/hmd list for changed offsets due to mat calibration
				if (m_overrideControllerId != -1)
				{
					m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestControllerTracking(true);
					request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
				}

				if (m_overrideHmdId != -1)
				{
					m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestHMDTracking(true);
					request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
				}
            }
            else if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepFailed)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::calibrateStepFailed);
            }
        }
        break;
	case eMenuState::testMagnetic:
	{
		t_controller_state_map_iterator m_controllerState = m_controllerViews.find(m_overrideControllerId);

		if (m_controllerState == m_controllerViews.end())
		{
			setState(AppStage_ComputeTrackerPoses::failedControllerOffsets);
		}
		else
		{
			PSMController *controllerView = m_controllerState->second.controllerView;
			if (controllerView == nullptr || !controllerView->bValid || controllerView->ControllerType != PSMControllerType::PSMController_Move)
			{
				setState(AppStage_ComputeTrackerPoses::failedControllerUnsupported);
			}
			else
			{
				if (m_areAllControllerStreamsActive && controllerView->OutputSequenceNum != m_lastControllerSeqNum)
				{
					m_lastControllerSeqNum = controllerView->OutputSequenceNum;

					PSMVector3i raw_mag = controllerView->ControllerState.PSMoveState.RawSensorData.Magnetometer;
					if (raw_mag.x == 0 && raw_mag.y == 0 && raw_mag.z == 0)
					{
						setState(AppStage_ComputeTrackerPoses::failedControllerMagnetometer);
					}
					else
					{
						PSMVector3f raw_mag = controllerView->ControllerState.PSMoveState.CalibratedSensorData.Magnetometer;
						if (is_nearly_zero(raw_mag.x) && is_nearly_zero(raw_mag.y) && is_nearly_zero(raw_mag.z))
						{
							setState(AppStage_ComputeTrackerPoses::failedControllerMagnetometer);
						}
						else
						{
							const float k_sample_distance = 30.f;
							const float k_sample_alpha = 0.1;

							const float raw_mag_q = fabsf(sqrtf(
								raw_mag.x * raw_mag.x +
								raw_mag.y * raw_mag.y +
								raw_mag.z * raw_mag.z
							) - 1.f);

							m_currentMag = k_sample_alpha*raw_mag_q + (1.f - k_sample_alpha)*m_currentMag;

							if (controllerView->ControllerState.PSMoveState.bIsCurrentlyTracking)
							{
								PSMVector3f position = controllerView->ControllerState.PSMoveState.Pose.Position;

								int targetIndex = -1;
								float nearestDistance = k_sample_distance;
								for (int j = m_magneticSamples.size()-1; j > -1; --j)
								{
									float dx = (position.x - m_magneticSamples[j].m_position.x);
									float dy = (position.y - m_magneticSamples[j].m_position.y);
									float dz = (position.z - m_magneticSamples[j].m_position.z);
									float sample_distance = sqrtf(dx*dx + dy*dy + dz*dz);

									if (nearestDistance < 0.f || sample_distance < nearestDistance)
									{
										nearestDistance = sample_distance;
										targetIndex = j;
									}
								}

								if (targetIndex > -1)
								{
									m_magneticSamples[targetIndex].magnetic_strength = 
										k_sample_alpha*m_currentMag + 
										(1.f - k_sample_alpha)*m_magneticSamples[targetIndex].magnetic_strength;
								}

								if (targetIndex == -1 && m_magneticSamples.size() < 1000)
								{
									MagneticInfo info;
									info.magnetic_strength = m_currentMag;
									info.m_position = position;

									m_magneticSamples.push_back(info);
								}
							}
						}
					}
				}
			}
		}
		break;
	}
	case eMenuState::testTracking:
		break;
    case eMenuState::showTrackerVideo:
        update_tracker_video();
        break;
	case eMenuState::calibrateStepFailed:
		break;
	case eMenuState::pendingControllerOffsets:
		{
			t_controller_state_map_iterator m_controllerState = m_controllerViews.find(m_overrideControllerId);
			t_tracker_state_map_iterator m_trackerState = m_trackerViews.find(m_triangTargetTrackerId);

			// Check if controller is in list
			if (m_controllerState == m_controllerViews.end() || 
				m_trackerState == m_trackerViews.end())
			{
				setState(AppStage_ComputeTrackerPoses::failedControllerOffsets);
			}
			else
			{
				PSMController *controllerView = m_controllerState->second.controllerView;
				PSMTracker *trackerView = m_trackerState->second.trackerView;
				if (controllerView == nullptr || !controllerView->bValid ||
					trackerView == nullptr)
				{
					setState(AppStage_ComputeTrackerPoses::failedControllerUnsupported);
				}
				else
				{
					if (!m_triangPendingTrackerDataIndexChange)
					{
						if(m_areAllControllerStreamsActive && controllerView->OutputSequenceNum != m_lastControllerSeqNum)
						{
							m_lastControllerSeqNum = controllerView->OutputSequenceNum;
							
							int currentTrackerId = -1;
							bool isTracked = false;
							PSMPosef pose;
							PSMVector2f screen_vector;

							switch (controllerView->ControllerType)
							{
							case PSMController_Move:
								pose = controllerView->ControllerState.PSMoveState.Pose;
								screen_vector = controllerView->ControllerState.PSMoveState.RawTrackerData.ScreenLocation;
								currentTrackerId = controllerView->ControllerState.PSMoveState.RawTrackerData.TrackerID;
								isTracked = (controllerView->ControllerState.PSMoveState.RawTrackerData.ValidTrackerBitmask &
									(1 << trackerView->tracker_info.tracker_id)) > 0;
								break;
							case PSMController_DualShock4:
								pose = controllerView->ControllerState.PSDS4State.Pose;
								screen_vector = controllerView->ControllerState.PSDS4State.RawTrackerData.ScreenLocation;
								currentTrackerId = controllerView->ControllerState.PSDS4State.RawTrackerData.TrackerID;
								isTracked = (controllerView->ControllerState.PSDS4State.RawTrackerData.ValidTrackerBitmask &
									(1 << trackerView->tracker_info.tracker_id)) > 0;
								break;
							case PSMController_Virtual:
								pose = controllerView->ControllerState.VirtualController.Pose;
								screen_vector = controllerView->ControllerState.VirtualController.RawTrackerData.ScreenLocation;
								currentTrackerId = controllerView->ControllerState.VirtualController.RawTrackerData.TrackerID;
								isTracked = (controllerView->ControllerState.VirtualController.RawTrackerData.ValidTrackerBitmask &
									(1 << trackerView->tracker_info.tracker_id)) > 0;
								break;
							}

							if (isTracked && currentTrackerId != m_triangTargetTrackerId)
							{
								request_controller_set_tracker_offset(m_overrideControllerId, m_triangTargetTrackerId);
							}
							else
							{
								if (isTracked)
								{
									m_triangLastControllerPose = pose;
									m_triangTrackerScreenLocations.insert(t_controller_screenloc_pair(currentTrackerId, screen_vector));
								}

								if (m_trackerViews.find(m_triangTargetTrackerId + 1) != m_trackerViews.end())
								{
									++m_triangTargetTrackerId;
								}
								else
								{
									unsigned int processedTrackers[PSMOVESERVICE_MAX_TRACKER_COUNT];
									m_triangInfo.clear();

									for (t_tracker_state_map_iterator tracker_iter = m_trackerViews.begin(); tracker_iter != m_trackerViews.end(); ++tracker_iter)
									{
										const PSMTracker *trackerView = tracker_iter->second.trackerView;
										const int tracker_id = trackerView->tracker_info.tracker_id;
										const PSMPosef trackerPose = trackerView->tracker_info.tracker_pose;
										const glm::mat4 trackerMat4 = psm_posef_to_glm_mat4(trackerPose);
										const PSMClientTrackerInfo trackerInfo = trackerView->tracker_info;

										cv::Matx33f tracker_inst;
										{
											cv::Matx<float, 5, 1> distortionOut;
											tracker_inst(0, 0) = trackerInfo.tracker_focal_lengths.x;
											tracker_inst(1, 1) = trackerInfo.tracker_focal_lengths.y;
											tracker_inst(0, 2) = trackerInfo.tracker_principal_point.x;
											tracker_inst(1, 2) = trackerInfo.tracker_principal_point.y;

											tracker_inst(1, 1) *= -1;  //Negate F_PY because the screen coordinate system has +Y down.

																	   // Fill the rest of the matrix with corrext values.
											tracker_inst(0, 1) = 0.f;
											tracker_inst(1, 0) = 0.f;
											tracker_inst(2, 0) = 0.f;
											tracker_inst(2, 1) = 0.f;
											tracker_inst(2, 2) = 1.f;

											distortionOut(0, 0) = trackerInfo.tracker_k1;
											distortionOut(1, 0) = trackerInfo.tracker_k2;
											distortionOut(4, 0) = trackerInfo.tracker_k3;
											distortionOut(2, 0) = trackerInfo.tracker_p1;
											distortionOut(3, 0) = trackerInfo.tracker_p2;
										}

										for (t_tracker_state_map_iterator trackerOther_iter = m_trackerViews.begin(); trackerOther_iter != m_trackerViews.end(); ++trackerOther_iter)
										{
											const PSMTracker *trackerOtherView = trackerOther_iter->second.trackerView;
											const int trackerOther_id = trackerOtherView->tracker_info.tracker_id;
											const PSMPosef trackerOtherPose = trackerOtherView->tracker_info.tracker_pose;
											const glm::mat4 trackerOtherMat4 = psm_posef_to_glm_mat4(trackerOtherPose);
											const PSMClientTrackerInfo trackerOtherInfo = trackerOtherView->tracker_info;

											if (tracker_id == trackerOther_id)
												continue;

											if ((processedTrackers[tracker_id] & (1 << trackerOther_id)) > 0)
												continue;

											processedTrackers[tracker_id] |= (1 << trackerOther_id);
											processedTrackers[trackerOther_id] |= (1 << tracker_id);

											cv::Matx33f trackerOther_inst;
											{
												cv::Matx<float, 5, 1> distortionOut;
												trackerOther_inst(0, 0) = trackerOtherInfo.tracker_focal_lengths.x;
												trackerOther_inst(1, 1) = trackerOtherInfo.tracker_focal_lengths.y;
												trackerOther_inst(0, 2) = trackerOtherInfo.tracker_principal_point.x;
												trackerOther_inst(1, 2) = trackerOtherInfo.tracker_principal_point.y;

												trackerOther_inst(1, 1) *= -1;  //Negate F_PY because the screen coordinate system has +Y down.

																				// Fill the rest of the matrix with corrext values.
												trackerOther_inst(0, 1) = 0.f;
												trackerOther_inst(1, 0) = 0.f;
												trackerOther_inst(2, 0) = 0.f;
												trackerOther_inst(2, 1) = 0.f;
												trackerOther_inst(2, 2) = 1.f;

												distortionOut(0, 0) = trackerOtherInfo.tracker_k1;
												distortionOut(1, 0) = trackerOtherInfo.tracker_k2;
												distortionOut(4, 0) = trackerOtherInfo.tracker_k3;
												distortionOut(2, 0) = trackerOtherInfo.tracker_p1;
												distortionOut(3, 0) = trackerOtherInfo.tracker_p2;
											}

											t_controller_screenloc_map_iterator tracker_point = m_triangTrackerScreenLocations.find(tracker_id);
											t_controller_screenloc_map_iterator trackerOther_point = m_triangTrackerScreenLocations.find(trackerOther_id);

											if (tracker_point != m_triangTrackerScreenLocations.end() &&
												trackerOther_point != m_triangTrackerScreenLocations.end())
											{
												cv::Point2f screen_location = cv::Point2f(tracker_point->second.x, tracker_point->second.y);
												cv::Point2f screenOther_location = cv::Point2f(trackerOther_point->second.x, trackerOther_point->second.y);

												cv::Mat projPoints1 = cv::Mat(screen_location);
												cv::Mat projPoints2 = cv::Mat(screenOther_location);

												cv::Matx34f projMat1;
												{
													const glm::quat glm_quat(trackerPose.Orientation.w, trackerPose.Orientation.x, trackerPose.Orientation.y, trackerPose.Orientation.z);
													const glm::vec3 glm_pos(trackerPose.Position.x, trackerPose.Position.y, trackerPose.Position.z);
													const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);
													const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);
													cv::Matx34f out;
													out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
													out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
													out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];

													// intrinsic matrix * extrinsic matrix
													projMat1 = cv::Mat(tracker_inst * out);
												}

												cv::Matx34f projMat2;
												{
													const glm::quat glm_quat(trackerOtherPose.Orientation.w, trackerOtherPose.Orientation.x, trackerOtherPose.Orientation.y, trackerOtherPose.Orientation.z);
													const glm::vec3 glm_pos(trackerOtherPose.Position.x, trackerOtherPose.Position.y, trackerOtherPose.Position.z);
													const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);
													const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);
													cv::Matx34f out;
													out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
													out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
													out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];

													// intrinsic matrix * extrinsic matrix
													projMat2 = cv::Mat(trackerOther_inst * out);
												}

												cv::Mat point3D(1, 1, CV_32FC4);
												cv::triangulatePoints(projMat1, projMat2, projPoints1, projPoints2, point3D);

												PSMVector3f result;
												const float w = point3D.at<float>(3, 0);
												result.x = point3D.at<float>(0, 0) / w;
												result.y = point3D.at<float>(1, 0) / w;
												result.z = point3D.at<float>(2, 0) / w;

												TriangulationInfo info;
												info.m_trackerId = tracker_id;
												info.m_trackerOtherId = trackerOther_id;
												info.m_trackerPose = trackerPose;
												info.m_trackerOtherPose = trackerOtherPose;
												info.m_point = result;
												m_triangInfo.push_back(info);
											}
										}
									}

									setState(AppStage_ComputeTrackerPoses::showControllerOffsets);
								}
							}
						}
					}
				}
			}

		}
		break;
	case eMenuState::showControllerOffsets:
		break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::render()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	case eMenuState::failedControllerOffsets:
	case eMenuState::failedControllerMagnetometer:
	case eMenuState::failedControllerUnsupported:
        break;
    case eMenuState::verifyTrackers:
        {
            render_tracker_video();
        } break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->render();
        break;

	case eMenuState::testMagnetic:
    case eMenuState::testTracking:
        {
            // Draw the chaperone origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);
			drawTransformeGrid(glm::mat4(1.0f), 250.f);

            // Draw the frustum for each tracking camera.
            // The frustums are defined in PSMove tracking space.
            // We need to transform them into chaperone space to display them along side the HMD.
            for (t_tracker_state_map_iterator tracker_iter = m_trackerViews.begin(); tracker_iter != m_trackerViews.end(); ++tracker_iter)
            {
                const PSMTracker *trackerView = tracker_iter->second.trackerView;
				const int tracker_id= trackerView->tracker_info.tracker_id;
                const PSMPosef trackerPose = trackerView->tracker_info.tracker_pose;
                const glm::mat4 trackerMat4 = psm_posef_to_glm_mat4(trackerPose);
				
                PSMFrustum frustum;
                PSM_GetTrackerFrustum(tracker_id, &frustum);

                // use color depending on tracking status
                glm::vec3 color= does_tracker_see_any_device(trackerView) ? k_psmove_frustum_color : k_psmove_frustum_color_no_track;

                drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(trackerPose.Position), "#%d", tracker_id);

				if (m_showTrackerFrustum)
					drawTransformedFrustum(glm::mat4(1.f), &frustum, color);

				drawPS3EyeModel(trackerMat4);
                drawTransformedAxes(trackerMat4, 20.f);
            }

            // Draw each controller model
            for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
            {
                const PSMController *controllerView = controller_iter->second.controllerView;
                const PSMTrackingColorType trackingColorType= controller_iter->second.trackingColorType;
				
                PSMPosef controllerPose;
                PSMPhysicsData physicsData;
                switch (controllerView->ControllerType)
                {
                case PSMControllerType::PSMController_Move:
                    controllerPose = controllerView->ControllerState.PSMoveState.Pose;
                    physicsData= controllerView->ControllerState.PSMoveState.PhysicsData;
                    break;
                case PSMControllerType::PSMController_DualShock4:
                    controllerPose = controllerView->ControllerState.PSDS4State.Pose;
                    physicsData= controllerView->ControllerState.PSDS4State.PhysicsData;
                    break;
                case PSMControllerType::PSMController_Virtual:
                    controllerPose = controllerView->ControllerState.VirtualController.Pose;
                    physicsData= controllerView->ControllerState.VirtualController.PhysicsData;
                    break;
                }
                glm::mat4 controllerMat4 = psm_posef_to_glm_mat4(controllerPose);

                if (m_controllerViews.size() > 1)
                {
                    drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(controllerPose.Position), "#%d", controllerView->ControllerID);
                }
                drawController(controllerView, controllerMat4, trackingColorType);
                drawTransformedAxes(controllerMat4, 10.f);

                // Draw the acceleration and velocity arrows
                {
                    const glm::mat4 originMat4= glm::translate(glm::mat4(1.f), psm_vector3f_to_glm_vec3(controllerPose.Position));
                    const glm::vec3 vel_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearVelocityCmPerSec);
                    const glm::vec3 acc_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearAccelerationCmPerSecSqr)*PSM_CENTIMETERS_TO_METERS;
                    
                    const float vel= glm::length(vel_endpoint);
                    if (vel > k_positional_epsilon)
                    {
                        drawArrow(originMat4, glm::vec3(0.f), vel_endpoint, 0.1f, glm::vec3(0.f, 1.f, 1.f));
                        //drawTextAtWorldPosition(originMat4, vel_endpoint, "v=%.2fcm/s", vel);
                    }

                    const float acc = glm::length(acc_endpoint);
                    if (acc > k_positional_epsilon)
                    {
                        drawArrow(originMat4, glm::vec3(0.f), acc_endpoint, 0.1f, glm::vec3(1.f, 1.f, 0.f));
                        //drawTextAtWorldPosition(originMat4, acc_endpoint, "a=%.2fm/s^2", acc);
                    }
                }
            }

            // Draw each HMD model
            for (t_hmd_state_map_iterator hmd_iter = m_hmdViews.begin(); hmd_iter != m_hmdViews.end(); ++hmd_iter)
            {
                const PSMHeadMountedDisplay *hmdView = hmd_iter->second.hmdView;
                const PSMTrackingColorType trackingColorType= hmd_iter->second.trackingColorType;

				PSMPosef hmdPose;
				PSMPhysicsData physicsData;
				switch (hmdView->HmdType)
				{
				case PSMHmdType::PSMHmd_Morpheus:
					hmdPose = hmdView->HmdState.MorpheusState.Pose;
					physicsData = hmdView->HmdState.MorpheusState.PhysicsData;
					break;
				case PSMHmdType::PSMHmd_Virtual:
					hmdPose = hmdView->HmdState.VirtualHMDState.Pose;
					physicsData = hmdView->HmdState.VirtualHMDState.PhysicsData;
					break;
				}
                glm::mat4 hmdMat4 = psm_posef_to_glm_mat4(hmdPose);

                if (m_hmdViews.size() > 1)
                {
                    drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(hmdPose.Position), "#%d", hmdView->HmdID);
                }

                drawHMD(hmdView, hmdMat4, trackingColorType);
                drawTransformedAxes(hmdMat4, 10.f);

				// Draw the acceleration and velocity arrows
				{
					const glm::mat4 originMat4 = glm::translate(glm::mat4(1.f), psm_vector3f_to_glm_vec3(hmdPose.Position));
					const glm::vec3 vel_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearVelocityCmPerSec);
					const glm::vec3 acc_endpoint = psm_vector3f_to_glm_vec3(physicsData.LinearAccelerationCmPerSecSqr)*PSM_CENTIMETERS_TO_METERS;

					const float vel = glm::length(vel_endpoint);
					if (vel > k_positional_epsilon)
					{
						drawArrow(originMat4, glm::vec3(0.f), vel_endpoint, 0.1f, glm::vec3(0.f, 1.f, 1.f));
						//drawTextAtWorldPosition(originMat4, vel_endpoint, "v=%.2fcm/s", vel);
					}

					const float acc = glm::length(acc_endpoint);
					if (acc > k_positional_epsilon)
					{
						drawArrow(originMat4, glm::vec3(0.f), acc_endpoint, 0.1f, glm::vec3(1.f, 1.f, 0.f));
						//drawTextAtWorldPosition(originMat4, acc_endpoint, "a=%.2fm/s^2", acc);
					}
				}
            }

			if (m_menuState == eMenuState::testMagnetic && m_magneticSamples.size() > 0)
			{
				for (auto it = m_magneticSamples.begin(); it != m_magneticSamples.end(); ++it)
				{
					float mag_str = fabsf(it->magnetic_strength);

					glm::vec3 color = glm::vec3(1.f, 1.f, 1.f);

					if (mag_str > k_magnetic_bad)
					{
						color = glm::vec3(1.f, 0.f, 0.f);
					}
					else if (mag_str > k_magnetic_warn)
					{
						color = glm::vec3(1.f, 1.f, 0.f);
					}
					else 
					{
						if (m_hideGoodSamples)
							continue;

						color = glm::vec3(0.f, 1.f, 0.f);
					}
					

					drawTextAtWorldPosition(glm::mat4(1.f), glm::vec3(it->m_position.x, it->m_position.y, it->m_position.z), "%.0f", mag_str * 100.f);
					drawPointCloud(glm::mat4(1.f), color, reinterpret_cast<float *>(&it->m_position), 1);
				}
			}

        } break;
    case eMenuState::showTrackerVideo:
        {
            render_tracker_video();
        } break;
	case eMenuState::calibrateStepFailed:
		break;
	case eMenuState::pendingControllerOffsets:
		break;
	case eMenuState::showControllerOffsets:
	{
		// Draw the chaperone origin axes
		drawTransformedAxes(glm::mat4(1.0f), 100.f);
		drawTransformeGrid(glm::mat4(1.0f), 250.f);

		// Get avergage center and recenter whole playspace
		PSMVector3f center_point;
		center_point.x = 0.f;
		center_point.y = 0.f;
		center_point.z = 0.f;

		if (m_triangCenter)
		{
			int N = 0;

			for (auto it = m_triangInfo.begin(); it != m_triangInfo.end(); ++it)
			{
				center_point.x += it->m_point.x;
				center_point.y += it->m_point.y;
				center_point.z += it->m_point.z;
				++N;
			}

			if (N > 0)
			{
				center_point.x /= N;
				center_point.y /= N;
				center_point.z /= N;
			}
		}

		// Draw the frustum for each tracking camera.
		// The frustums are defined in PSMove tracking space.
		// We need to transform them into chaperone space to display them along side the HMD.
		for (t_tracker_state_map_iterator tracker_iter = m_trackerViews.begin(); tracker_iter != m_trackerViews.end(); ++tracker_iter)
		{
			const PSMTracker *trackerView = tracker_iter->second.trackerView;
			const int tracker_id = trackerView->tracker_info.tracker_id;

			PSMPosef trackerPose = trackerView->tracker_info.tracker_pose;
			trackerPose.Position.x -= center_point.x;
			trackerPose.Position.y -= center_point.y;
			trackerPose.Position.z -= center_point.z;

			const glm::mat4 trackerMat4 = psm_posef_to_glm_mat4(trackerPose);
			
			PSMFrustum frustum;
			PSM_GetTrackerFrustum(tracker_id, &frustum);
			frustum.origin.x -= center_point.x;
			frustum.origin.y -= center_point.y;
			frustum.origin.z -= center_point.z;

			// use color depending on tracking status
			glm::vec3 color;
			if (m_triangTrackerScreenLocations.find(tracker_id) != m_triangTrackerScreenLocations.end())
			{
				color = k_psmove_frustum_color;
			}
			else
			{
				color = k_psmove_frustum_color_no_track;
			}

			drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(trackerPose.Position), "#%d", tracker_id);
			
			if (m_triangShowFrustum)
			{
				drawTransformedFrustum(glm::mat4(1.f), &frustum, color);
			}

			drawPS3EyeModel(trackerMat4);
			drawTransformedAxes(trackerMat4, 20.f);

			
		}

		PSMPosef m_newTriangLastControllerPose = m_triangLastControllerPose;
		m_newTriangLastControllerPose.Position.x -= center_point.x;
		m_newTriangLastControllerPose.Position.y -= center_point.y;
		m_newTriangLastControllerPose.Position.z -= center_point.z;

		// Draw each controller model
		t_controller_state_map_iterator controllerState = m_controllerViews.find(m_overrideControllerId);
		if (controllerState != m_controllerViews.end())
		{
			const PSMController *controllerView = controllerState->second.controllerView;
			if (controllerView != nullptr && controllerView->bValid)
			{
				{
					const PSMTrackingColorType trackingColorType = controllerState->second.trackingColorType;

					glm::mat4 controllerMat4 = psm_posef_to_glm_mat4(m_newTriangLastControllerPose);

					if (m_controllerViews.size() > 1)
					{
						drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(m_newTriangLastControllerPose.Position), "#%d", controllerView->ControllerID);
					}

					if (m_triangShowControllers)
					{
						drawController(controllerView, controllerMat4, trackingColorType);
						drawTransformedAxes(controllerMat4, 10.f);
					}
					else
					{
						drawPointCloud(glm::mat4(1.f), glm::vec3(1.f, 0.f, 0.f), reinterpret_cast<float *>(&m_newTriangLastControllerPose.Position), 1);
					}
				}

				for (auto it = m_triangInfo.begin(); it != m_triangInfo.end(); ++it)
				{
					if (m_triangSelectedTracker != -1 && m_triangSelectedTracker != it->m_trackerId && m_triangSelectedTracker != it->m_trackerOtherId)
						continue;

					PSMPosef offset_pose;
					offset_pose.Position.x = it->m_point.x - center_point.x;
					offset_pose.Position.y = it->m_point.y - center_point.y;
					offset_pose.Position.z = it->m_point.z - center_point.z;
					offset_pose.Orientation = m_newTriangLastControllerPose.Orientation;

					PSMPosef trackerPose = it->m_trackerPose;
					PSMPosef trackerOtherPose = it->m_trackerOtherPose;
					trackerPose.Position.x -= center_point.x;
					trackerPose.Position.y -= center_point.y;
					trackerPose.Position.z -= center_point.z;
					trackerOtherPose.Position.x -= center_point.x;
					trackerOtherPose.Position.y -= center_point.y;
					trackerOtherPose.Position.z -= center_point.z;

					glm::mat4 controllerMat4 = psm_posef_to_glm_mat4(offset_pose);

					if (m_triangShowControllers)
					{
						drawController(controllerView, controllerMat4, PSMTrackingColorType::PSMTrackingColorType_MaxColorTypes);
						drawTransformedAxes(controllerMat4, 10.f);
					}
					else
					{
						drawPointCloud(glm::mat4(1.f), glm::vec3(1.f, 1.f, 1.f), reinterpret_cast<float *>(&offset_pose.Position), 1);
					}

					if (m_triangShowTrackerIds)
					{
						drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(offset_pose.Position), "#%d+#%d", it->m_trackerId, it->m_trackerOtherId);
					}

					if (m_triangShowArrows)
					{
						drawArrow(glm::mat4(1.f), psm_vector3f_to_glm_vec3(trackerPose.Position), psm_vector3f_to_glm_vec3(offset_pose.Position), 0.1f, glm::vec3(1.f, 1.f, 1.f));
						drawArrow(glm::mat4(1.f), psm_vector3f_to_glm_vec3(trackerOtherPose.Position), psm_vector3f_to_glm_vec3(offset_pose.Position), 0.1f, glm::vec3(1.f, 1.f, 1.f));
					}
				}
			}
		}

		if (m_triangShowBounds && m_triangInfo.size() > 0)
		{
			PSMVector3f box_min;
			PSMVector3f box_max;
			PSMVector3f bounds_size;
			get_points_bounds(box_min, box_max, bounds_size);
			box_min.x -= center_point.x;
			box_min.y -= center_point.y;
			box_min.z -= center_point.z;
			box_max.x -= center_point.x;
			box_max.y -= center_point.y;
			box_max.z -= center_point.z;

			drawTransformedBox(glm::mat4(1.f), psm_vector3f_to_glm_vec3(box_min), psm_vector3f_to_glm_vec3(box_max), glm::vec3(1.f, 1.f, 1.f));
			drawTextAtWorldPosition(glm::mat4(1.f), psm_vector3f_to_glm_vec3(box_max), " %.2f, %.2f, %.2f cm", bounds_size.x, bounds_size.y, bounds_size.z);
		}

	} break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::renderUI()
{
	static float waitCount;

    const float k_panel_width = 300.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;

    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        {
            ImGui::SetNextWindowPos(ImVec2(
				ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 
				20.f)
			);
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
            ImGui::Text("Pending device initialization...");

			ImGui::Separator();
            if (ImGui::Button("      Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
            ImGui::End();
        } break;

    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	case eMenuState::failedControllerOffsets:
	case eMenuState::failedControllerMagnetometer:
	case eMenuState::failedControllerUnsupported:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            switch (m_menuState)
            {
            case eMenuState::failedControllerListRequest:
                ImGui::TextWrapped("Failed controller list retrieval!");
                break;
            case eMenuState::failedControllerStartRequest:
                ImGui::TextWrapped("Failed controller stream start!");
                break;
            case eMenuState::failedHmdListRequest:
                ImGui::TextWrapped("Failed HMD list retrieval!");
                break;
            case eMenuState::failedHmdStartRequest:
                ImGui::TextWrapped("Failed HMD stream start!");
                break;
            case eMenuState::failedTrackerListRequest:
                ImGui::TextWrapped("Failed tracker list retrieval!");
                break;
			case eMenuState::failedTrackerStartRequest:
				ImGui::TextWrapped("Failed tracker stream start!");
				break;
			case eMenuState::failedControllerOffsets:
				ImGui::TextWrapped("Failed tracker data stream start!");
				break;
			case eMenuState::failedControllerMagnetometer:
				ImGui::TextWrapped("Failed to read controller magnetometer data!");
				break;
			case eMenuState::failedControllerUnsupported:
				ImGui::TextWrapped("Controller does not support this action!");
				break;
            }

			ImGui::Separator();
            if (ImGui::Button("      OK"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

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

    case eMenuState::verifyTrackers:
        {
            ImGui::SetNextWindowPos(ImVec2(
				ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 
				20.f)
			);
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Verify that your tracking cameras can see the tracking origin.");
			ImGui::Text("Calibration mat paper format:");

			std::string paperFormats;
			for (int i = 0; i < AppSubStage_CalibrateWithMat::ePaperFormat::MAX_PAPER_FORMATS; i++)
			{
				paperFormats += AppSubStage_CalibrateWithMat::k_paper_formats_names[i];
				paperFormats += '\0';
			}
			paperFormats += '\0';

			int paperFormat = static_cast<int>(m_pCalibrateWithMat->m_iPaperFormat);
			if (ImGui::Combo("##CaliMatFormat", &paperFormat, paperFormats.c_str()))
			{
				assert(paperFormat > AppSubStage_CalibrateWithMat::ePaperFormat::MIN_PAPER_FORMATS && paperFormat < AppSubStage_CalibrateWithMat::ePaperFormat::MAX_PAPER_FORMATS);
				m_pCalibrateWithMat->m_iPaperFormat = static_cast<AppSubStage_CalibrateWithMat::ePaperFormat>(paperFormat);
			}

            ImGui::Separator();

            if (m_trackerViews.size() > 1)
            {
				if (ImGui::Button(" < ##Previous Tracker"))
                {
                    go_previous_tracker();
                }
                ImGui::SameLine();
				if (ImGui::Button(" > ##Next Tracker"))
                {
                    go_next_tracker();
                }
				ImGui::SameLine();
				ImGui::Text("Tracker #%d", m_renderTrackerIndex);
            }

			ImGui::Separator();
            if (ImGui::Button("      Start Calibration"))
            {
                setState(eMenuState::calibrateWithMat);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconCheck()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SameLine();

            if (ImGui::Button("      Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(500, 0));
            ImGui::End();
        }

        break;

    case eMenuState::selectCalibrationMethod:
        {
            ImGui::SetNextWindowPos(ImVec2(
				ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f,
				20.f)
			);
            ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(32, 32));
			ImGui::SameLine();
            ImGui::Text("Select a calibration method");
            ImGui::Separator();

            if (ImGui::Button("      Calibration Mat"))
            {
                setState(eMenuState::calibrateWithMat);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTarget()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(500, 0));
            ImGui::End();
        } break;

    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->renderUI();
        } break;

	case eMenuState::testMagnetic:
		{
			ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
			ImGui::Begin("Test Magnetic Interferences", nullptr, window_flags);

			ImGui::PushTextWrapPos();
			ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), ImColor(0.f, .5f, 1.f));
			ImGui::SameLine();
			ImGui::TextColored(ImColor(0.f, .5f, 1.f),
				"This preview uses the device's magnetometer. Ensure that the magnetometer of this device is properly calibrated!");
			ImGui::PopTextWrapPos();
			ImGui::Separator();

			int sample_count = 0;
			float samples_avg = 0.f;

			for (auto it = m_magneticSamples.begin(); it != m_magneticSamples.end(); ++it)
			{
				float mag_str = fabsf(it->magnetic_strength);

				samples_avg += mag_str;
				++sample_count;
			}

			ImGui::Text("Overall Magnetic Quality:");
			if (sample_count > 0)
			{
				float percent = (samples_avg / sample_count) / k_magnetic_bad;
				ImGui::ProgressBar(1.f - percent, ImVec2(-1, 0), " ");
			}
			else
			{
				ImGui::ProgressBar(0.f, ImVec2(-1, 0), " ");
			}

			ImGui::Checkbox("Show Interferences Only", &m_hideGoodSamples);
			ImGui::Checkbox("Show Tracker Frustum", &m_showTrackerFrustum);

			if (ImGui::Button("      Reset Samples"))
			{
				m_magneticSamples.clear();
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconUpdate()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::Separator();

			if (ImGui::Button("      Return to Test Tracking Pose"))
			{
				setState(eMenuState::testTracking);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(250, 0));
			ImGui::End();

			break;
		}
	case eMenuState::testTracking:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::Begin("Test Tracking Pose", nullptr, window_flags);

            // display per tracker UI
            for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
            {
                const PSMTracker *trackerView = iter->second.trackerView;

                ImGui::PushItemWidth(125.f);
                if (does_tracker_see_any_device(trackerView))
                {
                    ImGui::Text("Tracker #%d: OK  ", trackerView->tracker_info.tracker_id);
                }
                else 
                {
                    ImGui::Text("Tracker #%d: FAIL", trackerView->tracker_info.tracker_id);
                }
                ImGui::PopItemWidth();

                ImGui::SameLine();

                ImGui::PushItemWidth(100.f);
                ImGui::PushID(trackerView->tracker_info.tracker_id);
                if (m_app->getIsLocalServer())
                {
                    if (ImGui::Button("      Tracker Video") || trackerView->tracker_info.tracker_id == m_ShowTrackerVideoId)
                    {
                        m_ShowTrackerVideoId = -1;
                        m_renderTrackerIter = iter;
                        setState(eMenuState::showTrackerVideo);
                    }
					ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTracker()->getImTextureId(),
						ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
						ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
                }
                else
                {
                    ImGui::Button("      Tracker Video (Unavailable)");
					ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
						ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
						ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
                }
                ImGui::PopID();
                ImGui::PopItemWidth();
            }

			ImGui::Separator();

			if (m_controllerViews.size() > 0 && m_overrideControllerId != -1)
			{
				ImGui::Text("Controller ID: #%d", m_overrideControllerId);
				if (ImGui::Button("      Show Tracker Triangulations"))
				{
					setState(eMenuState::pendingControllerOffsets);
				}
				ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTarget()->getImTextureId(),
					ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
					ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
				
				if (ImGui::Button("      Show Magnetic Interferences"))
				{
					m_magneticSamples.clear();
					setState(eMenuState::testMagnetic);
				}
				ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconSearch()->getImTextureId(),
					ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
					ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

				ImGui::Separator();
			}

            if (!m_bSkipCalibration)
            {
                ImGui::Text("Calibration Complete");

                if (ImGui::Button("      Redo Calibration"))
                {
                    setState(eMenuState::verifyTrackers);
                }
				ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconUpdate()->getImTextureId(),
					ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
					ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
            }

            if (ImGui::Button("      Return to Tracker Settings"))
            {
				request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(250, 0));
            ImGui::End();
        }
        break;

    case eMenuState::showTrackerVideo:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::Begin("Test Tracking Video", nullptr, window_flags);

            //ImGui::Text("Tracker ID: #%d", m_renderTrackerIter->second.trackerView->tracker_info.tracker_id);

            if (m_trackerViews.size() > 1)
            {
                if (ImGui::Button(" < ##Previous Tracker"))
                {
                    go_previous_tracker();
                }
				ImGui::SameLine();
				if (ImGui::Button(" > ##Next Tracker"))
				{
					go_next_tracker();
				}
                ImGui::SameLine();
                int TrackerID = m_renderTrackerIter->second.trackerView->tracker_info.tracker_id;
                ImGui::Text("Tracker ID: #%d", TrackerID);
                m_app->getAppStage<AppStage_TrackerSettings>()->set_selectedTrackerIndex(TrackerID);
            } 
            else {
                ImGui::Text("Tracker ID: 0");
            }
            
            if (ImGui::Button("      Calibrate Tracking Colors"))
            {
                if (m_overrideHmdId != -1)
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoHMDColorCalib(true);
                }
                else
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoControllerColorCalib(true);
                }
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTracker()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

            if (ImGui::Button("      Test Tracking Pose"))
            {
                setState(eMenuState::testTracking);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTarget()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::Separator();

            if (ImGui::Button("      Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(250, 0));
            ImGui::End();
        }
        break;

		case eMenuState::calibrateStepFailed:
		{
			ImGui::SetNextWindowPosCenter();
			ImGui::Begin(k_window_title, nullptr, window_flags);

			ImGui::Text("Calibration Failed");

			if (ImGui::Button("      Restart Calibration"))
			{
				setState(eMenuState::verifyTrackers);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconUpdate()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			if (ImGui::Button("      Cancel"))
			{
				request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
			ImGui::End();
		}
		break;

		case eMenuState::pendingControllerOffsets:
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
			ImGui::Text("Reading tracker data streams...");

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
			ImGui::End();
		}
		break;

		case eMenuState::showControllerOffsets:
		{
			ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
			ImGui::Begin("Triangulation Quality", nullptr, window_flags);

			if (ImGui::Button(" < ##Previous Tracker"))
			{
				m_triangSelectedTracker = fmax(m_triangSelectedTracker - 1, -1);
			}
			ImGui::SameLine();
			if (ImGui::Button(" > ##Next Tracker"))
			{
				m_triangSelectedTracker = fmin(m_triangSelectedTracker + 1, m_trackerViews.size() - 1);
			}
			ImGui::SameLine();
			if (m_triangSelectedTracker == -1)
			{
				ImGui::Text("Tracker ID: <ALL>");
			}
			else
			{
				ImGui::Text("Tracker ID: #%d", m_triangSelectedTracker);
			}

			ImGui::Checkbox("Show Arrows", &m_triangShowArrows);
			ImGui::Checkbox("Show Controller Models", &m_triangShowControllers);
			ImGui::Checkbox("Show Tracker Frustum", &m_triangShowFrustum);
			ImGui::Checkbox("Show Tracker Ids", &m_triangShowTrackerIds);
			ImGui::Checkbox("Show Bounds", &m_triangShowBounds);
			ImGui::Checkbox("Center", &m_triangCenter);

			ImGui::Separator();

			{
				PSMVector3f box_min;
				PSMVector3f box_max;
				PSMVector3f bounds_size;
				get_points_bounds(box_min, box_max, bounds_size);

				PSMVector3f center_box;
				center_box.x = (0.5 * (box_min.x + box_max.x));
				center_box.y = (0.5 * (box_min.y + box_max.y));
				center_box.z = (0.5 * (box_min.z + box_max.z));

				float distance = 1.f - (fmaxf(fabsf(bounds_size.x), fmaxf(fabsf(bounds_size.y), fabsf(bounds_size.z))) / 15.f);
				float center_distance = fmaxf(fabsf(center_box.x), fmaxf(fabsf(center_box.y), fabsf(center_box.z)));

				ImGui::Text("Triangulation Quality:");

				// Should be more than 100cm away for measure
				if (m_triangInfo.size() > 1)
				{
					if (center_distance > 100.f)
					{
						if (distance > 0.75f) {
							ImGui::ProgressBar(distance, ImVec2(-1, 0), "Perfect");
						}
						else if (distance > 0.5f) {
							ImGui::ProgressBar(distance, ImVec2(-1, 0), "Good");
						}
						else if (distance > 0.25f) {
							ImGui::ProgressBar(distance, ImVec2(-1, 0), "Poor");
						}
						else {
							ImGui::ProgressBar(distance, ImVec2(-1, 0), "Terrible");
						}
					}
					else
					{
						ImGui::ProgressBar(0.0f, ImVec2(-1, 0), "Failed");

						ImGui::PushTextWrapPos();
						ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), ImColor(1.f, 0.f, 0.f));
						ImGui::SameLine();
						ImGui::TextColored(ImColor(1.f, 0.f, 0.f), "Point too close to playspace center.");
						ImGui::PopTextWrapPos();
					}
				}
				else
				{
					ImGui::ProgressBar(0.0f, ImVec2(-1, 0), "Failed");

					ImGui::PushTextWrapPos();
					ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), ImColor(1.f, 0.f, 0.f));
					ImGui::SameLine();
					ImGui::TextColored(ImColor(1.f, 0.f, 0.f), "More than 2 triangulation points are required.");
					ImGui::PopTextWrapPos();
				}
			}

			ImGui::Separator();

			if (ImGui::Button("      Return to Test Tracking Pose"))
			{
				setState(eMenuState::testTracking);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconLeft()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SetWindowSize(ImVec2(k_panel_width, 0));
			ImGui::End();
		}
		break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::setState(eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);

        m_menuState = newState;
    }
}

void AppStage_ComputeTrackerPoses::onExitState(eMenuState newState)
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
    case eMenuState::pendingControllerStartRequest:
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	case eMenuState::failedControllerOffsets:
	case eMenuState::failedControllerMagnetometer:
	case eMenuState::failedControllerUnsupported:
        break;
    case eMenuState::verifyTrackers:
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->exit();
        break;
	case eMenuState::testMagnetic:
	case eMenuState::testTracking:
        m_app->setCameraType(_cameraFixed);
		m_app->getFixedCamera()->resetOrientation();
        break;
    case eMenuState::showTrackerVideo:
        break;
	case eMenuState::calibrateStepFailed:
		break;
	case eMenuState::pendingControllerOffsets:
		break;
	case eMenuState::showControllerOffsets:
		m_app->setCameraType(_cameraFixed);
		m_app->getFixedCamera()->resetOrientation();
		break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::onEnterState(eMenuState newState)
{
	m_showTrackerFrustum = true;

    switch (newState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingControllerListRequest:
        break;
    case eMenuState::pendingControllerStartRequest:
        m_controllerViews.clear();
        m_pendingControllerStartCount = 0;
		m_areAllControllerStreamsActive = false;
        break;
    case eMenuState::pendingHmdListRequest:
        break;
    case eMenuState::pendingHmdStartRequest:
        m_hmdViews.clear();
        m_pendingHmdStartCount = 0;
        break;
    case eMenuState::pendingTrackerListRequest:
        break;
    case eMenuState::pendingTrackerStartRequest:
        m_trackerViews.clear();
        m_pendingTrackerStartCount = 0;
        break;
    case eMenuState::failedControllerListRequest:
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
	case eMenuState::failedTrackerStartRequest:
	case eMenuState::failedControllerOffsets:
	case eMenuState::failedControllerMagnetometer:
	case eMenuState::failedControllerUnsupported:
        break;
    case eMenuState::verifyTrackers:
        m_renderTrackerIter = m_trackerViews.begin();
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->enter();
        break;
	case eMenuState::testMagnetic:
	case eMenuState::testTracking:
        {
            for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
            {
                PSMController *controllerView = controller_iter->second.controllerView;

                switch (controllerView->ControllerType)
                {
                case PSMController_Move:
                    controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;
                    break;
                case PSMController_DualShock4:
                    controllerView->ControllerState.PSDS4State.bPoseResetButtonEnabled= true;
                    break;
                }
            }

            m_app->setCameraType(_cameraOrbit);
			m_app->getOrbitCamera()->resetOrientation();
			m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
        }
        break;
    case eMenuState::showTrackerVideo:
        break;
	case eMenuState::calibrateStepFailed:
		break;
	case eMenuState::pendingControllerOffsets:
		{
			m_triangTargetTrackerId = 0;
			m_triangPendingTrackerDataIndexChange = false;
			m_triangTrackerScreenLocations.clear();
			m_triangLastControllerPose.Position.x = 0.f;
			m_triangLastControllerPose.Position.y = 0.f;
			m_triangLastControllerPose.Position.z = 0.f;
			m_triangLastControllerPose.Orientation.x = 0.f;
			m_triangLastControllerPose.Orientation.y = 0.f;
			m_triangLastControllerPose.Orientation.z = 0.f;
			m_triangLastControllerPose.Orientation.w = 1.f;
		}
		break;
	case eMenuState::showControllerOffsets:
		
		m_app->setCameraType(_cameraOrbit);
		m_app->getOrbitCamera()->resetOrientation();
		m_app->getOrbitCamera()->setCameraOrbitLocation(45.f, 25.f, 1000.f);
		break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::update_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end())
    {
        const int tracker_id= m_renderTrackerIter->second.trackerView->tracker_info.tracker_id;

        // Render the latest from the currently active tracker
        if (PSM_PollTrackerVideoStream(tracker_id) == PSMResult_Success)
        {
            const unsigned char *buffer= nullptr;
            if (PSM_GetTrackerVideoFrameBuffer(tracker_id, &buffer) == PSMResult_Success)
            {
                m_renderTrackerIter->second.textureAsset->copyBufferIntoTexture(buffer);
            }
        }
    }
}

void AppStage_ComputeTrackerPoses::render_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end() &&
        m_renderTrackerIter->second.textureAsset != nullptr)
    {
        drawFullscreenTexture(m_renderTrackerIter->second.textureAsset->texture_id);
    }
}

void AppStage_ComputeTrackerPoses::go_to_tracker(PSMTrackerID tracker_id)
{
	const int trackerCount = static_cast<int>(m_trackerViews.size());

	if (trackerCount > 1 && tracker_id > -1)
	{
		m_renderTrackerIndex = tracker_id % trackerCount;

		// Find the tracker iterator that corresponds to the render index we want to show
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			if (iter->second.listIndex == m_renderTrackerIndex)
			{
				m_renderTrackerIter = iter;
			}
		}
	}
}

void AppStage_ComputeTrackerPoses::go_next_tracker()
{
	const int trackerCount = static_cast<int>(m_trackerViews.size());

	if (trackerCount > 1)
	{
		m_renderTrackerIndex = (m_renderTrackerIndex + 1) % trackerCount;

		// Find the tracker iterator that corresponds to the render index we want to show
		for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
		{
			if (iter->second.listIndex == m_renderTrackerIndex)
			{
				m_renderTrackerIter = iter;
			}
		}
	}
}

void AppStage_ComputeTrackerPoses::go_previous_tracker()
{
    const int trackerCount = static_cast<int>(m_trackerViews.size());

    if (trackerCount > 1)
    {
        m_renderTrackerIndex = (m_renderTrackerIndex + trackerCount - 1) % trackerCount;

        // Find the tracker iterator that corresponds to the render index we want to show
        for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
        {
            if (iter->second.listIndex == m_renderTrackerIndex)
            {
                m_renderTrackerIter = iter;
            }
        }
    }
}

int AppStage_ComputeTrackerPoses::get_tracker_count() const
{
    return static_cast<int>(m_trackerViews.size());
}

int AppStage_ComputeTrackerPoses::get_render_tracker_index() const
{
    return m_renderTrackerIndex;
}

PSMTracker *AppStage_ComputeTrackerPoses::get_render_tracker_view() const
{
    return (m_trackerViews.size() > 0) ? m_renderTrackerIter->second.trackerView : nullptr;
}

PSMController *AppStage_ComputeTrackerPoses::get_calibration_controller_view() const
{
    return (m_controllerViews.size() > 0) ? m_controllerViews.begin()->second.controllerView : nullptr;
}

PSMHeadMountedDisplay *AppStage_ComputeTrackerPoses::get_calibration_hmd_view() const
{
    return (m_hmdViews.size() > 0) ? m_hmdViews.begin()->second.hmdView : nullptr;
}

void AppStage_ComputeTrackerPoses::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?

    for (t_controller_state_map_iterator iter = m_controllerViews.begin(); iter != m_controllerViews.end(); ++iter)
    {
        ControllerState &controllerState = iter->second;

        if (controllerState.controllerView != nullptr)
        {
            PSMRequestID request_id;
            PSM_StopControllerDataStreamAsync(controllerState.controllerView->ControllerID, &request_id);
            PSM_EatResponse(request_id);

            PSM_FreeControllerListener(controllerState.controllerView->ControllerID);
        }
    }
    m_controllerViews.clear();
    m_pendingControllerStartCount = 0;
	m_areAllControllerStreamsActive = false;

    for (t_hmd_state_map_iterator iter = m_hmdViews.begin(); iter != m_hmdViews.end(); ++iter)
    {
        HMDState &hmdState = iter->second;

        if (hmdState.hmdView != nullptr)
        {
            PSMRequestID request_id;
            PSM_StopHmdDataStreamAsync(hmdState.hmdView->HmdID, &request_id);
            PSM_EatResponse(request_id);

            PSM_FreeHmdListener(hmdState.hmdView->HmdID);
        }
    }
    m_hmdViews.clear();
    m_pendingHmdStartCount = 0;

    for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
    {
        TrackerState &trackerState = iter->second;

		// Free the texture we were rendering to
		if (trackerState.textureAsset != nullptr)
		{
			delete trackerState.textureAsset;
			trackerState.textureAsset = nullptr;
		}

        if (trackerState.trackerView != nullptr)
        {
            const int tracker_id= trackerState.trackerView->tracker_info.tracker_id;

            PSM_CloseTrackerVideoStream(tracker_id);

            PSMRequestID request_id;
            PSM_StopTrackerDataStreamAsync(tracker_id, &request_id);
            PSM_EatResponse(request_id);

            PSM_FreeTrackerListener(tracker_id);
        }
    }
    m_trackerViews.clear();
    m_pendingTrackerStartCount= 0;

    m_renderTrackerIndex= 0;
    m_renderTrackerIter = m_trackerViews.end();
}

void AppStage_ComputeTrackerPoses::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_ComputeTrackerPoses::get_points_bounds(PSMVector3f &box_min, PSMVector3f &box_max, PSMVector3f &bounds_size)
{
	if (m_triangInfo.size() < 1)
	{
		box_min.x = 0.f;
		box_min.y = 0.f;
		box_min.z = 0.f;
		box_max.x = 0.f;
		box_max.y = 0.f;
		box_max.z = 0.f;
		bounds_size.x = 0.f;
		bounds_size.y = 0.f;
		bounds_size.z = 0.f;

		return;
	}

	int N = 0;
	for (auto it = m_triangInfo.begin(); it != m_triangInfo.end(); ++it)
	{
		if (N == 0)
		{
			box_min.x = it->m_point.x;
			box_min.y = it->m_point.y;
			box_min.z = it->m_point.z;
			box_max.x = it->m_point.x;
			box_max.y = it->m_point.y;
			box_max.z = it->m_point.z;
		}
		else
		{
			PSMVector3f point = it->m_point;
			box_min.x = fmax(box_min.x, point.x);
			box_min.y = fmax(box_min.y, point.y);
			box_min.z = fmax(box_min.z, point.z);
			box_max.x = fmin(box_max.x, point.x);
			box_max.y = fmin(box_max.y, point.y);
			box_max.z = fmin(box_max.z, point.z);
		}
		++N;
	}
	if (N > 1)
	{
		bounds_size.x = sqrtf(powf(abs(box_min.x - box_max.x), 2));
		bounds_size.y = sqrtf(powf(abs(box_min.y - box_max.y), 2));
		bounds_size.z = sqrtf(powf(abs(box_min.z - box_max.z), 2));
	}
	else
	{
		bounds_size.x = 0.f;
		bounds_size.y = 0.f;
		bounds_size.z = 0.f;
	}
}

void AppStage_ComputeTrackerPoses::request_controller_list()
{
    if (m_menuState != AppStage_ComputeTrackerPoses::pendingControllerListRequest)
    {
        m_menuState = AppStage_ComputeTrackerPoses::pendingControllerListRequest;
        
        // Request a list of controllers back from the server
        PSMRequestID requestID;
        PSM_GetControllerListAsync(&requestID);
        PSM_RegisterCallback(requestID, AppStage_ComputeTrackerPoses::handle_controller_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_message->opaque_response_handle);
    const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
    
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_ControllerList);
            const PSMControllerList *controller_list = &response_message->payload.controller_list;

            if (thisPtr->m_overrideControllerId == -1)
            {
                bool bStartedAnyControllers = false;

                // Start all psmove, dual shock 4, and virtual controllers
                for (int list_index = 0; list_index < controller_list->count; ++list_index)
                {
                    PSMControllerType controllerType= controller_list->controller_type[list_index];

                    if (controllerType == PSMController_Move ||
                        controllerType == PSMController_DualShock4 ||
                        controllerType == PSMController_Virtual)
                    {
                        int trackedControllerId = controller_list->controller_id[list_index];
                        const auto &protocolControllerResponse = response->result_controller_list().controllers(list_index);
                        const PSMTrackingColorType trackingColorType=
                            static_cast<PSMTrackingColorType>(protocolControllerResponse.tracking_color_type());

                        thisPtr->request_start_controller_stream(trackedControllerId, list_index, trackingColorType);
						bStartedAnyControllers = true;
                    }
                }

                if (!bStartedAnyControllers)
                {
                    thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
                }
            }
            else
            {
                int trackedControllerId = -1;
                int trackedControllerListIndex = -1;
                PSMTrackingColorType trackingColorType;

                // Start only the selected controller
                for (int list_index = 0; list_index < controller_list->count; ++list_index)
                {
                    if (controller_list->controller_id[list_index] == thisPtr->m_overrideControllerId)
                    {
                        const auto &protocolControllerResponse = response->result_controller_list().controllers(list_index);

                        trackingColorType = static_cast<PSMTrackingColorType>(protocolControllerResponse.tracking_color_type());
                        trackedControllerId = controller_list->controller_id[list_index];
                        trackedControllerListIndex = list_index;
                        break;
                    }
                }

                if (trackedControllerId != -1)
                {
                    thisPtr->request_start_controller_stream(trackedControllerId, trackedControllerListIndex, trackingColorType);
                }
                else
                {
                    thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
                }
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_controller_set_tracker_offset(
	int ControllerID,
	int TrackerID)
{
	m_triangPendingTrackerDataIndexChange = true;

	PSMRequestID requestId;
	PSM_SetControllerDataStreamTrackerIndexAsync(ControllerID, TrackerID, &requestId);
	PSM_RegisterCallback(requestId, &AppStage_ComputeTrackerPoses::handle_controller_set_tracker_offset_response, this);
}

void AppStage_ComputeTrackerPoses::handle_controller_set_tracker_offset_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

	const PSMResult ResultCode = response_message->result_code;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		thisPtr->m_triangPendingTrackerDataIndexChange = false;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerOffsets);
	} break;
	}
}

void AppStage_ComputeTrackerPoses::request_start_controller_stream(
    int ControllerID,
    int listIndex,
    PSMTrackingColorType trackingColorType)
{
    ControllerState controllerState;

    setState(eMenuState::pendingControllerStartRequest);

    // Allocate a new controller view
    PSM_AllocateControllerListener(ControllerID);
    controllerState.listIndex = listIndex;
    controllerState.controllerView = PSM_GetController(ControllerID);
    controllerState.trackingColorType = trackingColorType;

    // Add the controller to the list of controllers we're monitoring
    assert(m_controllerViews.find(ControllerID) == m_controllerViews.end());
    m_controllerViews.insert(t_id_controller_state_pair(ControllerID, controllerState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingControllerStartCount;

    unsigned int flags =
		PSMStreamFlags_includePositionData |
		PSMStreamFlags_includeRawSensorData |
        PSMStreamFlags_includeCalibratedSensorData |
        PSMStreamFlags_includeRawTrackerData |
        PSMStreamFlags_includePhysicsData;

    // If we are jumping straight to testing, we want the ROI optimization on
    if (!m_bSkipCalibration)
    {
        flags|= PSMStreamFlags_disableROI;
    }

    // Start off getting getting projection data from tracker 0
    {
        PSMRequestID requestId;
		
        PSM_SetControllerDataStreamTrackerIndexAsync(controllerState.controllerView->ControllerID, 0, &requestId);
        PSM_EatResponse(requestId);
    }

    // Start receiving data from the controller
    {
        PSMRequestID request_id;

        PSM_StartControllerDataStreamAsync(controllerState.controllerView->ControllerID, flags, &request_id);
        PSM_RegisterCallback(request_id, &AppStage_ComputeTrackerPoses::handle_start_controller_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_start_controller_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            // See if this was the last controller we were waiting to get a response from
            --thisPtr->m_pendingControllerStartCount;
            if (thisPtr->m_pendingControllerStartCount <= 0)
            {
				thisPtr->m_areAllControllerStreamsActive = true;
				
				if (thisPtr->m_overrideControllerId != -1)
                {
                    // If we requested a specific controller to test, 
                    // that means we don't care about testing any HMDs
                    thisPtr->request_tracker_list();
                }
                else
                {
                    // Move on to the HMDs
                    thisPtr->request_hmd_list();
                }
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedControllerStartRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_hmd_list()
{
    if (m_menuState != AppStage_ComputeTrackerPoses::pendingHmdListRequest)
    {
        m_menuState = AppStage_ComputeTrackerPoses::pendingHmdListRequest;
        
        // Request a list of controllers back from the server
        PSMRequestID requestID;
        PSM_GetHmdListAsync(&requestID);
        PSM_RegisterCallback(requestID, AppStage_ComputeTrackerPoses::handle_hmd_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_hmd_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_message->opaque_response_handle);
    const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
    
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_HmdList);
            const PSMHmdList *hmd_list = &response_message->payload.hmd_list;

            if (thisPtr->m_overrideHmdId == -1)
            {
                bool bStartedAnyHMDs = false;

                // Start all head mounted displays
                for (int list_index = 0; list_index < hmd_list->count; ++list_index)
                {
                    if (hmd_list->hmd_type[list_index] == PSMHmd_Morpheus ||
                        hmd_list->hmd_type[list_index] == PSMHmd_Virtual)
                    {
                        int trackedHmdId = hmd_list->hmd_id[list_index];
                        const auto &protocolHmdResponse = response->result_hmd_list().hmd_entries(list_index);
                        const PSMTrackingColorType trackingColorType=
                            static_cast<PSMTrackingColorType>(protocolHmdResponse.tracking_color_type());

                        thisPtr->request_start_hmd_stream(trackedHmdId, list_index, trackingColorType);
                        bStartedAnyHMDs = true;
                    }
                }

                if (!bStartedAnyHMDs)
                {
                    // Move on to the tracker list if there are no HMDs
                    thisPtr->request_tracker_list();
                }
            }
            else
            {
                int trackedHmdId = -1;
                int trackedHmdListIndex = -1;
                PSMTrackingColorType trackingColorType;

                // Start only the selected HMD
                for (int list_index = 0; list_index < hmd_list->count; ++list_index)
                {
                    if (hmd_list->hmd_id[list_index] == thisPtr->m_overrideHmdId)
                    {
                        const auto &protocolHmdResponse = response->result_hmd_list().hmd_entries(list_index);

                        trackingColorType = static_cast<PSMTrackingColorType>(protocolHmdResponse.tracking_color_type());
                        trackedHmdId = hmd_list->hmd_id[list_index];
                        trackedHmdListIndex = list_index;
                        break;
                    }
                }

                if (trackedHmdId != -1)
                {
                    thisPtr->request_start_hmd_stream(trackedHmdId, trackedHmdListIndex, trackingColorType);
                }
                else
                {
                    thisPtr->setState(AppStage_ComputeTrackerPoses::failedHmdListRequest);
                }
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedHmdListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_start_hmd_stream(
    PSMHmdID HmdID,
    int listIndex,
    PSMTrackingColorType trackingColorType)
{
    HMDState hmdState;

    setState(eMenuState::pendingHmdStartRequest);

    // Allocate a new HMD view
    PSM_AllocateHmdListener(HmdID);
    hmdState.listIndex = listIndex;
    hmdState.hmdView = PSM_GetHmd(HmdID);
    hmdState.trackingColorType = trackingColorType;

    // Add the hmd to the list of HMDs we're monitoring
    assert(m_hmdViews.find(HmdID) == m_hmdViews.end());
    m_hmdViews.insert(t_id_hmd_state_pair(HmdID, hmdState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingHmdStartCount;

	unsigned int flags =
		PSMStreamFlags_includePositionData |
		PSMStreamFlags_includeRawSensorData |
		PSMStreamFlags_includeCalibratedSensorData |
		PSMStreamFlags_includeRawTrackerData |
		PSMStreamFlags_includePhysicsData;

    // Start off getting getting projection data from tracker 0
    {
        PSMRequestID requestId;

        PSM_SetHmdDataStreamTrackerIndexAsync(hmdState.hmdView->HmdID, 0, &requestId);
        PSM_EatResponse(requestId);
    }

    // Start receiving data from the controller
    {
        PSMRequestID request_id;

        PSM_StartHmdDataStreamAsync(hmdState.hmdView->HmdID, flags, &request_id);
        PSM_RegisterCallback(request_id, &AppStage_ComputeTrackerPoses::handle_start_hmd_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_start_hmd_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    const PSMResult ResultCode = response_message->result_code;

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            // See if this was the last controller we were waiting to get a response from
            --thisPtr->m_pendingHmdStartCount;
            if (thisPtr->m_pendingHmdStartCount <= 0)
            {
                // Move on to the trackers
                thisPtr->request_tracker_list();
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(AppStage_ComputeTrackerPoses::failedHmdStartRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_list()
{
    if (m_menuState != eMenuState::pendingTrackerListRequest)
    {
        setState(eMenuState::pendingTrackerListRequest);

        // Tell the psmove service that we we want a list of trackers connected to this machine
        PSMRequestID requestId;
        PSM_GetTrackerListAsync(&requestId);
        PSM_RegisterCallback(requestId, AppStage_ComputeTrackerPoses::handle_tracker_list_response, this);
    }
}

void AppStage_ComputeTrackerPoses::handle_tracker_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
            const PSMTrackerList &tracker_list = response_message->payload.tracker_list;

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                thisPtr->request_tracker_start_stream(&tracker_list.trackers[tracker_index], tracker_index);
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(eMenuState::failedTrackerListRequest);
        } break;
    }
}

void AppStage_ComputeTrackerPoses::request_tracker_start_stream(
    const PSMClientTrackerInfo *TrackerInfo,
    int listIndex)
{
    TrackerState trackerState;

    setState(eMenuState::pendingTrackerStartRequest);

    // Allocate a new tracker view
    const int tracker_id= TrackerInfo->tracker_id;
    trackerState.listIndex = listIndex;
    PSM_AllocateTrackerListener(tracker_id, TrackerInfo);
    trackerState.trackerView = PSM_GetTracker(tracker_id);
    trackerState.textureAsset = nullptr;

    // Add the tracker to the list of trackers we're monitoring
    assert(m_trackerViews.find(TrackerInfo->tracker_id) == m_trackerViews.end());
    m_trackerViews.insert(t_id_tracker_state_pair(TrackerInfo->tracker_id, trackerState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingTrackerStartCount;

    // Request data to start streaming to the tracker
    PSMRequestID requestID;
    PSM_StartTrackerDataStreamAsync(
        TrackerInfo->tracker_id, 
        &requestID);
    PSM_RegisterCallback(requestID, AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response, this);
}

void AppStage_ComputeTrackerPoses::handle_tracker_start_stream_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ComputeTrackerPoses *thisPtr = static_cast<AppStage_ComputeTrackerPoses *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            // Get the tracker ID this request was for
            const PSMoveProtocol::Request *request = GET_PSMOVEPROTOCOL_REQUEST(response_message->opaque_request_handle);
            const int tracker_id= request->request_start_tracker_data_stream().tracker_id();

            // Get the tracker state associated with the tracker id
            t_tracker_state_map_iterator trackerStateEntry = thisPtr->m_trackerViews.find(tracker_id);
            assert(trackerStateEntry != thisPtr->m_trackerViews.end());

            // The context holds everything a handler needs to evaluate a response
            TrackerState &trackerState = trackerStateEntry->second;
            PSMClientTrackerInfo &trackerInfo= trackerState.trackerView->tracker_info;
			
            // Open the shared memory that the video stream is being written to
            if (PSM_OpenTrackerVideoStream(trackerInfo.tracker_id) == PSMResult_Success)
            {
				// Free the texture we were rendering to
				if (trackerState.textureAsset != nullptr)
				{
					delete trackerState.textureAsset;
					trackerState.textureAsset = nullptr;
				}

                // Create a texture to render the video frame to
                trackerState.textureAsset = new TextureAsset();
                trackerState.textureAsset->init(
                    static_cast<unsigned int>(trackerInfo.tracker_screen_dimensions.x),
                    static_cast<unsigned int>(trackerInfo.tracker_screen_dimensions.y),
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);
            }

            // See if this was the last tracker we were waiting to get a response from
            --thisPtr->m_pendingTrackerStartCount;
            if (thisPtr->m_pendingTrackerStartCount <= 0)
            {
                thisPtr->handle_all_devices_ready();
            }
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->setState(eMenuState::failedTrackerStartRequest);
        } break;
    }
}

static void copy_pose_to_request(
    const PSMPosef &pose,
    PSMoveProtocol::Pose *pose_request)
{
    {
        PSMoveProtocol::Orientation *orientation_request= pose_request->mutable_orientation();

        orientation_request->set_w(pose.Orientation.w);
        orientation_request->set_x(pose.Orientation.x);
        orientation_request->set_y(pose.Orientation.y);
        orientation_request->set_z(pose.Orientation.z);
    }

    {
        PSMoveProtocol::Position *position_request = pose_request->mutable_position();

        position_request->set_x(pose.Position.x);
        position_request->set_y(pose.Position.y);
        position_request->set_z(pose.Position.z);
    }
}

void AppStage_ComputeTrackerPoses::request_set_tracker_pose(
    const PSMPosef *pose,
    PSMTracker *TrackerView)
{
    // Set the pose on out local tracker view
    TrackerView->tracker_info.tracker_pose = *pose;

    // Update the pose on the service
    {
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_POSE);

        PSMoveProtocol::Request_RequestSetTrackerPose *set_pose_request =
            request->mutable_request_set_tracker_pose();

        set_pose_request->set_tracker_id(TrackerView->tracker_info.tracker_id);
        copy_pose_to_request(TrackerView->tracker_info.tracker_pose, set_pose_request->mutable_pose());

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_EatResponse(request_id);
    }
}

void AppStage_ComputeTrackerPoses::handle_all_devices_ready()
{
    if (!m_bSkipCalibration)
    {
        setState(eMenuState::verifyTrackers);
    }
    else
    {
        setState(eMenuState::testTracking);
    }
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_device(const PSMTracker *trackerView)
{
    return does_tracker_see_any_controller(trackerView) || does_tracker_see_any_hmd(trackerView);
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_controller(const PSMTracker *trackerView)
{
    bool bTrackerSeesAnyController = false;
    for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
    {
        const PSMController *controllerView = controller_iter->second.controllerView;
        const int tracker_id= trackerView->tracker_info.tracker_id;

        if (controllerView->ControllerType == PSMControllerType::PSMController_Move)
        {
            bTrackerSeesAnyController |= 
                (controllerView->ControllerState.PSMoveState.RawTrackerData.ValidTrackerBitmask & 
                 (1 << tracker_id)) > 0;
        }
        else if (controllerView->ControllerType == PSMControllerType::PSMController_DualShock4)
        {
            bTrackerSeesAnyController |= 
                (controllerView->ControllerState.PSDS4State.RawTrackerData.ValidTrackerBitmask & 
                 (1 << tracker_id)) > 0;
        }
        else if (controllerView->ControllerType == PSMControllerType::PSMController_Virtual)
        {
            bTrackerSeesAnyController |= 
                (controllerView->ControllerState.VirtualController.RawTrackerData.ValidTrackerBitmask & 
                 (1 << tracker_id)) > 0;
        }
    }

    return bTrackerSeesAnyController;
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_hmd(const PSMTracker *trackerView)
{
    bool bTrackerSeesAnyHmd = false;
    for (t_hmd_state_map_iterator hmd_iter = m_hmdViews.begin(); hmd_iter != m_hmdViews.end(); ++hmd_iter)
    {
        const PSMHeadMountedDisplay *hmdView = hmd_iter->second.hmdView;
        const int tracker_id= trackerView->tracker_info.tracker_id;

        if (hmdView->HmdType == PSMHmd_Morpheus &&
            hmdView->HmdState.MorpheusState.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyHmd |= 
                (hmdView->HmdState.MorpheusState.RawTrackerData.ValidTrackerBitmask & 
                 (1 << tracker_id)) > 0;
        }
        else if (hmdView->HmdType == PSMHmd_Virtual &&
                 hmdView->HmdState.VirtualHMDState.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyHmd |= 
                (hmdView->HmdState.VirtualHMDState.RawTrackerData.ValidTrackerBitmask & 
                 (1 << tracker_id)) > 0;
        }
    }

    return bTrackerSeesAnyHmd;
}

//-- private methods -----
static void drawController(
    const PSMController *controllerView, 
    const glm::mat4 &transform, 
    const PSMTrackingColorType trackingColorType)
{
    glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

    switch (trackingColorType)
    {
    case PSMTrackingColorType_Magenta:
        bulb_color = glm::vec3(1.f, 0.f, 1.f);
        break;
    case PSMTrackingColorType_Cyan:
        bulb_color = glm::vec3(0.f, 1.f, 1.f);
        break;
    case PSMTrackingColorType_Yellow:
        bulb_color = glm::vec3(1.f, 1.f, 0.f);
        break;
    case PSMTrackingColorType_Red:
        bulb_color = glm::vec3(1.f, 0.f, 0.f);
        break;
    case PSMTrackingColorType_Green:
        bulb_color = glm::vec3(0.f, 1.f, 0.f);
        break;
    case PSMTrackingColorType_Blue:
        bulb_color = glm::vec3(0.f, 0.f, 1.f);
        break;
    default:
        break;
    }

    switch(controllerView->ControllerType)
    {
    case PSMController_Move:
        drawPSMoveModel(transform, bulb_color);
        break;
    case PSMController_DualShock4:
        drawPSDualShock4Model(transform, bulb_color);
        break;
    case PSMController_Virtual:
        drawVirtualControllerModel(transform, bulb_color);
        break;
    }
}

static void drawHMD(
    const PSMHeadMountedDisplay *hmdView, 
    const glm::mat4 &transform, 
    const PSMTrackingColorType trackingColorType)
{
    glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

    switch (trackingColorType)
    {
    case PSMTrackingColorType_Magenta:
        bulb_color = glm::vec3(1.f, 0.f, 1.f);
        break;
    case PSMTrackingColorType_Cyan:
        bulb_color = glm::vec3(0.f, 1.f, 1.f);
        break;
    case PSMTrackingColorType_Yellow:
        bulb_color = glm::vec3(1.f, 1.f, 0.f);
        break;
    case PSMTrackingColorType_Red:
        bulb_color = glm::vec3(1.f, 0.f, 0.f);
        break;
    case PSMTrackingColorType_Green:
        bulb_color = glm::vec3(0.f, 1.f, 0.f);
        break;
    case PSMTrackingColorType_Blue:
        bulb_color = glm::vec3(0.f, 0.f, 1.f);
        break;
    default:
        break;
    }
	
    switch(hmdView->HmdType)
    {
    case PSMHmd_Morpheus:
        drawMorpheusModel(transform, true, false, bulb_color);
        break;
    case PSMHmd_Virtual:
        drawVirtualHMDModel(transform, bulb_color);
        break;
    }
}
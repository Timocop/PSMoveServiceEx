//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_OpticalCalibration.h"
#include "AppStage_OpticalRecenter.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_DistortionCalibration.h"
#include "AppStage_MainMenu.h"
#include "MathUtility.h"
#include "App.h"
#include "Camera.h"
#include "PSMoveClient_CAPI.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_TrackerSettings::APP_STAGE_NAME= "CameraSettings";

//-- constants -----

//-- public methods -----
AppStage_TrackerSettings::AppStage_TrackerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_TrackerSettings::inactive)
    , m_selectedTrackerIndex(-1)
    , m_selectedControllerIndex(-1)
    , m_selectedHmdIndex(-1)
    , m_gotoControllerColorCalib(false)
    , m_gotoHMDColorCalib(false)
    , m_gotoTestControllerTracking(false)
    , m_gotoTrackingControllerVideo(false)
    , m_gotoTestHmdTracking(false)
    , m_gotoTrackingHmdVideo(false)
    , m_gotoTrackingVideoALL(false)
	, playspace_orientation_yaw(0.f)
	, playspace_position_x(0.f)
	, playspace_position_y(0.f)
	, playspace_position_z(0.f)
{ }

void AppStage_TrackerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

    request_tracker_list();
}

void AppStage_TrackerSettings::exit()
{
}

void AppStage_TrackerSettings::update()
{
}
    
void AppStage_TrackerSettings::render()
{
    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        if (m_selectedTrackerIndex >= 0)
        {
            const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            switch (trackerInfo.tracker_type)
            {
            case PSMoveProtocol::PS3EYE:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS3EyeModel(scale3);
                } break;
            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::pendingControllerListRequest:
    case eTrackerMenuState::failedControllerListRequest:
	case eTrackerMenuState::pendingHmdListRequest:
	case eTrackerMenuState::failedHmdListRequest:
	case eTrackerMenuState::pendingPlayspaceRequest:
	case eTrackerMenuState::failedPlayspaceRequest:
    {
    } break;

    default:
        assert(0 && "unreachable");
    }
}

const PSMClientTrackerInfo *AppStage_TrackerSettings::getSelectedTrackerInfo() const
{
    return
        (m_selectedTrackerIndex != -1)
        ? &m_trackerInfos[m_selectedTrackerIndex]
        : nullptr;
}
    
void AppStage_TrackerSettings::set_selectedTrackerIndex(int index)
{
    m_selectedTrackerIndex = 
        (index != -1 && index < m_trackerInfos.size())
        ? index
        : m_selectedTrackerIndex;
}
    
void AppStage_TrackerSettings::set_selectedControllerIndex(int index)
{
    m_selectedControllerIndex =
        (index > -2 && index < m_controllerInfos.size())
        ? index
        : m_selectedControllerIndex;
}

int AppStage_TrackerSettings::get_tracker_count() const
{
    return static_cast<int>(m_trackerInfos.size()); 
}

int AppStage_TrackerSettings::get_tracker_Index() const
{
    return m_selectedTrackerIndex;
}

int AppStage_TrackerSettings::get_controller_count() const
{
    return static_cast<int>(m_controllerInfos.size());
}

const AppStage_TrackerSettings::ControllerInfo * AppStage_TrackerSettings::get_controller_info(int index) const
{
    return &m_controllerInfos[index];
}

const AppStage_TrackerSettings::ControllerInfo *AppStage_TrackerSettings::get_selected_controller() {
    const ControllerInfo *controller = NULL;

    if (m_selectedControllerIndex != -1)
    {
        const AppStage_TrackerSettings::ControllerInfo &controllerInfo =
            m_controllerInfos[m_selectedControllerIndex];

        controller = &controllerInfo;
    }

    return controller;
}

const AppStage_TrackerSettings::HMDInfo *AppStage_TrackerSettings::get_selected_hmd()
{
    const HMDInfo *hmd = NULL;

    if (m_selectedHmdIndex != -1)
    {
        const AppStage_TrackerSettings::HMDInfo &hmdinfo =
            m_hmdInfos[m_selectedHmdIndex];

        hmd = &hmdinfo;
    }

    return hmd;
}

void AppStage_TrackerSettings::renderUI()
{
    const char *k_window_title = "Tracker Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(500, ImGui::GetIO().DisplaySize.y - 32));
        ImGui::Begin(k_window_title, nullptr, window_flags & ~ImGuiWindowFlags_NoScrollbar);

        //###HipsterSloth $TODO The tracker restart currently takes longer than it does
        // just to close and re-open the service.
        // For now let's just disable this until we can make this more performant.
        //if (ImGui::Button("Refresh Tracker List"))
        //{
        //    request_search_for_new_trackers();
        //}

        //ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
            const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            if (m_selectedTrackerIndex > 0)
            {
                if (ImGui::Button(" < ##TrackerIndex"))
                {
                    --m_selectedTrackerIndex;
                }
            }
            else {
                if (ImGui::Button(" < ##TrackerIndex"))
                {
                    m_selectedTrackerIndex = static_cast<int>(m_trackerInfos.size()) -1;
                }
            }
			ImGui::SameLine();
			if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.size()))
			{
				if (ImGui::Button(" > ##TrackerIndex"))
				{
					++m_selectedTrackerIndex;
				}
			}
			else {
				if (ImGui::Button(" > ##TrackerIndex"))
				{
					m_selectedTrackerIndex = 0;
				}
			}
            ImGui::SameLine();
            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);

			{
				static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
				ImGui::BeginChild("##TrackerInfoChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
				ImGui::BeginGroup();
				{
					ImGui::Text("Tracker Information:");
					ImGui::Separator();
					ImGui::BulletText("Tracker ID: %d", trackerInfo.tracker_id);

					// Virtual trackers have a common device path "VirtualTracker_#"
					// ###Externet $TODO: Add better virtual tracker check. Probably should do that after changing protocols.
					bool is_virtual = (trackerInfo.device_path[0] == 'V');

					switch (trackerInfo.tracker_type)
					{
					case PSMTracker_PS3Eye:
					{
						if (is_virtual)
						{
							ImGui::BulletText("Controller Type: PS3 Eye (Virtual)");
						}
						else
						{
							ImGui::BulletText("Controller Type: PS3 Eye");
						}
					} break;
					default:
						assert(0 && "Unreachable");
					}

					if (is_virtual)
					{
						ImGui::BulletText("Controller Driver: Virtual");
					}
					else
					{
						switch (trackerInfo.tracker_driver)
						{
						case PSMDriver_LIBUSB:
						{
							ImGui::BulletText("Controller Driver: LIBUSB");
						} break;
						case PSMDriver_CL_EYE:
						{
							ImGui::BulletText("Controller Driver: CLEye");
						} break;
						case PSMDriver_CL_EYE_MULTICAM:
						{
							ImGui::BulletText("Controller Driver: CLEye(Multicam SDK)");
						} break;
						case PSMDriver_GENERIC_WEBCAM:
						{
							ImGui::BulletText("Controller Driver: Generic Webcam");
						} break;
						default:
							assert(0 && "Unreachable");
						}
					}

					ImGui::BulletText("Shared Mem Name: %s", trackerInfo.shared_memory_name);
					ImGui::BulletText("Device Path: ");
					ImGui::SameLine();
					ImGui::TextWrapped("%s", trackerInfo.device_path);
				}
				ImGui::EndGroup();
				if(ImGui::IsItemVisible())
					lastChildVec = ImGui::GetItemRectSize();
				ImGui::EndChild();
			}

            if (m_app->getIsLocalServer())
            {
                if (ImGui::Button("Test Video Feed"))
                {
                    m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
                }

                if (ImGui::Button("Calibrate Tracker Distortion"))
                {
                    m_app->setAppStage(AppStage_DistortionCalibration::APP_STAGE_NAME);
                }
            }
            else
            {
                ImGui::TextDisabled("Test Video Feed");
                ImGui::TextDisabled("Calibrate Tracker Distortion");
            }
        }
        else
        {
            ImGui::Text("No trackers");
        }

        ImGui::Separator();

		if (m_trackerInfos.size() > 0)
		{
			if (m_controllerInfos.size() > 0)
			{
				if (ImGui::CollapsingHeader("Controllers", 0, true, true) ||
					m_gotoControllerColorCalib || m_gotoTestControllerTracking || m_gotoTrackingControllerVideo || m_gotoTrackingVideoALL)
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##ControllersChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						if (m_selectedControllerIndex >= 0)
						{
							if (ImGui::Button(" < ##Controller"))
							{
								--m_selectedControllerIndex;
							}
						}
						else
						{
							ImGui::Button(" < ##Controller");
						}
						ImGui::SameLine();

						if (m_selectedControllerIndex + 1 < static_cast<int>(m_controllerInfos.size()))
						{
							if (ImGui::Button(" > ##Controller"))
							{
								++m_selectedControllerIndex;
							}
						}
						else
						{
							ImGui::Button(" > ##Controller");
						}
						ImGui::SameLine();

						if (m_selectedControllerIndex != -1)
						{
							const AppStage_TrackerSettings::ControllerInfo &controllerInfo =
								m_controllerInfos[m_selectedControllerIndex];

							if (controllerInfo.ControllerType == PSMController_Move ||
								controllerInfo.ControllerType == PSMController_Virtual)
							{
								const char * szControllerLabel = (controllerInfo.ControllerType == PSMController_Move) ? "PSMove" : "Virtual";

								if (0 <= controllerInfo.TrackingColorType && controllerInfo.TrackingColorType < PSMTrackingColorType_MaxColorTypes)
								{
									const char *colors[] = {
										"Magenta","Cyan","Yellow","Red","Green","Blue",
										"Custom0" ,"Custom1" ,"Custom2" ,"Custom3" ,"Custom4" ,"Custom5" ,"Custom6" ,"Custom7" ,"Custom8" ,"Custom9"
									};

									ImGui::Text("Controller: %d (%s) - %s",
										m_selectedControllerIndex,
										szControllerLabel,
										colors[controllerInfo.TrackingColorType]);
								}
								else
								{
									ImGui::Text("Controller: %d (%s)", m_selectedControllerIndex, szControllerLabel);
								}
							}
							else
							{
								ImGui::Text("Controller: %d (DualShock4)", m_selectedControllerIndex);
							}
						}
						else
						{
							ImGui::Text("Controller: <ALL>");
						}

						{
							int controllerID = (m_selectedControllerIndex != -1) ? m_controllerInfos[m_selectedControllerIndex].ControllerID : -1;

							if (ImGui::CollapsingHeader("Calibration##ControllerCalibration", 0, true, true) || 
								m_gotoControllerColorCalib)
							{
								static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
								ImGui::BeginChild("##ControllerCalibrationChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
								ImGui::BeginGroup();
								{
									if (m_app->getIsLocalServer())
									{
										if (ImGui::Button("Calibrate Tracking Colors##Controller") || 
											m_gotoControllerColorCalib)
										{
											m_gotoControllerColorCalib = false;
											
											const ControllerInfo *controller = get_selected_controller();
											if (controller != NULL) {
												m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(controller->ControllerID);
												m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(-1);
												m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(controller->TrackingColorType);
											}
											m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
										}

										if (ImGui::Button("Calibrate Tracker Poses##Controller"))
										{
											AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithController(m_app, controllerID);
										}

										const ControllerInfo *controller = get_selected_controller();
										if (controller != NULL)
										{
											if (ImGui::Button("Calibrate Optical Noise##Controller"))
											{
												const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];
												m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(false);
												m_app->getAppStage<AppStage_OpticalCalibration>()->setTargetTrackerId(trackerInfo.tracker_id);
												m_app->getAppStage<AppStage_OpticalCalibration>()->setTargetControllerId(controller->ControllerID);
												m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
											}
										}

										if (ImGui::IsItemHovered())
											ImGui::SetTooltip(
												"NOTE: This only appiles for following positional filters:\n"
												" - ComplimentaryOpticalIMU\n"
												" - PositionKalman"
											);


										//ImGui::Separator();

										//if (ImGui::Button("Optical Playspace Recenter##Controller"))
										//{
										//	const ControllerInfo *controller = get_selected_controller();
										//	if (controller != NULL)
										//	{
										//		m_app->getAppStage<AppStage_OpticalRecenter>()->setTargetControllerId(controller->ControllerID);
										//		m_app->getAppStage<AppStage_OpticalRecenter>()->setTargetTrackerSettings(this);
										//		m_app->setAppStage(AppStage_OpticalRecenter::APP_STAGE_NAME);
										//	}
										//}
									}
									else
									{
										ImGui::TextDisabled("Calibrate Tracking Colors");
										ImGui::TextDisabled("Calibrate Tracker Poses");
										ImGui::TextDisabled("Calibrate Optical Noise");

										//ImGui::Separator();

										//ImGui::TextDisabled("Optical Playspace Recenter");
									}
								}
								ImGui::EndGroup();
								if (ImGui::IsItemVisible())
									lastChildVec2 = ImGui::GetItemRectSize();
								ImGui::EndChild();
							}

							if (ImGui::CollapsingHeader("Testing##ControllerTesting", 0, true, true) || 
								m_gotoTestControllerTracking)
							{
								static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
								ImGui::BeginChild("##ControllerTestingChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
								ImGui::BeginGroup();
								{
									if (ImGui::Button("Test Tracking Colors##Controller") || m_gotoTrackingControllerVideo)
									{
										m_gotoTrackingControllerVideo = false;
										m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
										AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, controllerID, -1);
									}
									if (ImGui::Button("Test Tracking Pose##Controller") || 
										m_gotoTestControllerTracking)
									{
										m_gotoTestControllerTracking = false;

										AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, controllerID, -1);
									}

									const ControllerInfo *controller = get_selected_controller();
									if (controller != NULL)
									{
										if (ImGui::Button("Test Optical Noise##Controller"))
										{
											const PSMClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];
											m_app->getAppStage<AppStage_OpticalCalibration>()->setBypassCalibrationFlag(true);
											m_app->getAppStage<AppStage_OpticalCalibration>()->setTargetTrackerId(trackerInfo.tracker_id);
											m_app->getAppStage<AppStage_OpticalCalibration>()->setTargetControllerId(controller->ControllerID);
											m_app->setAppStage(AppStage_OpticalCalibration::APP_STAGE_NAME);
										}
									}
								}
								ImGui::EndGroup();
								if (ImGui::IsItemVisible())
									lastChildVec2 = ImGui::GetItemRectSize();
								ImGui::EndChild();
							}

							if (m_gotoTrackingVideoALL)
							{
								m_gotoTrackingVideoALL = false;

								m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
								AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, -1);
							}
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}
			}

			if (m_hmdInfos.size() > 0)
			{
				if (ImGui::CollapsingHeader("Head Mount Devices", 0, true, true) || 
					m_gotoHMDColorCalib || m_gotoTestHmdTracking || m_gotoTrackingHmdVideo || m_gotoTrackingVideoALL)
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##HeadMountDevicesChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						int hmdID = (m_selectedHmdIndex != -1) ? m_hmdInfos[m_selectedHmdIndex].HmdID : -1;

						if (m_selectedHmdIndex > 0)
						{
							if (ImGui::Button(" < ##HMD"))
							{
								--m_selectedHmdIndex;
							}
						}
						else
						{
							ImGui::Button(" < ##HMD");
						}
						ImGui::SameLine();

						if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
						{
							if (ImGui::Button(" > ##HMD"))
							{
								++m_selectedHmdIndex;
							}
						}
						else
						{
							ImGui::Button(" > ##HMD");
						}
						ImGui::SameLine();

						if (m_selectedHmdIndex != -1)
						{
							const AppStage_TrackerSettings::HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];
							const char *colors[] = {
								"Magenta","Cyan","Yellow","Red","Green","Blue",
								"Custom0" ,"Custom1" ,"Custom2" ,"Custom3" ,"Custom4" ,"Custom5" ,"Custom6" ,"Custom7" ,"Custom8" ,"Custom9"
							};

							if (hmdInfo.HmdType == PSMHmd_Morpheus)
							{
								if (0 <= hmdInfo.TrackingColorType && hmdInfo.TrackingColorType < PSMTrackingColorType_MaxColorTypes)
								{
									ImGui::Text("HMD: %d (Morpheus) - %s",
										m_selectedHmdIndex,
										colors[hmdInfo.TrackingColorType]);
								}
								else
								{
									ImGui::Text("HMD: %d (Morpheus)", m_selectedHmdIndex);
								}
							}
							else if (hmdInfo.HmdType == PSMHmd_Virtual)
							{
								if (0 <= hmdInfo.TrackingColorType && hmdInfo.TrackingColorType < PSMTrackingColorType_MaxColorTypes)
								{
									ImGui::Text("HMD: %d (Virtual) - %s",
										m_selectedHmdIndex,
										colors[hmdInfo.TrackingColorType]);
								}
								else
								{
									ImGui::Text("HMD: %d (Virtual)", m_selectedHmdIndex);
								}
							}
						}

						if (ImGui::CollapsingHeader("Calibration##HMDCalibration", 0, true, true) || 
							m_gotoHMDColorCalib)
						{
							static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##HMDCalibrationChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
							ImGui::BeginGroup();
							{
								if (m_app->getIsLocalServer())
								{
									if (ImGui::Button("Calibrate Tracking Colors##HMD") || 
										m_gotoHMDColorCalib)
									{
										m_gotoHMDColorCalib = false;
										
										const HMDInfo *hmd = get_selected_hmd();
										if (hmd != NULL)
										{
											m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(-1);
											m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(hmd->HmdID);
											m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(hmd->TrackingColorType);
										}

										m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
									}

									if (m_selectedHmdIndex != -1)
									{
										const AppStage_TrackerSettings::HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

										if (hmdInfo.HmdType == PSMHmd_Virtual)
										{
											if (ImGui::Button("Calibrate Tracker Poses##HMD"))
											{
												AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithHMD(m_app, hmdID);
											}
										}
									}
								}
								else
								{
									ImGui::TextDisabled("Calibrate Tracking Colors");
									ImGui::TextDisabled("Calibrate Tracker Poses");
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec2 = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}

						if (ImGui::CollapsingHeader("Testing##HMDTesting", 0, true, true) || 
							m_gotoTestHmdTracking || m_gotoTrackingHmdVideo)
						{
							static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##HMDTestingChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
							ImGui::BeginGroup(); 
							{
								if (ImGui::Button("Test Tracking Colors##HMD") || 
									m_gotoTrackingHmdVideo)
								{
									m_gotoTrackingHmdVideo = false;

									m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
									AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, hmdID);
								}
								if (ImGui::Button("Test Tracking Pose##HMD") || 
									m_gotoTestHmdTracking)
								{
									m_gotoTestHmdTracking = false;

									AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, hmdID);
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec2 = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}

						if (m_gotoTrackingVideoALL)
						{
							m_gotoTrackingVideoALL = false;

							m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
							AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, -1);
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}
			}

			if (ImGui::CollapsingHeader("Playspace Offsets", 0, true, false))
			{
				static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
				ImGui::BeginChild("##PlayspaceOffsetsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
				ImGui::BeginGroup();
				{
					bool request_offset = false;

					ImGui::Text("Orientation Y (Yaw): ");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(120.f);
					if (ImGui::InputFloat("##OffsetOrientationY", &playspace_orientation_yaw, 1.f, 5.f, 2))
					{
						while (playspace_orientation_yaw < 0.f)
							playspace_orientation_yaw += 360.f;
						while (playspace_orientation_yaw >= 360.f)
							playspace_orientation_yaw -= 360.f;

						request_offset = true;
					}
					ImGui::PopItemWidth();

					ImGui::Separator();

					ImGui::Text("Position X (Right): ");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(120.f);
					if (ImGui::InputFloat("##OffsetPositionX", &playspace_position_x, 1.f, 5.f, 2))
					{
						playspace_position_x = clampf(playspace_position_x, -(1 << 16), (1 << 16));

						request_offset = true;
					}
					ImGui::PopItemWidth();

					ImGui::Text("Position Y (Up): ");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(120.f);
					if (ImGui::InputFloat("##OffsetPositionY", &playspace_position_y, 1.f, 5.f, 2))
					{
						playspace_position_y = clampf(playspace_position_y, -(1 << 16), (1 << 16));

						request_offset = true;
					}
					ImGui::PopItemWidth();

					ImGui::Text("Position Z (Backward): ");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(120.f);
					if (ImGui::InputFloat("##OffsetPositionZ", &playspace_position_z, 1.f, 5.f, 2))
					{
						playspace_position_z = clampf(playspace_position_z, -(1 << 16), (1 << 16));

						request_offset = true;
					}
					ImGui::PopItemWidth();

					ImGui::Separator();

					if (ImGui::Button("Reset All"))
					{
						playspace_orientation_yaw = 0.f;
						playspace_position_x = 0.f;
						playspace_position_y = 0.f;
						playspace_position_z = 0.f;

						request_offset = true;
					}

					if (request_offset)
					{
						request_set_playspace_offsets(
							playspace_orientation_yaw,
							playspace_position_x,
							playspace_position_y,
							playspace_position_z);
					}
				}
				ImGui::EndGroup();
				if (ImGui::IsItemVisible())
					lastChildVec = ImGui::GetItemRectSize();
				ImGui::EndChild();
			}
        }

        ImGui::Separator();

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::pendingControllerListRequest:
	case eTrackerMenuState::pendingHmdListRequest:
	case eTrackerMenuState::pendingPlayspaceRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for server response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::failedControllerListRequest:
	case eTrackerMenuState::failedHmdListRequest:
	case eTrackerMenuState::failedPlayspaceRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get server response!");

        if (ImGui::Button("Retry"))
        {
            request_tracker_list();
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_TrackerSettings::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case  PSMEventMessage::PSMEvent_controllerListUpdated:
        {
            bHandled = true;
            request_tracker_list();
        } break;
    }

    return bHandled;
}

void AppStage_TrackerSettings::request_tracker_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingTrackerListRequest)
    {
        m_menuState = AppStage_TrackerSettings::pendingTrackerListRequest;

        // Tell the psmove service that we we want a list of trackers connected to this machine
        PSMRequestID requestId;
        PSM_GetTrackerListAsync(&requestId);
        PSM_RegisterCallback(requestId, AppStage_TrackerSettings::handle_tracker_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    switch (response_message->result_code)
    {
    case PSMResult_Success:
        {
            assert(response_message->payload_type == PSMResponseMessage::_responsePayloadType_TrackerList);
            const PSMTrackerList &tracker_list= response_message->payload.tracker_list;
            int oldSelectedTrackerIndex= thisPtr->m_selectedTrackerIndex;

            thisPtr->m_selectedTrackerIndex = -1;
            thisPtr->m_trackerInfos.clear();

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const PSMClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

                thisPtr->m_trackerInfos.push_back(TrackerInfo);
            }

            if (oldSelectedTrackerIndex != -1)
            {
                // Maintain the same position in the list if possible
                thisPtr->m_selectedTrackerIndex= 
                    (oldSelectedTrackerIndex < thisPtr->m_trackerInfos.size()) 
                    ? oldSelectedTrackerIndex
                    : 0;
            }
            else
            {
                thisPtr->m_selectedTrackerIndex= (thisPtr->m_trackerInfos.size() > 0) ? 0 : -1;
            }

            // Request the list of controllers next
            thisPtr->request_controller_list();
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_controller_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_TrackerSettings::pendingControllerListRequest;

        // Tell the psmove service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Don't need the usb controllers
        request->mutable_request_get_controller_list()->set_include_usb_controllers(false);

        PSMRequestID request_id;
        PSM_SendOpaqueRequest(&request, &request_id);
        PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_controller_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr= static_cast<AppStage_TrackerSettings *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
    const PSMResponseHandle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;

            thisPtr->m_controllerInfos.clear();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_TrackerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();
                ControllerInfo.TrackingColorType = (PSMTrackingColorType)ControllerResponse.tracking_color_type();

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType = PSMController_Move;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType = PSMController_Navi;
                    break;
                case PSMoveProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = PSMController_DualShock4;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSMoveProtocol::VIRTUALCONTROLLER:
                    ControllerInfo.ControllerType = PSMController_Virtual;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                default:
                    assert(0 && "unreachable");
                }			                
            }

            if (oldSelectedControllerIndex != -1)
            {
                // Maintain the same position in the list if possible
                thisPtr->m_selectedControllerIndex= 
                    (oldSelectedControllerIndex < thisPtr->m_controllerInfos.size()) 
                    ? oldSelectedControllerIndex
                    : -1;
            }
            else
            {
                thisPtr->m_selectedControllerIndex= (thisPtr->m_controllerInfos.size() > 0) ? 0 : -1;
            }

            // Request the list of HMDs next
            thisPtr->request_hmd_list();
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
        case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_TrackerSettings::failedControllerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_hmd_list()
{
	if (m_menuState != AppStage_TrackerSettings::pendingHmdListRequest)
	{
		m_menuState = AppStage_TrackerSettings::pendingHmdListRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_LIST);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_hmd_list_response, this);
	}
}

void AppStage_TrackerSettings::handle_hmd_list_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
		int oldSelectedHmdIndex = thisPtr->m_selectedHmdIndex;

		thisPtr->m_hmdInfos.clear();

		for (int hmd_index = 0; hmd_index < response->result_hmd_list().hmd_entries_size(); ++hmd_index)
		{
			const auto &HmdResponse = response->result_hmd_list().hmd_entries(hmd_index);

			AppStage_TrackerSettings::HMDInfo HmdInfo;

			HmdInfo.HmdID = HmdResponse.hmd_id();
			HmdInfo.TrackingColorType = (PSMTrackingColorType)HmdResponse.tracking_color_type();

			switch (HmdResponse.hmd_type())
			{
			case PSMoveProtocol::Morpheus:
				HmdInfo.HmdType = PSMHmd_Morpheus;
				thisPtr->m_hmdInfos.push_back(HmdInfo);
				break;
			case PSMoveProtocol::VirtualHMD:
				HmdInfo.HmdType = PSMHmd_Virtual;
				thisPtr->m_hmdInfos.push_back(HmdInfo);
				break;
			default:
				assert(0 && "unreachable");
			}
		}

		if (oldSelectedHmdIndex != -1)
		{
			// Maintain the same position in the list if possible
			thisPtr->m_selectedHmdIndex =
				(oldSelectedHmdIndex < thisPtr->m_hmdInfos.size())
				? oldSelectedHmdIndex
				: -1;
		}
		else
		{
			thisPtr->m_selectedHmdIndex = (thisPtr->m_hmdInfos.size() > 0) ? 0 : -1;
		}

		thisPtr->request_playspace_info();
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TrackerSettings::failedControllerListRequest;
	} break;
	}
}

void AppStage_TrackerSettings::request_playspace_info()
{
	if (m_menuState != AppStage_TrackerSettings::pendingPlayspaceRequest)
	{
		m_menuState = AppStage_TrackerSettings::pendingPlayspaceRequest;

		// Tell the psmove service that we we want a list of HMDs connected to this machine
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_GET_PLAYSPACE_OFFSETS);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_playspace_info_response, this);
	}
}

void AppStage_TrackerSettings::handle_playspace_info_response(
	const PSMResponseMessage *response_message,
	void *userdata)
{
	AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

	const PSMResult ResultCode = response_message->result_code;
	const PSMResponseHandle response_handle = response_message->opaque_response_handle;

	switch (ResultCode)
	{
	case PSMResult_Success:
	{
		const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->playspace_orientation_yaw = response->result_get_playspace_offsets().playspace_orientation_yaw();
		thisPtr->playspace_position_x = response->result_get_playspace_offsets().playspace_position().x();
		thisPtr->playspace_position_y = response->result_get_playspace_offsets().playspace_position().y();
		thisPtr->playspace_position_z = response->result_get_playspace_offsets().playspace_position().z();

		thisPtr->m_menuState = AppStage_TrackerSettings::idle;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TrackerSettings::failedControllerListRequest;
	} break;
	}
}
void AppStage_TrackerSettings::request_set_playspace_offsets(
	float offset_orientation_yaw,
	float offset_position_x,
	float offset_position_y,
	float offset_position_z)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_PLAYSPACE_OFFSETS);

	PSMoveProtocol::Request_RequestSetPlayspaceOffsets *mutable_request_set_playspace_offsets = request->mutable_request_set_playspace_offsets();
	PSMoveProtocol::Position *mutable_offset_position = mutable_request_set_playspace_offsets->mutable_playspace_position();

	request->mutable_request_set_playspace_offsets()->set_playspace_orientation_yaw(offset_orientation_yaw);
	mutable_offset_position->set_x(offset_position_x);
	mutable_offset_position->set_y(offset_position_y);
	mutable_offset_position->set_z(offset_position_z);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}
void AppStage_TrackerSettings::request_search_for_new_trackers()
{
    // Tell the psmove service that we want see if new trackers are connected.
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SEARCH_FOR_NEW_TRACKERS);

    m_menuState = AppStage_TrackerSettings::pendingSearchForNewTrackersRequest;
    m_selectedTrackerIndex = -1;
    m_trackerInfos.clear();

    PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
    PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_search_for_new_trackers_response, this);
}

void AppStage_TrackerSettings::handle_search_for_new_trackers_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    thisPtr->request_tracker_list();
}

void AppStage_TrackerSettings::setPlayspaceOffsets(
	float orientation_yaw,
	float position_x,
	float position_y,
	float position_z)
{
	playspace_orientation_yaw = orientation_yaw;
	playspace_position_x = position_x;
	playspace_position_y = position_y;
	playspace_position_z = position_z;

	request_set_playspace_offsets(
		playspace_orientation_yaw,
		playspace_position_x,
		playspace_position_y,
		playspace_position_z);
}
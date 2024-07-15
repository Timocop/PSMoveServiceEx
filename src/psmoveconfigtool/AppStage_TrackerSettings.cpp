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
#include "AssetManager.h"

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
	, m_gotoVideoTest(false)
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
	, playspace_scale_x(1.f)
	, playspace_scale_y(1.f)
	, playspace_scale_z(1.f)
	, m_drawRotation(0.f)
	, m_tabSelectedTab(0)
{ }

void AppStage_TrackerSettings::enter()
{
	m_drawRotation = 0.f;

    m_app->setCameraType(_cameraFixed);
	m_app->getFixedCamera()->resetOrientation();
	m_app->getFixedCamera()->setCameraOrbitLocation(45.f, 25.f, 0.f);

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
	m_drawRotation += 0.1;
	while (m_drawRotation > 360.f)
		m_drawRotation -= 360.f;

	glm::mat4 scale2RotateX90 =
		glm::rotate(
			glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f)),
			0.f, glm::vec3(1.f, 0.f, 0.f));

	scale2RotateX90 = glm::translate(
		scale2RotateX90,
		glm::vec3(8.f, 0.f, 0.f));

	scale2RotateX90 = glm::rotate(
		scale2RotateX90,
		-m_drawRotation, glm::vec3(0.f, 1.f, 0.f));

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
					drawPS3EyeModel(scale2RotateX90);
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
	case eTrackerMenuState::trackerResetPoseWarning:
	case eTrackerMenuState::failedTrackerResetPoseRequest:
	case eTrackerMenuState::pendingTrackerResetPoseRequest:
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
	static float waitCount;

	const float k_panel_width = 550.f;

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
		static ImVec2 lastWindowVec = ImVec2(0, 4);
		ImGui::SetNextWindowSize(ImVec2(
			k_panel_width, fminf(lastWindowVec.y + 36,
				ImGui::GetIO().DisplaySize.y - 64))
		);
		ImGui::SetNextWindowPos(ImVec2(
			fmaxf((k_panel_width / 2) - 32, (ImGui::GetIO().DisplaySize.x / 2) - (k_panel_width / 2)) - 200,
			32)
		);
        ImGui::Begin(k_window_title, nullptr, window_flags & ~ImGuiWindowFlags_NoScrollbar);
		ImGui::BeginGroup();
		{
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
						m_selectedTrackerIndex = static_cast<int>(m_trackerInfos.size()) - 1;
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
					static ImVec2 lastChildVec = ImVec2(0, 4);
					ImGui::BeginChild("##TrackerInfoChild", ImVec2(0, lastChildVec.y + 16), true);
					ImGui::BeginGroup();
					{
						ImGui::Image(AssetManager::getInstance()->getIconTracker()->getImTextureId(), ImVec2(32, 32));
						ImGui::SameLine();
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
								ImGui::BulletText("Tracker Type: PS3 Eye (Virtual)");
							}
							else
							{
								ImGui::BulletText("Tracker Type: PS3 Eye");
							}
						} break;
						default:
							assert(0 && "Unreachable");
						}

						if (is_virtual)
						{
							ImGui::BulletText("Tracker Driver: Virtual");
						}
						else
						{
							switch (trackerInfo.tracker_driver)
							{
							case PSMDriver_LIBUSB:
							{
								ImGui::BulletText("Tracker Driver: LIBUSB");
							} break;
							case PSMDriver_CL_EYE:
							{
								ImGui::BulletText("Tracker Driver: CLEye");
							} break;
							case PSMDriver_CL_EYE_MULTICAM:
							{
								ImGui::BulletText("Tracker Driver: CLEye(Multicam SDK)");
							} break;
							case PSMDriver_GENERIC_WEBCAM:
							{
								ImGui::BulletText("Tracker Driver: Generic Webcam");
							} break;
							default:
								assert(0 && "Unreachable");
							}
						}

						ImGui::BulletText("Shared Mem Name: %s", trackerInfo.shared_memory_name);
						ImGui::BulletText("Device Path: ");
						ImGui::SameLine();
						ImGui::TextWrapped("%s", trackerInfo.device_path);

						const ImColor k_colorGreen = ImColor(0.f, 1.f, 0.f);
						const ImColor k_colorOrange = ImColor(1.f, .5f, 0.f);
						const ImColor k_colorRed = ImColor(1.f, 0.f, 0.f);
						const ImColor k_colorBlue = ImColor(0.f, 0.5f, 1.f);

						bool bWarningAndIssuesShown = false;

						if (m_trackerInfos.size() == 1)
						{
							if (!bWarningAndIssuesShown)
							{
								ImGui::Separator();
								ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
								ImGui::SameLine();
								ImGui::Text("Warnings and Issues:");
								ImGui::Separator();
								bWarningAndIssuesShown = true;
							}

							ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), k_colorOrange);
							ImGui::SameLine();
							ImGui::PushTextWrapPos();
							ImGui::TextDisabled(
								"Only one tracker detected!\n"
								"Tracking quality and range will be very limited due to triangulations being unavailable!\n"
								"A minimum of 2 trackers are required for decent tracking quality and range."
							);
							ImGui::PopTextWrapPos();
						}
						else if (m_trackerInfos.size() < 4)
						{
							if (!bWarningAndIssuesShown)
							{
								ImGui::Separator();
								ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
								ImGui::SameLine();
								ImGui::Text("Warnings and Issues:");
								ImGui::Separator();
								bWarningAndIssuesShown = true;
							}

							ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), k_colorBlue);
							ImGui::SameLine();
							ImGui::PushTextWrapPos();
							ImGui::TextDisabled(
								"Limited 180 degree front facing setup.\n"
								"A minimum of 4 trackers are required for a full 360 degree tracking setup."
							);
							ImGui::PopTextWrapPos();
						}

						// Warn user if bus could be overloaded by the amount of cameras.
						for (int i = m_trackerBusInfo.size() - 1; i >= 0; --i)
						{
							if (m_trackerBusInfo[i].TrackerID != m_selectedTrackerIndex)
								continue;
							
							std::vector<int> sameDevices;
							for (int j = m_trackerBusInfo.size() - 1; j >= 0; --j)
							{
								if (strcmp(m_trackerBusInfo[j].port_path, m_trackerBusInfo[i].port_path) != 0)
									continue;

								sameDevices.push_back(m_trackerBusInfo[j].TrackerID);
							}

							if (sameDevices.size() > 1)
							{
								ImGui::Separator();
								ImGui::Image(AssetManager::getInstance()->getIconUsb()->getImTextureId(), ImVec2(32, 32));
								ImGui::SameLine();
								ImGui::Text("Sharing USB Controller with Trackers:", sameDevices.size());
								ImGui::Separator();

								for (int j = sameDevices.size() - 1; j >= 0; --j)
								{
									ImGui::Bullet();
									ImGui::SameLine();
									ImGui::Text("Tracker #%d", sameDevices[j]);
								}

								// Warn if theres too many PSeyes on one USB controller causing possible bandwidth issues.
								if (sameDevices.size() >= 4)
								{
									ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), k_colorBlue);
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled(
										"USB 3.2 (20 Gbit) controller or higher is required."
										"This amount of trackers connected to a USB 3.1 (10 Gbit) controller may not work due to bandwidth limitations.\n"
										"(Assuming the trackers run at 480p@30Hz)"
									);
									ImGui::PopTextWrapPos();
								}
								else if (sameDevices.size() >= 3)
								{
									ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), k_colorBlue);
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled(
										"USB 3.1 (10 Gbit) controller or higher is required.\n"
										"This amount of trackers connected to a USB 3.0 (5 Gbit) controller may not work due to bandwidth limitations.\n"
										"(Assuming the trackers run at 480p@30Hz)"
									);
									ImGui::PopTextWrapPos();
								}
								else if (sameDevices.size() >= 2)
								{
									ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0,0), ImVec2(1,1), k_colorBlue);
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled(
										"USB 3.0 (5 Gbit) controller or higher is required.\n"
										"This amount of trackers connected to a USB 2.0 (500 Mbit) controller may not work due to bandwidth limitations.\n"
										"(Assuming the trackers run at 480p@30Hz)"
									);
									ImGui::PopTextWrapPos();
								}
							}
							break;
						}

						ImGui::Separator();

						if (m_app->getIsLocalServer())
						{
							if (ImGui::Button("Test Video Feed") || m_gotoVideoTest)
							{
								m_gotoVideoTest = false;

								m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
							}

							if (ImGui::Button("Calibrate Tracker Distortion"))
							{
								m_app->setAppStage(AppStage_DistortionCalibration::APP_STAGE_NAME);
							}

							if (ImGui::Button("Reset Tracker Pose"))
							{
								m_menuState = AppStage_TrackerSettings::trackerResetPoseWarning;
							}
						}
						else
						{
							ImGui::Button("Test Video Feed (Unavailable)");
							ImGui::Button("Calibrate Tracker Distortion (Unavailable)");
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}
			}
			else
			{
				ImGui::Text("No trackers");
			}

			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();

			if (m_trackerInfos.size() > 0)
			{
				if (ImGui::ButtonChecked("Controllers##TabControllers", (m_tabSelectedTab == 0), ImVec2(125.f, 0.f)))
					m_tabSelectedTab = 0;
				ImGui::SameLine(0.f, 0.f);
				if (ImGui::ButtonChecked("Head-mounted Displays##TabHmds", (m_tabSelectedTab == 1), ImVec2(200.f, 0.f)))
					m_tabSelectedTab = 1;
				ImGui::SameLine(0.f, 0.f);
				if (ImGui::ButtonChecked("Playspace Offsets##TabOffsets", (m_tabSelectedTab == 2), ImVec2(175.f, 0.f)))
					m_tabSelectedTab = 2;

				if (m_tabSelectedTab == 0 ||
					m_gotoControllerColorCalib || m_gotoTestControllerTracking || m_gotoTrackingControllerVideo || m_gotoTrackingVideoALL)
				{
					ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 5);

					static ImVec2 lastChildVec = ImVec2(0, 4);
					ImGui::BeginChild("##ControllersChild", ImVec2(0, lastChildVec.y + 16), true);
					ImGui::BeginGroup();
					{
						if (m_controllerInfos.size() > 0)
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
									static ImVec2 lastChildVec2 = ImVec2(0, 4);
									ImGui::BeginChild("##ControllerCalibrationChild", ImVec2(0, lastChildVec2.y + 16), true);
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
										}
										else
										{
											ImGui::Button("Calibrate Tracking Colors (Unavailable)");
											ImGui::Button("Calibrate Tracker Poses (Unavailable)");
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
									static ImVec2 lastChildVec2 = ImVec2(0, 4);
									ImGui::BeginChild("##ControllerTestingChild", ImVec2(0, lastChildVec2.y + 16), true);
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
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (m_tabSelectedTab == 1 ||
					m_gotoHMDColorCalib || m_gotoTestHmdTracking || m_gotoTrackingHmdVideo || m_gotoTrackingVideoALL)
				{
					ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 5);

					static ImVec2 lastChildVec = ImVec2(0, 4);
					ImGui::BeginChild("##HeadMountDevicesChild", ImVec2(0, lastChildVec.y + 16), true);
					ImGui::BeginGroup();
					{
						if (m_hmdInfos.size() > 0)
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
								static ImVec2 lastChildVec2 = ImVec2(0, 4);
								ImGui::BeginChild("##HMDCalibrationChild", ImVec2(0, lastChildVec2.y + 16), true);
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
										ImGui::Button("Calibrate Tracking Colors (Unavailable)");
										ImGui::Button("Calibrate Tracker Poses (Unavailable)");
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
								static ImVec2 lastChildVec2 = ImVec2(0, 4);
								ImGui::BeginChild("##HMDTestingChild", ImVec2(0, lastChildVec2.y + 16), true);
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
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (m_tabSelectedTab == 2)
				{
					ImGui::SetCursorPosY(ImGui::GetCursorPosY() - 5);

					static ImVec2 lastChildVec = ImVec2(0, 4);
					ImGui::BeginChild("##PlayspaceOffsetsChild", ImVec2(0, lastChildVec.y + 16), true);
					ImGui::BeginGroup();
					{
						bool request_offset = false;

						ImGui::Text("Orientation Y (Yaw): ");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(120.f);
						if (ImGui::InputFloat("##OffsetOrientationY", &playspace_orientation_yaw, 1.f, 5.f, 2))
						{
							while (playspace_orientation_yaw < -180.f)
								playspace_orientation_yaw += 360.f;
							while (playspace_orientation_yaw >= 180.f)
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

						ImGui::Text("Scale X (Left/Right): ");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(120.f);
						if (ImGui::InputFloat("##ScalePositionX", &playspace_scale_x, 0.01f, 0.05f, 2))
						{
							playspace_scale_x = clampf(playspace_scale_x, 0.01f, 100.f);

							request_offset = true;
						}
						ImGui::PopItemWidth();

						ImGui::Text("Scale Y (Up/Down): ");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(120.f);
						if (ImGui::InputFloat("##ScalePositionY", &playspace_scale_y, 0.01f, 0.05f, 2))
						{
							playspace_scale_y = clampf(playspace_scale_y, 0.01f, 100.f);

							request_offset = true;
						}
						ImGui::PopItemWidth();

						ImGui::Text("Scale Z (Backward): ");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(120.f);
						if (ImGui::InputFloat("##ScalePositionZ", &playspace_scale_z, 0.01f, 0.05f, 2))
						{
							playspace_scale_z = clampf(playspace_scale_z, 0.01f, 100.f);

							request_offset = true;
						}
						ImGui::PopItemWidth();

						ImGui::Separator();

						if (playspace_scale_x != 1.0f ||
							playspace_scale_y != 1.0f ||
							playspace_scale_z != 1.0f)
						{
							ImGui::PushTextWrapPos();
							ImGui::Image(AssetManager::getInstance()->getIconExclamation()->getImTextureId(), ImVec2(24, 24), ImVec2(0, 0), ImVec2(1, 1), ImColor(1.f, .5f, 0.f));
							ImGui::SameLine();
							ImGui::TextDisabled(
								"Playspace scale has been changed!\n"
								"Scaling will not be applied to trackers.\n"
								"Changing the playspace scale can cause abnormal artifacts in pose previews!"
							);
							ImGui::PopTextWrapPos();

							ImGui::Separator();
						}

						if (ImGui::Button("Reset All"))
						{
							playspace_orientation_yaw = 0.f;
							playspace_position_x = 0.f;
							playspace_position_y = 0.f;
							playspace_position_z = 0.f;
							playspace_scale_x = 1.f;
							playspace_scale_y = 1.f;
							playspace_scale_z = 1.f;

							request_offset = true;
						}

						if (request_offset)
						{
							request_set_playspace_offsets(
								playspace_orientation_yaw,
								playspace_position_x,
								playspace_position_y,
								playspace_position_z,
								playspace_scale_x,
								playspace_scale_y,
								playspace_scale_z);
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
		}
		ImGui::EndGroup();
		if (ImGui::IsItemVisible())
			lastWindowVec = ImGui::GetItemRectSize();
        ImGui::End();
    } break;
    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::pendingControllerListRequest:
	case eTrackerMenuState::pendingHmdListRequest:
	case eTrackerMenuState::pendingPlayspaceRequest:
	case eTrackerMenuState::pendingTrackerResetPoseRequest:
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

		ImGui::SetWindowSize(ImVec2(300, 0));
        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::failedControllerListRequest:
	case eTrackerMenuState::failedHmdListRequest:
	case eTrackerMenuState::failedPlayspaceRequest:
	case eTrackerMenuState::failedTrackerResetPoseRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::Begin(k_window_title, nullptr, window_flags);

		ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
		ImGui::SameLine();
		ImGui::Text("Failed to get server response!");

		if (ImGui::Button("Retry"))
		{
			request_tracker_list();
		}

		if (ImGui::Button("Return to Main Menu"))
		{
			m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
		}

		ImGui::SetWindowSize(ImVec2(300, 0));
        ImGui::End();
    } break;
	case eTrackerMenuState::trackerResetPoseWarning:
	{
		ImGui::SetNextWindowPosCenter();
		ImGui::Begin("Reset Tracker Pose", nullptr, window_flags);

		ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
		ImGui::SameLine();
		ImGui::Text(
			"You are about to reset the calibrated pose for this tracker!\n"
			"Do you want to continue?"
		);

		if (ImGui::Button("Reset Pose"))
		{
			request_tracker_reset_pose();
		}

		ImGui::SameLine();

		if (ImGui::Button("Cancel"))
		{
			m_menuState = AppStage_TrackerSettings::idle;
		}

		ImGui::SetWindowSize(ImVec2(500, 0));
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
			thisPtr->m_trackerBusInfo.clear();

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const PSMClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

                thisPtr->m_trackerInfos.push_back(TrackerInfo);

				// Find trackers in the same bus.
				std::string devicePath = std::string(TrackerInfo.device_path);
				if (devicePath.find("USB") == 0)
				{
					int bus_start = devicePath.rfind("\\");
					int bus_end = devicePath.rfind("_");
					if (bus_start != std::string::npos && bus_end != std::string::npos)
					{
						int bus_size = (bus_end - bus_start - 1);
						if (bus_size > 0)
						{
							std::string bus = devicePath.substr(bus_start + 1, bus_size);

							TrackerBusInfo info;
							info.TrackerID = TrackerInfo.tracker_id;
							size_t len = bus.copy(info.port_path, sizeof(info.port_path));
							info.port_path[len] = 0;

							thisPtr->m_trackerBusInfo.push_back(info);
						}
					}
				}
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
		thisPtr->playspace_scale_x = response->result_get_playspace_offsets().playspace_scale().x();
		thisPtr->playspace_scale_y = response->result_get_playspace_offsets().playspace_scale().y();
		thisPtr->playspace_scale_z = response->result_get_playspace_offsets().playspace_scale().z();

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
	float playspace_orientation_yaw,
	float playspace_position_x,
	float playspace_position_y,
	float playspace_position_z,
	float playspace_scale_x,
	float playspace_scale_y,
	float playspace_scale_z)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_PLAYSPACE_OFFSETS);

	PSMoveProtocol::Request_RequestSetPlayspaceOffsets *mutable_request_set_playspace_offsets = request->mutable_request_set_playspace_offsets();
	PSMoveProtocol::Position *mutable_playspace_position = mutable_request_set_playspace_offsets->mutable_playspace_position();
	PSMoveProtocol::Position *mutable_playspace_scale = mutable_request_set_playspace_offsets->mutable_playspace_scale();

	request->mutable_request_set_playspace_offsets()->set_playspace_orientation_yaw(playspace_orientation_yaw);
	mutable_playspace_position->set_x(playspace_position_x);
	mutable_playspace_position->set_y(playspace_position_y);
	mutable_playspace_position->set_z(playspace_position_z);
	mutable_playspace_scale->set_x(playspace_scale_x);
	mutable_playspace_scale->set_y(playspace_scale_y);
	mutable_playspace_scale->set_z(playspace_scale_z);

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

void AppStage_TrackerSettings::request_tracker_reset_pose()
{
	if (m_menuState != AppStage_TrackerSettings::pendingTrackerResetPoseRequest)
	{
		m_menuState = AppStage_TrackerSettings::pendingTrackerResetPoseRequest;

		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_TRACKER_POSE);


		PSMoveProtocol::Request_RequestSetTrackerPose *set_pose_request =
			request->mutable_request_set_tracker_pose();

		set_pose_request->set_tracker_id(m_selectedTrackerIndex);

		PSMoveProtocol::Pose *pose_request = set_pose_request->mutable_pose();
		
		{
			PSMoveProtocol::Orientation *orientation_request = pose_request->mutable_orientation();

			orientation_request->set_w(k_psm_quaternion_identity->w);
			orientation_request->set_x(k_psm_quaternion_identity->x);
			orientation_request->set_y(k_psm_quaternion_identity->y);
			orientation_request->set_z(k_psm_quaternion_identity->z);
		}

		{
			PSMoveProtocol::Position *position_request = pose_request->mutable_position();

			position_request->set_x(0.f);
			position_request->set_y(0.f);
			position_request->set_z(0.f);
		}


		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_RegisterCallback(request_id, AppStage_TrackerSettings::handle_tracker_reset_pose_request, this);
	}
}

void AppStage_TrackerSettings::handle_tracker_reset_pose_request(
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
		//const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

		thisPtr->m_menuState = AppStage_TrackerSettings::idle;
	} break;

	case PSMResult_Error:
	case PSMResult_Canceled:
	case PSMResult_Timeout:
	{
		thisPtr->m_menuState = AppStage_TrackerSettings::failedTrackerResetPoseRequest;
	} break;
	}
}

void AppStage_TrackerSettings::setPlayspaceOffsets(
	float orientation_yaw,
	float position_x,
	float position_y,
	float position_z,
	float scale_x,
	float scale_y,
	float scale_z)
{
	playspace_orientation_yaw = orientation_yaw;
	playspace_position_x = position_x;
	playspace_position_y = position_y;
	playspace_position_z = position_z;
	playspace_scale_x = scale_x;
	playspace_scale_y = scale_y;
	playspace_scale_z = scale_z;

	request_set_playspace_offsets(
		playspace_orientation_yaw,
		playspace_position_x,
		playspace_position_y,
		playspace_position_z,
		playspace_scale_x,
		playspace_scale_y,
		playspace_scale_z);
}
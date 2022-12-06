//-- inludes -----
#include "AppStage_ControllerSettings.h"
#include "AppStage_AccelerometerCalibration.h"
#include "AppStage_OpticalCalibration.h"
#include "AppStage_GyroscopeCalibration.h"
#include "AppStage_MagnetometerCalibration.h"
#include "AppStage_MainMenu.h"
#include "AppStage_PairController.h"
#include "AppStage_TestButtons.h"
#include "AppStage_TestRumble.h"
#include "App.h"
#include "Camera.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>
#include <sstream>

#ifdef _WIN32
#include <windows.h>  // Required for data types
#include <winuser.h>
#include <bthsdpdef.h>
#include <bluetoothapis.h>
#include <Dbt.h>
#include <guiddef.h>
#include <setupapi.h> // Device setup APIs
#include <assert.h>
#include <strsafe.h>
#include <winreg.h>
#include <Shellapi.h>
#include <TlHelp32.h>
#include <Psapi.h>
#endif // _WIN32

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

#ifdef _WIN32
class Win32AdminCheck {

public:
	BOOL psMoveServiceFound = FALSE;
	BOOL psMoveServiceAdminFound = FALSE;
	BOOL psMoveServiceElevated = FALSE;
	BOOL psMoveServiceAdminElevated = FALSE;
	DWORD psMoveServiceId = 0;
	DWORD psMoveServiceAdminEId = 0;

	BOOL IsElevated(HANDLE hProcess) {
		BOOL fRet = FALSE;
		HANDLE hToken = NULL;
		if (OpenProcessToken(hProcess, TOKEN_QUERY, &hToken)) {
			TOKEN_ELEVATION Elevation;
			DWORD cbSize = sizeof(TOKEN_ELEVATION);
			if (GetTokenInformation(hToken, TokenElevation, &Elevation, sizeof(Elevation), &cbSize)) {
				fRet = Elevation.TokenIsElevated;
			}
		}
		if (hToken) {
			CloseHandle(hToken);
		}
		return fRet;
	}

	void CheckProcesses() {

		psMoveServiceFound = FALSE;
		psMoveServiceAdminFound = FALSE;
		psMoveServiceElevated = FALSE;
		psMoveServiceAdminElevated = FALSE;
		psMoveServiceId = 0;
		psMoveServiceAdminEId = 0;

		
		PROCESSENTRY32 entry;
		entry.dwSize = sizeof(PROCESSENTRY32);

		HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);

		if (Process32First(snapshot, &entry) == TRUE)
		{
			while (Process32Next(snapshot, &entry) == TRUE)
			{
				if (stricmp(entry.szExeFile, "PSMoveService.exe") == 0)
				{
					psMoveServiceId = entry.th32ProcessID;
					HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);

					psMoveServiceFound = TRUE;
					psMoveServiceElevated = IsElevated(hProcess);

					CloseHandle(hProcess);
				}

				if (stricmp(entry.szExeFile, "PSMoveServiceAdmin.exe") == 0)
				{
					psMoveServiceAdminEId = entry.th32ProcessID;
					HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);

					psMoveServiceAdminFound = TRUE;
					psMoveServiceAdminElevated = IsElevated(hProcess);

					CloseHandle(hProcess);
				}

			}
		}

		CloseHandle(snapshot);
	}

	void RestartAdminMode() {
		if (psMoveServiceId != 0 && !psMoveServiceElevated) 
		{
			char moduleFileName[MAXCHAR];
			HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, psMoveServiceId);
			if (hProcess != NULL) {

				GetModuleFileNameEx(hProcess, NULL, moduleFileName, MAXCHAR);

				TerminateProcess(hProcess, 0);
				CloseHandle(hProcess);

				char drive[5];
				char dir[MAXCHAR];
				_splitpath_s(moduleFileName, drive, sizeof(drive), dir, sizeof(dir), NULL, 0, NULL, 0);

				std::string adminDir = drive;
				adminDir = adminDir + dir;
				std::string adminPath = adminDir + "PSMoveServiceAdmin.exe";

				SHELLEXECUTEINFO ShExecInfo;
				ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
				ShExecInfo.fMask = NULL;
				ShExecInfo.hwnd = NULL;
				ShExecInfo.lpVerb = NULL;
				ShExecInfo.lpFile = adminPath.c_str();
				ShExecInfo.lpParameters = NULL;
				ShExecInfo.lpDirectory = NULL;
				ShExecInfo.nShow = SW_NORMAL;
				ShExecInfo.hInstApp = NULL;

				BOOL create = ShellExecuteEx(&ShExecInfo);
			}
		}
	}

	BOOL IsAnyElevated() {
		return psMoveServiceElevated || psMoveServiceAdminElevated;
	}

};

Win32AdminCheck adminCheck;
#endif

//-- statics ----
const char *AppStage_ControllerSettings::APP_STAGE_NAME= "ControllerSettings";
const char *AppStage_ControllerSettings::GAMEPAD_COMBO_LABELS[MAX_GAMEPAD_LABELS] = {
    "<NONE>",
    "gamepad0",
    "gamepad1",
    "gamepad2",
    "gamepad3",
    "gamepad4",
    "gamepad5",
    "gamepad6",
    "gamepad7",
    "gamepad8",
    "gamepad9",
    "gamepad10",
    "gamepad11",
    "gamepad12",
    "gamepad13",
    "gamepad14",
    "gamepad15"
};

//-- constants -----
const int k_default_position_filter_index = 1; // LowPassOptical
const int k_default_psmove_orientation_filter_index = 3; // ComplementaryMARG
const int k_default_ds4_position_filter_index = 4; // ComplimentaryOpticalIMU
const int k_default_ds4_orientation_filter_index = 2; // ComplementaryOpticalARG
const int k_default_ds4_gyro_gain_index = 4; // 2000deg/s

const char* k_controller_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "PositionKalman" , "PositionExternalAttachment" };
const char* k_psmove_orientation_filter_names[] = { "PassThru", "MadgwickARG", "MadgwickMARG", "ComplementaryMARG", "ComplementaryOpticalARG", /*"OrientationTargetOpticalARG",*/ "OrientationKalman", "OrientationExternal" };
const char* k_ds4_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "OrientationKalman", "OrientationExternal" };
const char* k_ds4_gyro_gain_setting_labels[] = { "125deg/s", "250deg/s", "500deg/s", "1000deg/s", "2000deg/s", "custom"};

const float k_max_hmd_prediction_time = 0.15f; // About 150ms seems to be about the point where you start to get really bad over-prediction 

inline int find_string_entry(const char *string_entry, const char* string_list[], size_t list_size)
{
    int found_index = -1;
    for (size_t test_index = 0; test_index < list_size; ++test_index)
    {
        if (strncmp(string_entry, string_list[test_index], 32) == 0)
        {
            found_index = static_cast<int>(test_index);
            break;
        }
    }

    return found_index;
}

//-- public methods -----
AppStage_ControllerSettings::AppStage_ControllerSettings(App *app)
	: AppStage(app)
	, m_menuState(AppStage_ControllerSettings::inactive)
	, m_selectedControllerIndex(-1)
	, m_gamepadCount(0)
{ }

void AppStage_ControllerSettings::enter()
{
#ifdef _WIN32
	adminCheck.CheckProcesses();
#endif

    m_app->setCameraType(_cameraFixed);

    request_controller_list();
}

void AppStage_ControllerSettings::exit()
{
    m_menuState= AppStage_ControllerSettings::inactive;

	for (PSMControllerID controller_id : m_controllersStreams)
	{
		PSMRequestID request_id;
		PSM_StopControllerDataStreamAsync(controller_id, &request_id);
		PSM_EatResponse(request_id);

		PSM_FreeControllerListener(controller_id);
	}

	m_controllersStreams.clear();
}

void AppStage_ControllerSettings::update()
{
}
    
void AppStage_ControllerSettings::render()
{
    glm::mat4 scale2RotateX90= 
        glm::rotate(
            glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f)), 
            90.f, glm::vec3(1.f, 0.f, 0.f));    

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
            if (m_selectedControllerIndex >= 0 && m_selectedControllerIndex < m_controllerInfos.size())
            {
                const ControllerInfo &controllerInfo= m_controllerInfos[m_selectedControllerIndex];

                switch(controllerInfo.ControllerType)
                {
                    case PSMController_Move:
                    case PSMController_DualShock4:
                    case PSMController_Virtual:
                        {
                            const ControllerInfo &controllerInfo = m_controllerInfos[m_selectedControllerIndex];

                            // Display the tracking color being used for the controller
                            glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

                            switch (controllerInfo.TrackingColorType)
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

                            if (controllerInfo.ControllerType == PSMController_Move)
                            {
                                drawPSMoveModel(scale2RotateX90, bulb_color);
                            }
                            else if (controllerInfo.ControllerType == PSMController_DualShock4)
                            {
                                drawPSDualShock4Model(scale2RotateX90, bulb_color);
                            }
                            else if (controllerInfo.ControllerType == PSMController_Virtual)
                            {
                                drawVirtualControllerModel(scale2RotateX90, bulb_color);
                            }
                        } break;
                    case PSMController_Navi:
                        {
                            drawPSNaviModel(scale2RotateX90);
                        } break;
                    default:
                        assert(0 && "Unreachable");
                }        
            }
        } break;

    case eControllerMenuState::pendingControllerListRequest:
    case eControllerMenuState::failedControllerListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ControllerSettings::renderUI()
{
    const char *k_window_title= "Controller Settings";
    const ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eControllerMenuState::idle:
        {
			static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(550, fminf(lastWindowVec.y + 32.f, ImGui::GetIO().DisplaySize.y - 32)));
            ImGui::Begin("Controller Settings", nullptr, window_flags & ~ImGuiWindowFlags_NoScrollbar);
			ImGui::BeginGroup();
			{
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##HostInfoChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						ImGui::Text("Bluetooth Information:");
						ImGui::Separator();

						if (m_hostSerial.length() > 1 && m_hostSerial != "00:00:00:00:00:00")
						{
							ImGui::BulletText("Host Serial: %s", m_hostSerial.c_str());
						}
						else
						{
							ImGui::BulletText("No bluetooth adapter detected!");
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (m_controllerInfos.size() > 0)
				{
					ControllerInfo &controllerInfo = m_controllerInfos[m_selectedControllerIndex];

					if (m_selectedControllerIndex > 0)
					{
						if (ImGui::Button(" < ##ControllerIndex"))
						{
							--m_selectedControllerIndex;
						}
					}
					else
					{
						ImGui::Button(" < ##ControllerIndex");
					}
					ImGui::SameLine();
					if (m_selectedControllerIndex + 1 < static_cast<int>(m_controllerInfos.size()))
					{
						if (ImGui::Button(" > ##ControllerIndex"))
						{
							++m_selectedControllerIndex;
						}
					}
					else
					{
						ImGui::Button(" > ##ControllerIndex");
					}
					ImGui::SameLine();
					ImGui::Text("Controller: %d", m_selectedControllerIndex);

					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##ControllerInfoChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							ImGui::Text("Controller Information:");
							ImGui::Separator();

							ImGui::BulletText("Controller ID: %d", controllerInfo.ControllerID);

							switch (controllerInfo.ControllerType)
							{
							case PSMController_Move:
							{
								//###HipsterSloth $TODO - The HID report for fetching the firmware revision doesn't appear to work
								//ImGui::BulletText("Controller Type: PSMove (v%d.%d)", controllerInfo.FirmwareVersion, controllerInfo.FirmwareRevision);
								ImGui::BulletText("Controller Type: PSMove");
							} break;

							case PSMController_Navi:
							{
								ImGui::BulletText("Controller Type: PSNavi");
							} break;

							case PSMController_DualShock4:
							{
								ImGui::BulletText("Controller Type: PSDualShock4");
							} break;

							case PSMController_Virtual:
							{
								ImGui::BulletText("Controller Type: Virtual");
							} break;

							default:
								assert(0 && "Unreachable");
							}

							switch (controllerInfo.ControllerType)
							{
							case PSMController_Move:
							{
								PSMController *psm_controller = PSM_GetController(controllerInfo.ControllerID);
								if (psm_controller != nullptr)
								{
									PSMBatteryState battery = psm_controller->ControllerState.PSMoveState.BatteryValue;

									switch (battery)
									{
									case PSMBattery_0:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(0.0F);
										break;
									}

									case PSMBattery_20:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(0.20F);
										break;
									}

									case PSMBattery_40:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(0.40F);
										break;
									}

									case PSMBattery_60:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(0.60F);
										break;
									}

									case PSMBattery_80:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(0.80F);
										break;
									}

									case PSMBattery_100:
									{
										ImGui::BulletText("Battery discharging:");
										ImGui::SameLine();
										ImGui::ProgressBar(1.0F);
										break;
									}

									case PSMBattery_Charged:
									{
										ImGui::BulletText("Battery fully charged:");
										ImGui::SameLine();
										ImGui::ProgressBar(1.0F);
										break;
									}

									case PSMBattery_Charging:
									{
										ImGui::BulletText("Battery charging...:");
										ImGui::SameLine();

										static float charged_value;
										static std::chrono::milliseconds charged_time;
										std::chrono::milliseconds now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

										if (charged_time < now)
										{
											charged_time = (now + std::chrono::milliseconds(500));

											charged_value += 0.25F;

											if (charged_value > 1.0F)
												charged_value = 0.0F;
										}

										ImGui::ProgressBar(fmin(1.0f, charged_value));
										break;
									}

									default:
									{
										ImGui::BulletText("Battery: Unknown");
										break;
									}
									}
								}
								else
								{
									ImGui::BulletText("Battery: N/A");
								}
								break;
							}
							default:
							{
								ImGui::BulletText("Battery: N/A");
								break;
							}
							}

							ImGui::BulletText("Device Serial: %s", controllerInfo.DeviceSerial.c_str());
							ImGui::BulletText("Assigned Host Serial: %s", controllerInfo.AssignedHostSerial.c_str());

							int separatorCount = 0;

							if (controllerInfo.ControllerType == PSMController_Virtual)
							{
								int comboIndex = (controllerInfo.GamepadIndex < m_gamepadCount) ? controllerInfo.GamepadIndex + 1 : 0;

								if (separatorCount++ == 0)
									ImGui::Separator();

								ImGui::PushItemWidth(195);
								if (ImGui::Combo(
									"Assigned Gamepad",
									&comboIndex,
									ControllerInfo::GamepadIndexComboItemGetter,
									this,
									m_gamepadCount + 1))
								{
									controllerInfo.GamepadIndex = comboIndex - 1;
									request_set_controller_gamepad_index(controllerInfo.ControllerID, controllerInfo.GamepadIndex);
								}
								ImGui::PopItemWidth();
							}
							if (controllerInfo.ControllerType == PSMController_Navi &&
								controllerInfo.PotentialParentControllerSerials.size() > 0)
							{
								if (separatorCount++ == 0)
									ImGui::Separator();

								ImGui::PushItemWidth(195);
								if (ImGui::Combo(
									"Parent Controller",
									&controllerInfo.AssignedParentControllerIndex,
									ControllerInfo::ParentControllerComboItemGetter,
									&controllerInfo,
									static_cast<int>(controllerInfo.PotentialParentControllerSerials.size())))
								{
									std::string parentControllerSerial = controllerInfo.PotentialParentControllerSerials[controllerInfo.AssignedParentControllerIndex];

									controllerInfo.AssignedParentControllerSerial = parentControllerSerial;

									request_set_parent_controller_id(controllerInfo.ControllerID, find_controller_id_by_serial(parentControllerSerial));
								}
								ImGui::PopItemWidth();
							}

							if (controllerInfo.ControllerType == PSMController_Move)
							{
#ifdef _WIN32
								if (adminCheck.IsAnyElevated())
								{
#endif
									if (controllerInfo.IsBluetooth)
									{
										ImGui::Separator();
										if (ImGui::Button("Unpair Controller"))
										{
											m_app->getAppStage<AppStage_PairController>()->request_controller_unpair(controllerInfo.ControllerID, controllerInfo.ControllerType);
											m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
										}
									}
									else
									{
										ImGui::Separator();
										if (ImGui::Button("Pair Controller"))
										{
											m_app->getAppStage<AppStage_PairController>()->request_controller_pair(controllerInfo.ControllerID, controllerInfo.ControllerType);
											m_app->setAppStage(AppStage_PairController::APP_STAGE_NAME);
										}
									}
#ifdef _WIN32
								}
								else
								{
									ImGui::Separator();
									if (ImGui::Button("Pair/Unpair Controller\n(restart required)"))
									{
										adminCheck.RestartAdminMode();
									}
									ImGui::Bullet();
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled("Administrator privileges are required to pair or unpair controllers.");
									ImGui::PopTextWrapPos();
								}
#endif
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}

					if (ImGui::CollapsingHeader("Settings", 0, true, true))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##SettingsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							// Combo box selection for controller tracking color
							if (controllerInfo.ControllerType == PSMController_Virtual ||
								(controllerInfo.ControllerType == PSMController_Move && controllerInfo.IsBluetooth))
							{
								int newTrackingColorType = controllerInfo.TrackingColorType;

								if (ImGui::Checkbox("Enable Optical Tracking", &controllerInfo.OpticalTracking))
								{
									request_set_controller_opticaltracking(controllerInfo.ControllerID, controllerInfo.OpticalTracking);
								}

								if (controllerInfo.ControllerType == PSMController_Virtual)
								{
									if (ImGui::Checkbox("Enable PSmove Emulation", &controllerInfo.PSmoveEmulation))
									{
										request_set_controller_psmove_emulation(controllerInfo.ControllerID, controllerInfo.PSmoveEmulation);
									}
									if (ImGui::IsItemHovered())
									{
										ImGui::SetTooltip("Enables orientation for virtual controllers using external sources.");
									}
								}

								if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0Custom0\0Custom1\0Custom2\0Custom3\0Custom4\0Custom5\0Custom6\0Custom7\0Custom8\0Custom9\0\0"))
								{
									controllerInfo.TrackingColorType = static_cast<PSMTrackingColorType>(newTrackingColorType);

									request_set_controller_tracking_color_id(controllerInfo.ControllerID, controllerInfo.TrackingColorType);

									// Re-request the controller list since the tracking colors could changed for other controllers
									request_controller_list();
								}
							}

							// Combo box selection for controller hand
							if (controllerInfo.ControllerType == PSMController_Virtual ||
								(controllerInfo.ControllerType == PSMController_DualShock4 && controllerInfo.IsBluetooth) ||
								(controllerInfo.ControllerType == PSMController_Move && controllerInfo.IsBluetooth))
							{
								int newHand = controllerInfo.ControllerHand;

								if (ImGui::Combo("Hand", &newHand, "Any\0Left\0Right\0\0"))
								{
									controllerInfo.ControllerHand = static_cast<PSMControllerHand>(newHand);

									request_set_controller_hand(controllerInfo.ControllerID, controllerInfo.ControllerHand);
								}
							}

							if (!m_app->excludePositionSettings &&
								(controllerInfo.IsBluetooth || controllerInfo.ControllerType == PSMController_Virtual))
							{
								if (ImGui::CollapsingHeader("Filters", 0, true, false))
								{
									static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
									ImGui::BeginChild("##FiltersChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
									ImGui::BeginGroup();
									{
										if ((controllerInfo.ControllerType == PSMController_Move && controllerInfo.IsBluetooth) ||
											controllerInfo.ControllerType == PSMController_Virtual)
										{
											ImGui::PushItemWidth(195);
											if (ImGui::Combo("Position Filter", &controllerInfo.PositionFilterIndex, k_controller_position_filter_names, UI_ARRAYSIZE(k_controller_position_filter_names)))
											{
												controllerInfo.PositionFilterName = k_controller_position_filter_names[controllerInfo.PositionFilterIndex];
												request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
											}
											if (ImGui::IsItemHovered())
											{
												show_position_filter_tooltip(controllerInfo.PositionFilterName);
											}

											if (controllerInfo.ControllerType == PSMController_Move)
											{
												if (ImGui::Combo("Orientation Filter", &controllerInfo.OrientationFilterIndex, k_psmove_orientation_filter_names, UI_ARRAYSIZE(k_psmove_orientation_filter_names)))
												{
													controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[controllerInfo.OrientationFilterIndex];
													request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
												}
												if (ImGui::IsItemHovered())
												{
													show_orientation_filter_tooltip(controllerInfo.OrientationFilterName);
												}
											}

											if (ImGui::SliderFloat("##PredictionTime", &controllerInfo.PredictionTime, 0.f, k_max_hmd_prediction_time))
											{
												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											if (ImGui::Button(" - ##PredictionTimeInput"))
											{
												controllerInfo.PredictionTime = clampf(controllerInfo.PredictionTime - 0.025, 0.f, k_max_hmd_prediction_time);

												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											if (ImGui::Button(" + ##PredictionTimeInput"))
											{
												controllerInfo.PredictionTime = clampf(controllerInfo.PredictionTime + 0.025, 0.f, k_max_hmd_prediction_time);

												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											ImGui::Text("Prediction Time");

											ImGui::Separator();

											if (ImGui::Button("Reset Filter Defaults"))
											{
												controllerInfo.PredictionTime = 0.0f;
												controllerInfo.PositionFilterIndex = k_default_position_filter_index;
												controllerInfo.PositionFilterName = k_controller_position_filter_names[k_default_position_filter_index];
												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
												request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);

												if (controllerInfo.ControllerType == PSMController_Move)
												{
													controllerInfo.OrientationFilterIndex = k_default_psmove_orientation_filter_index;
													controllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[k_default_psmove_orientation_filter_index];
													request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
												}
											}
											ImGui::PopItemWidth();
										}
										else if (controllerInfo.ControllerType == PSMController_DualShock4 && controllerInfo.IsBluetooth)
										{
											ImGui::PushItemWidth(195);
											if (ImGui::Combo("Position Filter", &controllerInfo.PositionFilterIndex, k_controller_position_filter_names, UI_ARRAYSIZE(k_controller_position_filter_names)))
											{
												controllerInfo.PositionFilterName = k_controller_position_filter_names[controllerInfo.PositionFilterIndex];
												request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
											}
											if (ImGui::IsItemHovered())
											{
												show_position_filter_tooltip(controllerInfo.PositionFilterName);
											}

											if (ImGui::Combo("Orientation Filter", &controllerInfo.OrientationFilterIndex, k_ds4_orientation_filter_names, UI_ARRAYSIZE(k_ds4_orientation_filter_names)))
											{
												controllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[controllerInfo.OrientationFilterIndex];
												request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
											}
											if (ImGui::IsItemHovered())
											{
												show_orientation_filter_tooltip(controllerInfo.OrientationFilterName);
											}

											if (ImGui::Combo("Gyro Gain", &controllerInfo.GyroGainIndex, k_ds4_gyro_gain_setting_labels, UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels)))
											{
												controllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[controllerInfo.GyroGainIndex];
												request_set_gyroscope_gain_setting(controllerInfo.ControllerID, controllerInfo.GyroGainSetting);
											}

											if (ImGui::SliderFloat("##PredictionTime", &controllerInfo.PredictionTime, 0.f, k_max_hmd_prediction_time))
											{
												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											if (ImGui::Button(" - ##PredictionTimeInput"))
											{
												controllerInfo.PredictionTime = clampf(controllerInfo.PredictionTime - 0.025, 0.f, k_max_hmd_prediction_time);

												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											if (ImGui::Button(" + ##PredictionTimeInput"))
											{
												controllerInfo.PredictionTime = clampf(controllerInfo.PredictionTime + 0.025, 0.f, k_max_hmd_prediction_time);

												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
											}
											ImGui::SameLine();
											ImGui::Text("Prediction Time");

											ImGui::Separator();

											if (ImGui::Button("Reset Filter Defaults"))
											{
												controllerInfo.PredictionTime = 0.0f;
												controllerInfo.PositionFilterIndex = k_default_ds4_position_filter_index;
												controllerInfo.OrientationFilterIndex = k_default_ds4_orientation_filter_index;
												controllerInfo.GyroGainIndex = k_default_ds4_gyro_gain_index;
												controllerInfo.PositionFilterName = k_controller_position_filter_names[k_default_ds4_position_filter_index];
												controllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[k_default_ds4_orientation_filter_index];
												controllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[k_default_ds4_gyro_gain_index];
												request_set_controller_prediction(controllerInfo.ControllerID, controllerInfo.PredictionTime);
												request_set_position_filter(controllerInfo.ControllerID, controllerInfo.PositionFilterName);
												request_set_orientation_filter(controllerInfo.ControllerID, controllerInfo.OrientationFilterName);
												request_set_gyroscope_gain_setting(controllerInfo.ControllerID, controllerInfo.GyroGainSetting);
											}
											ImGui::PopItemWidth();
										}
									}
									ImGui::EndGroup();
									if (ImGui::IsItemVisible())
										lastChildVec2 = ImGui::GetItemRectSize();
									ImGui::EndChild();
								}

								if (controllerInfo.ControllerType != PSMController_Navi)
								{
									if (ImGui::CollapsingHeader("Filter Settings", 0, true, false))
									{
										static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
										ImGui::BeginChild("##FilterSettingsChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
										ImGui::BeginGroup();
										{
											bool request_offset = false;
											bool settings_shown = false;

											if (controllerInfo.PositionFilterName == "PassThru" ||
												controllerInfo.PositionFilterName == "LowPassOptical" ||
												controllerInfo.PositionFilterName == "LowPassExponential")
											{
												settings_shown = true;

												ImGui::Text("Position Prediction Smoothing Distance: ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												if (ImGui::InputFloat("##PredictionSmoothingDistance", &controllerInfo.FilterPredictionDistance, 1.f, 5.f, 2))
												{
													controllerInfo.FilterPredictionDistance = clampf(controllerInfo.FilterPredictionDistance, 1.0f, (1 << 16));

													request_offset = true;
												}
												ImGui::PopItemWidth();

												ImGui::Text("Position Prediction Smoothing Power (%%): ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												float filter_predict_smoothing = (1.f - controllerInfo.FilterPredictionSmoothing) * 100.f;
												if (ImGui::InputFloat("##PredictionSmoothingPower", &filter_predict_smoothing, 1.f, 5.f, 2))
												{
													controllerInfo.FilterPredictionSmoothing = clampf(1.f - (filter_predict_smoothing / 100.f), 0.1f, 1.0f);

													request_offset = true;
												}
												ImGui::PopItemWidth();

											}

											if (controllerInfo.PositionFilterName == "LowPassOptical")
											{
												settings_shown = true;

												ImGui::Text("Position Smoothing Distance: ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												if (ImGui::InputFloat("##LowPassOpticalSmoothingDistance", &controllerInfo.FilterLowPassOpticalDistance, 1.f, 5.f, 2))
												{
													controllerInfo.FilterLowPassOpticalDistance = clampf(controllerInfo.FilterLowPassOpticalDistance, 1.0f, (1 << 16));

													request_offset = true;
												}
												ImGui::PopItemWidth();

												ImGui::Text("Position Smoothing Power (%%): ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												float filter_lowpassoptical_smoothing = (1.f - controllerInfo.FilterLowPassOpticalSmoothing) * 100.f;
												if (ImGui::InputFloat("##LowPassOpticalSmoothingPower", &filter_lowpassoptical_smoothing, 1.f, 5.f, 2))
												{
													controllerInfo.FilterLowPassOpticalSmoothing = clampf(1.f - (filter_lowpassoptical_smoothing / 100.f), 0.1f, 1.0f);

													request_offset = true;
												}
												ImGui::PopItemWidth();
											}

											if (controllerInfo.OrientationFilterName == "ComplementaryMARG")
											{
												settings_shown = true;

												ImGui::Text("Enable Magnetometer: ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												if (ImGui::Checkbox("##EnableMagnetometer", &controllerInfo.FilterEnableMagnetometer))
												{
													request_offset = true;
												}
												ImGui::PopItemWidth();

												ImGui::Text("Use Passive Drift Correction: ");
												ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
												ImGui::PushItemWidth(120.f);
												if (ImGui::Checkbox("##UsePassiveDriftCorrection", &controllerInfo.FilterUsePassiveDriftCorrection))
												{
													request_offset = true;
												}
												ImGui::PopItemWidth();

												ImGui::Indent();
												{
													ImGui::Text("Drift Correction Deadzone: ");
													ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
													ImGui::PushItemWidth(120.f);
													float filter_passive_drift_correction_deadzone = controllerInfo.FilterPassiveDriftCorrectionDeazone;
													if (ImGui::InputFloat("##PassiveDriftCorrectionDeadzone", &filter_passive_drift_correction_deadzone, 1.f, 5.f, 2))
													{
														controllerInfo.FilterPassiveDriftCorrectionDeazone = clampf(filter_passive_drift_correction_deadzone, 1.0f, 100.0f);

														request_offset = true;
													}
													ImGui::PopItemWidth();

													ImGui::Text("Drift Correction Delay (ms): ");
													ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
													ImGui::PushItemWidth(120.f);
													float filter_passive_drift_correction_delay = controllerInfo.FilterPassiveDriftCorrectionDelay;
													if (ImGui::InputFloat("##PassiveDriftCorrectionDelay", &filter_passive_drift_correction_delay, 1.f, 5.f, 2))
													{
														controllerInfo.FilterPassiveDriftCorrectionDelay = clampf(filter_passive_drift_correction_delay, 1.0f, (1 << 16));

														request_offset = true;
													}
													ImGui::PopItemWidth();
												}
												ImGui::Unindent();
											}

											if (!settings_shown)
											{
												ImGui::Text("There are no settings for this filter.");
											}

											ImGui::Separator();

											if (ImGui::Button("Reset Filter Settings Defaults"))
											{
												controllerInfo.FilterPredictionDistance = 10.f;
												controllerInfo.FilterPredictionSmoothing = 0.40f;
												controllerInfo.FilterLowPassOpticalDistance = 10.f;
												controllerInfo.FilterLowPassOpticalSmoothing = 0.40f;
												controllerInfo.FilterEnableMagnetometer = true;
												controllerInfo.FilterUsePassiveDriftCorrection = false;
												controllerInfo.FilterPassiveDriftCorrectionDeazone = 3.f;
												controllerInfo.FilterPassiveDriftCorrectionDelay = 100.f;

												request_offset = true;

											}

											if (request_offset)
											{
												FilterSettings filterSettings;
												filterSettings.filter_prediction_distance = controllerInfo.FilterPredictionDistance;
												filterSettings.filter_prediction_smoothing = controllerInfo.FilterPredictionSmoothing;
												filterSettings.filter_lowpassoptical_distance = controllerInfo.FilterLowPassOpticalDistance;
												filterSettings.filter_lowpassoptical_smoothing = controllerInfo.FilterLowPassOpticalSmoothing;
												filterSettings.filter_enable_magnetometer = controllerInfo.FilterEnableMagnetometer;
												filterSettings.filter_use_passive_drift_correction = controllerInfo.FilterUsePassiveDriftCorrection;
												filterSettings.filter_passive_drift_correction_deadzone = controllerInfo.FilterPassiveDriftCorrectionDeazone;
												filterSettings.filter_passive_drift_correction_delay = controllerInfo.FilterPassiveDriftCorrectionDelay;

												request_set_controller_filter_settings(controllerInfo.ControllerID, filterSettings);
											}
										}
										ImGui::EndGroup();
										if (ImGui::IsItemVisible())
											lastChildVec2 = ImGui::GetItemRectSize();
										ImGui::EndChild();
									}
								}

								if (ImGui::CollapsingHeader("Offsets", 0, true, false))
								{
									static ImVec2 lastChildVec2 = ImVec2(0.f, 4.f);
									ImGui::BeginChild("##OffsetsChild", ImVec2(0.f, lastChildVec2.y + 16.f), true);
									ImGui::BeginGroup();
									{
										static int iOffsetView = 0;
										ImGui::PushItemWidth(250);
										ImGui::Combo("View", &iOffsetView, "Simple\0Advanced\0\0");
										ImGui::PopItemWidth();

										ImGui::Separator();

										bool request_offset = false;

										if (iOffsetView == 1)
										{
											ImGui::Text("Local Orientation X (Roll): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##LocalOffsetOrientationX", &controllerInfo.OffsetOrientation.x, 1.f, 5.f, 2))
											{
												while (controllerInfo.OffsetOrientation.x < 0.f)
													controllerInfo.OffsetOrientation.x += 360.f;
												while (controllerInfo.OffsetOrientation.x >= 360.f)
													controllerInfo.OffsetOrientation.x -= 360.f;

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Local Orientation Y (Yaw): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##LocalOffsetOrientationY", &controllerInfo.OffsetOrientation.y, 1.f, 5.f, 2))
											{
												while (controllerInfo.OffsetOrientation.y < 0.f)
													controllerInfo.OffsetOrientation.y += 360.f;
												while (controllerInfo.OffsetOrientation.y >= 360.f)
													controllerInfo.OffsetOrientation.y -= 360.f;

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Local Orientation Z (Pitch): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##LocalOffsetOrientationZ", &controllerInfo.OffsetOrientation.z, 1.f, 5.f, 2))
											{
												while (controllerInfo.OffsetOrientation.z < 0.f)
													controllerInfo.OffsetOrientation.z += 360.f;
												while (controllerInfo.OffsetOrientation.z >= 360.f)
													controllerInfo.OffsetOrientation.z -= 360.f;

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Separator();

											ImGui::Text("World Orientation X (Roll): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##WorldOffsetOrientationX", &controllerInfo.OffsetWorldOrientation.x, 1.f, 5.f, 2))
											{
												while (controllerInfo.OffsetWorldOrientation.x < 0.f)
													controllerInfo.OffsetWorldOrientation.x += 360.f;
												while (controllerInfo.OffsetWorldOrientation.x >= 360.f)
													controllerInfo.OffsetWorldOrientation.x -= 360.f;

												request_offset = true;
											}
											ImGui::PopItemWidth();
										}

										ImGui::Text("World Orientation Y (Yaw): ");
										ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
										ImGui::PushItemWidth(120.f);
										if (ImGui::InputFloat("##WorldOffsetOrientationY", &controllerInfo.OffsetWorldOrientation.y, 1.f, 5.f, 2))
										{
											while (controllerInfo.OffsetWorldOrientation.y < 0.f)
												controllerInfo.OffsetWorldOrientation.y += 360.f;
											while (controllerInfo.OffsetWorldOrientation.y >= 360.f)
												controllerInfo.OffsetWorldOrientation.y -= 360.f;

											request_offset = true;
										}
										ImGui::PopItemWidth();

										if (iOffsetView == 1)
										{
											ImGui::Text("World Orientation Z (Pitch): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##WorldOffsetOrientationZ", &controllerInfo.OffsetWorldOrientation.z, 1.f, 5.f, 2))
											{
												while (controllerInfo.OffsetWorldOrientation.z < 0.f)
													controllerInfo.OffsetWorldOrientation.z += 360.f;
												while (controllerInfo.OffsetWorldOrientation.z >= 360.f)
													controllerInfo.OffsetWorldOrientation.z -= 360.f;

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Separator();

											ImGui::Text("Position X (Right): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetPositionX", &controllerInfo.OffsetPosition.x, 1.f, 5.f, 2))
											{
												controllerInfo.OffsetPosition.x = clampf(controllerInfo.OffsetPosition.x, -(1 << 16), (1 << 16));

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Position Y (Up): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetPositionY", &controllerInfo.OffsetPosition.y, 1.f, 5.f, 2))
											{
												controllerInfo.OffsetPosition.y = clampf(controllerInfo.OffsetPosition.y, -(1 << 16), (1 << 16));

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Position Z (Backward): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetPositionZ", &controllerInfo.OffsetPosition.z, 1.f, 5.f, 2))
											{
												controllerInfo.OffsetPosition.z = clampf(controllerInfo.OffsetPosition.z, -(1 << 16), (1 << 16));

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Separator();

											ImGui::Text("Scale X (Left/Right): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetScaleX", &controllerInfo.OffsetScale.x, 0.01f, 0.05f, 2))
											{
												controllerInfo.OffsetScale.x = clampf(controllerInfo.OffsetScale.x, 0.01f, 100.0f);

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Scale Y (Up/Down): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetScaleY", &controllerInfo.OffsetScale.y, 0.01f, 0.05f, 2))
											{
												controllerInfo.OffsetScale.y = clampf(controllerInfo.OffsetScale.y, 0.01f, 100.0f);

												request_offset = true;
											}
											ImGui::PopItemWidth();

											ImGui::Text("Scale Z (Forward/Backward): ");
											ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
											ImGui::PushItemWidth(120.f);
											if (ImGui::InputFloat("##OffsetScaleZ", &controllerInfo.OffsetScale.z, 0.01f, 0.05f, 2))
											{
												controllerInfo.OffsetScale.z = clampf(controllerInfo.OffsetScale.z, 0.01f, 100.0f);

												request_offset = true;
											}
											ImGui::PopItemWidth();
										}

										ImGui::Separator();

										ImGui::Text("Magnetometer Center (Yaw): ");
										ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
										ImGui::PushItemWidth(120.f);
										if (ImGui::InputFloat("##OffsetMagnetometerYaw", &controllerInfo.OffsetMagnetometer, 1.f, 5.f, 2))
										{
											controllerInfo.OffsetMagnetometer = clampf(controllerInfo.OffsetMagnetometer, -(1 << 16), (1 << 16));

											request_offset = true;
										}
										if (ImGui::IsItemHovered())
										{
											ImGui::SetTooltip("Use this setting to compensate for magnetometer drift.");
										}

										ImGui::PopItemWidth();

										ImGui::Separator();

										if (ImGui::Button("Reset All"))
										{
											controllerInfo.OffsetOrientation.x = 0.f;
											controllerInfo.OffsetOrientation.y = 0.f;
											controllerInfo.OffsetOrientation.z = 0.f;
											controllerInfo.OffsetWorldOrientation.x = 0.f;
											controllerInfo.OffsetWorldOrientation.y = 0.f;
											controllerInfo.OffsetWorldOrientation.z = 0.f;
											controllerInfo.OffsetPosition.x = 0.f;
											controllerInfo.OffsetPosition.y = 0.f;
											controllerInfo.OffsetPosition.z = 0.f;
											controllerInfo.OffsetScale.x = 1.f;
											controllerInfo.OffsetScale.y = 1.f;
											controllerInfo.OffsetScale.z = 1.f;
											controllerInfo.OffsetMagnetometer = 0.f;

											request_offset = true;
										}

										if (request_offset)
										{
											OffsetSettings offset;
											offset.offset_orientation = controllerInfo.OffsetOrientation;
											offset.offset_world_orientation = controllerInfo.OffsetWorldOrientation;
											offset.offset_position = controllerInfo.OffsetPosition;
											offset.offset_scale = controllerInfo.OffsetScale;
											offset.offset_magnetometer = controllerInfo.OffsetMagnetometer;

											request_set_controller_offsets(controllerInfo.ControllerID, offset);
										}
									}
									ImGui::EndGroup();
									if (ImGui::IsItemVisible())
										lastChildVec2 = ImGui::GetItemRectSize();
									ImGui::EndChild();
								}
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}

					if (controllerInfo.ControllerType == PSMController_Move && controllerInfo.IsBluetooth)
					{
						if (ImGui::CollapsingHeader("Calibration", 0, true, m_app->excludePositionSettings))
						{
							static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##CalibrationChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
							ImGui::BeginGroup();
							{
								if (controllerInfo.HasMagnetometer)
								{
									if (ImGui::Button("Calibrate Magnetometer"))
									{
										m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(false);
										m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
									}
								}
								else
								{
									ImGui::Button("Calibrate Magnetometer (Unavailable)");
									ImGui::Bullet();
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled(
										"The magnetometer for this controller has been disabled or is not available. "
										"Magnetometers are only available for first generation PS3 PSmove controllers."
									);
									ImGui::PopTextWrapPos();
									ImGui::Spacing();
								}

								if (ImGui::Button("Calibrate Gyroscope"))
								{
									m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(false);
									m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}
					}

					if (controllerInfo.IsBluetooth || controllerInfo.ControllerType == PSMController_Virtual)
					{
						if (ImGui::CollapsingHeader("Tests", 0, true, m_app->excludePositionSettings))
						{
							static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
							ImGui::BeginChild("##TestsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
							ImGui::BeginGroup();
							{
								if (controllerInfo.ControllerType == PSMController_Move)
								{
									if (ImGui::Button("Test Orientation"))
									{
										m_app->getAppStage<AppStage_MagnetometerCalibration>()->setBypassCalibrationFlag(true);
										m_app->setAppStage(AppStage_MagnetometerCalibration::APP_STAGE_NAME);
									}
								}

								if (controllerInfo.ControllerType == PSMController_DualShock4)
								{
									if (ImGui::Button("Test Orientation"))
									{
										m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(true);
										m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
									}
								}

								if (controllerInfo.ControllerType == PSMController_Virtual)
								{
									if (ImGui::Button("Test Orientation"))
									{
										m_app->getAppStage<AppStage_GyroscopeCalibration>()->setBypassCalibrationFlag(true);
										m_app->setAppStage(AppStage_GyroscopeCalibration::APP_STAGE_NAME);
									}

									ImGui::Bullet();
									ImGui::SameLine();
									ImGui::PushTextWrapPos();
									ImGui::TextDisabled(
										"Requires PSmove emulation to be enabled."
									);
									ImGui::PopTextWrapPos();
									ImGui::Spacing();
								}

								if (controllerInfo.ControllerType == PSMController_Move ||
									controllerInfo.ControllerType == PSMController_DualShock4)
								{
									if (ImGui::Button("Test Accelerometer"))
									{
										m_app->getAppStage<AppStage_AccelerometerCalibration>()->setBypassCalibrationFlag(true);
										m_app->setAppStage(AppStage_AccelerometerCalibration::APP_STAGE_NAME);
									}

									if (ImGui::Button("Test Rumble"))
									{
										m_app->setAppStage(AppStage_TestRumble::APP_STAGE_NAME);
									}
								}

								if (ImGui::Button("Test Buttons"))
								{
									m_app->setAppStage(AppStage_TestButtons::APP_STAGE_NAME);
								}
							}
							ImGui::EndGroup();
							if (ImGui::IsItemVisible())
								lastChildVec = ImGui::GetItemRectSize();
							ImGui::EndChild();
						}
					}
				}
				else
				{
					ImGui::Text("No connected controllers found!");
				}

				ImGui::Separator();

				if (m_app->excludePositionSettings)
				{
					if (ImGui::Button("Exit"))
					{
						m_app->requestShutdown();
					}
				}
				else
				{
					if (ImGui::Button("Return to Main Menu"))
					{
						m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
					}
				}
			}
			ImGui::EndGroup();
			if (ImGui::IsItemVisible())
				lastWindowVec = ImGui::GetItemRectSize();

            ImGui::End();
        } break;
    case eControllerMenuState::pendingControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(300, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for controller list response...");

            ImGui::End();
        } break;
    case eControllerMenuState::failedControllerListRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(300, 150));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to get controller list!");

            if (ImGui::Button("Retry"))
            {
                request_controller_list();
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

bool AppStage_ControllerSettings::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled= false;

    switch(event)
    {
	case PSMEventMessage::PSMEvent_controllerListUpdated:
	{
		bHandled = true;
		request_controller_list();
	} break;
    }

    return bHandled;
}

void AppStage_ControllerSettings::request_controller_list()
{
    if (m_menuState != AppStage_ControllerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_ControllerSettings::pendingControllerListRequest;

        // Tell the psmove service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Do get controllers connected bia USB in this menu since we need the info for pairing/unpairing
        request->mutable_request_get_controller_list()->set_include_usb_controllers(true);

        PSMRequestID request_id;
        PSM_SendOpaqueRequest(&request, &request_id);
        PSM_RegisterCallback(request_id, AppStage_ControllerSettings::handle_controller_list_response, this);
    }
}

void AppStage_ControllerSettings::request_set_orientation_filter(
    const int controller_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_ORIENTATION_FILTER);

    request->mutable_request_set_orientation_filter()->set_controller_id(controller_id);
    request->mutable_request_set_orientation_filter()->set_orientation_filter(filter_name);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_position_filter(
    const int controller_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_POSITION_FILTER);

    request->mutable_request_set_position_filter()->set_controller_id(controller_id);
    request->mutable_request_set_position_filter()->set_position_filter(filter_name);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_gyroscope_gain_setting(
    const int controller_id,
    const std::string& gain_setting)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_GYROSCOPE_CALIBRATION);

    PSMoveProtocol::Request_RequestSetControllerGyroscopeCalibration *calibration =
        request->mutable_set_controller_gyroscope_calibration_request();

    calibration->set_controller_id(controller_id);
    calibration->set_drift(-1.f); // keep existing drift
    calibration->set_variance(-1.f); // keep existing variance
    calibration->set_gyro_gain_setting(gain_setting);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_prediction(
	const int controller_id,
	const float prediction_time)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_PREDICTION_TIME);

	PSMoveProtocol::Request_RequestSetControllerPredictionTime *calibration =
		request->mutable_request_set_controller_prediction_time();

	calibration->set_controller_id(controller_id);
	calibration->set_prediction_time(prediction_time); // keep existing drift

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_opticaltracking(
	const int controller_id,
	const bool enabled)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_OPTICAL_TRACKING);

	PSMoveProtocol::Request_RequestSetControllerOpticalTracking *calibration =
		request->mutable_request_set_controller_optical_tracking();

	calibration->set_controller_id(controller_id);
	calibration->set_enabled(enabled);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_psmove_emulation(
	const int controller_id,
	const bool enabled)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_PSMOVE_EMULATION);

	PSMoveProtocol::Request_RequestSetControllerPSmoveEmulation *calibration =
		request->mutable_request_set_controller_psmove_emulation();

	calibration->set_controller_id(controller_id);
	calibration->set_enabled(enabled);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_gamepad_index(
    const int controller_id, 
    const int gamepad_index)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_GAMEPAD_INDEX);

    PSMoveProtocol::Request_RequestSetGamepadIndex *gamepad_request =
        request->mutable_request_set_gamepad_index();

    gamepad_request->set_controller_id(controller_id);
    gamepad_request->set_gamepad_index(gamepad_index);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_hand(
	const int controller_id, 
	const PSMControllerHand controller_hand)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_HAND);

    PSMoveProtocol::Request_RequestSetControllerHand *hand_request =
        request->mutable_request_set_controller_hand();

    hand_request->set_controller_id(controller_id);
    hand_request->set_controller_hand(static_cast<PSMoveProtocol::ControllerHand>(controller_hand));

	PSMRequestID request_id;
    PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_controller_filter_settings(
	const int controller_id,
	FilterSettings filterSettings)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_FILTER_SETTINGS);

	PSMoveProtocol::Request_RequestSetControllerFilterSettings *filter_settings =
		request->mutable_request_set_controller_filter_settings();

	filter_settings->set_controller_id(controller_id);
	filter_settings->set_filter_prediction_distance(filterSettings.filter_prediction_distance);
	filter_settings->set_filter_prediction_smoothing(filterSettings.filter_prediction_smoothing);
	filter_settings->set_filter_lowpassoptical_distance(filterSettings.filter_lowpassoptical_distance);
	filter_settings->set_filter_lowpassoptical_smoothing(filterSettings.filter_lowpassoptical_smoothing);
	filter_settings->set_filter_enable_magnetometer(filterSettings.filter_enable_magnetometer);
	filter_settings->set_filter_use_passive_drift_correction(filterSettings.filter_use_passive_drift_correction);
	filter_settings->set_filter_passive_drift_correction_deadzone(filterSettings.filter_passive_drift_correction_deadzone);
	filter_settings->set_filter_passive_drift_correction_delay(filterSettings.filter_passive_drift_correction_delay);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::handle_controller_list_response(
    const PSMResponseMessage *response_message,
    void *userdata)
{
    AppStage_ControllerSettings *thisPtr= static_cast<AppStage_ControllerSettings *>(userdata);

    const PSMResult ResultCode = response_message->result_code;
    const PSMResponseHandle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case PSMResult_Success:
        {
			const PSMoveProtocol::Response *response= GET_PSMOVEPROTOCOL_RESPONSE(response_handle);
            int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;
			
            thisPtr->m_hostSerial = response->result_controller_list().host_serial();
            thisPtr->m_selectedControllerIndex= -1;
            thisPtr->m_controllerInfos.clear();
            thisPtr->m_gamepadCount= response->result_controller_list().gamepad_count();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_ControllerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();

                switch(ControllerResponse.controller_type())
                {
                case PSMoveProtocol::PSMOVE:
                    ControllerInfo.ControllerType = PSMController_Move;
                    break;
                case PSMoveProtocol::PSNAVI:
                    ControllerInfo.ControllerType = PSMController_Navi;
                    break;
                case PSMoveProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = PSMController_DualShock4;
                    break;
                case PSMoveProtocol::VIRTUALCONTROLLER:
                    ControllerInfo.ControllerType = PSMController_Virtual;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                ControllerInfo.TrackingColorType = 
                    static_cast<PSMTrackingColorType>(ControllerResponse.tracking_color_type());
                ControllerInfo.DevicePath= ControllerResponse.device_path();
                ControllerInfo.DeviceSerial= ControllerResponse.device_serial();
                ControllerInfo.AssignedHostSerial= ControllerResponse.assigned_host_serial();
                ControllerInfo.AssignedParentControllerSerial= ControllerResponse.parent_controller_serial();
                ControllerInfo.AssignedParentControllerIndex= -1;
                ControllerInfo.PotentialParentControllerSerials.clear();
                ControllerInfo.IsBluetooth= ControllerResponse.connection_type() == PSMoveProtocol::Response_ResultControllerList_ControllerInfo_ConnectionType_BLUETOOTH;
                ControllerInfo.FirmwareVersion = ControllerResponse.firmware_version();
                ControllerInfo.FirmwareRevision = ControllerResponse.firmware_revision();
                ControllerInfo.HasMagnetometer = ControllerResponse.has_magnetometer();
                ControllerInfo.OrientationFilterName= ControllerResponse.orientation_filter();
                ControllerInfo.PositionFilterName = ControllerResponse.position_filter();
                ControllerInfo.GyroGainSetting = ControllerResponse.gyro_gain_setting();
				ControllerInfo.PredictionTime = ControllerResponse.prediction_time();
                ControllerInfo.GamepadIndex = ControllerResponse.gamepad_index();
				ControllerInfo.ControllerHand = static_cast<PSMControllerHand>(ControllerResponse.controller_hand());
				ControllerInfo.OpticalTracking = ControllerResponse.opticaltracking();
				ControllerInfo.PSmoveEmulation = ControllerResponse.psmove_emulation();

				ControllerInfo.OffsetOrientation.x = ControllerResponse.offset_orientation().x();
				ControllerInfo.OffsetOrientation.y = ControllerResponse.offset_orientation().y();
				ControllerInfo.OffsetOrientation.z = ControllerResponse.offset_orientation().z();
				ControllerInfo.OffsetWorldOrientation.x = ControllerResponse.offset_world_orientation().x();
				ControllerInfo.OffsetWorldOrientation.y = ControllerResponse.offset_world_orientation().y();
				ControllerInfo.OffsetWorldOrientation.z = ControllerResponse.offset_world_orientation().z();
				ControllerInfo.OffsetPosition.x = ControllerResponse.offset_position().x();
				ControllerInfo.OffsetPosition.y = ControllerResponse.offset_position().y();
				ControllerInfo.OffsetPosition.z = ControllerResponse.offset_position().z();
				ControllerInfo.OffsetScale.x = ControllerResponse.offset_scale().x();
				ControllerInfo.OffsetScale.y = ControllerResponse.offset_scale().y();
				ControllerInfo.OffsetScale.z = ControllerResponse.offset_scale().z();
				ControllerInfo.OffsetMagnetometer = ControllerResponse.offset_magnetometer();

				ControllerInfo.FilterPredictionDistance = ControllerResponse.filter_prediction_distance();
				ControllerInfo.FilterPredictionSmoothing = ControllerResponse.filter_prediction_smoothing();
				ControllerInfo.FilterLowPassOpticalDistance = ControllerResponse.filter_lowpassoptical_distance();
				ControllerInfo.FilterLowPassOpticalSmoothing = ControllerResponse.filter_lowpassoptical_smoothing();
				ControllerInfo.FilterEnableMagnetometer = ControllerResponse.filter_enable_magnetometer();
				ControllerInfo.FilterUsePassiveDriftCorrection = ControllerResponse.filter_use_passive_drift_correction();
				ControllerInfo.FilterPassiveDriftCorrectionDeazone = ControllerResponse.filter_passive_drift_correction_deadzone();
				ControllerInfo.FilterPassiveDriftCorrectionDelay = ControllerResponse.filter_passive_drift_correction_delay();

                if (ControllerInfo.ControllerType == PSMController_Move)
                {
                    ControllerInfo.OrientationFilterIndex =
                        find_string_entry(
                            ControllerInfo.OrientationFilterName.c_str(),
                            k_psmove_orientation_filter_names,
                            UI_ARRAYSIZE(k_psmove_orientation_filter_names));
                    if (ControllerInfo.OrientationFilterIndex == -1)
                    {
                        ControllerInfo.OrientationFilterName = k_psmove_orientation_filter_names[0];
                        ControllerInfo.OrientationFilterIndex = 0;
                    }
                }
                else if (ControllerInfo.ControllerType == PSMController_DualShock4)
                {
                    ControllerInfo.OrientationFilterIndex =
                        find_string_entry(
                            ControllerInfo.OrientationFilterName.c_str(),
                            k_ds4_orientation_filter_names,
                            UI_ARRAYSIZE(k_ds4_orientation_filter_names));
                    if (ControllerInfo.OrientationFilterIndex == -1)
                    {
                        ControllerInfo.OrientationFilterName = k_ds4_orientation_filter_names[0];
                        ControllerInfo.OrientationFilterIndex = 0;
                    }
                }
                else
                {
                    ControllerInfo.OrientationFilterName = "";
                    ControllerInfo.OrientationFilterIndex = -1;
                }

                if (ControllerInfo.ControllerType == PSMController_Move ||
                    ControllerInfo.ControllerType == PSMController_DualShock4 ||
                    ControllerInfo.ControllerType == PSMController_Virtual)
                {
                    ControllerInfo.PositionFilterIndex =
                        find_string_entry(
                            ControllerInfo.PositionFilterName.c_str(),
                            k_controller_position_filter_names,
                            UI_ARRAYSIZE(k_controller_position_filter_names));
                    if (ControllerInfo.PositionFilterIndex == -1)
                    {
                        ControllerInfo.PositionFilterName = k_controller_position_filter_names[0];
                        ControllerInfo.PositionFilterIndex = 0;
                    }
                }
                else
                {
                    ControllerInfo.PositionFilterName = "";
                    ControllerInfo.PositionFilterIndex = -1;
                }

                if (ControllerInfo.ControllerType == PSMController_DualShock4)
                {
                    ControllerInfo.GyroGainIndex =
                        find_string_entry(
                            ControllerInfo.GyroGainSetting.c_str(),
                            k_ds4_gyro_gain_setting_labels,
                            UI_ARRAYSIZE(k_ds4_gyro_gain_setting_labels));
                    if (ControllerInfo.GyroGainIndex == -1)
                    {
                        ControllerInfo.GyroGainSetting = k_ds4_gyro_gain_setting_labels[0];
                        ControllerInfo.GyroGainIndex = 0;
                    }
                }
                else
                {
                    ControllerInfo.GyroGainSetting = "";
                    ControllerInfo.GyroGainIndex = -1;
                }

                if (ControllerResponse.controller_type() == PSMoveProtocol::PSMOVE ||
                    ControllerResponse.controller_type() == PSMoveProtocol::PSNAVI ||
                    ControllerResponse.controller_type() == PSMoveProtocol::VIRTUALCONTROLLER)
                {
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                }
            }

            // Build a list of potential parent controllers for each navi controller
            // and assign the index current parent controller serial
            for (ControllerInfo &navi_info : thisPtr->m_controllerInfos)
            {
                if (navi_info.ControllerType != PSMController_Navi)
                    continue;

                for (ControllerInfo &psmove_info : thisPtr->m_controllerInfos)
                {
                    if (psmove_info.ControllerType != PSMController_Move)
                        continue;

                    if (navi_info.AssignedParentControllerSerial.length() > 0 &&
                        navi_info.AssignedParentControllerSerial == psmove_info.DeviceSerial)
                    {
                        navi_info.AssignedParentControllerIndex= static_cast<int>(navi_info.PotentialParentControllerSerials.size());
                    }

                    navi_info.PotentialParentControllerSerials.push_back(psmove_info.DeviceSerial);
                }
            }

			// Remove any USB controller entries that have a corresponding bluetooth controller entry
			// This means the controller is already paired via bluetooth
			auto iter = thisPtr->m_controllerInfos.begin();
			while (iter != thisPtr->m_controllerInfos.end())
			{
				bool bRemoveUSBControllerEntry= false;

				ControllerInfo &usb_psmove_info= *iter;
				if (!usb_psmove_info.IsBluetooth)
				{
					for (const ControllerInfo &bluetooth_psmove_info : thisPtr->m_controllerInfos)
					{
						if (bluetooth_psmove_info.IsBluetooth &&
							bluetooth_psmove_info.DeviceSerial == usb_psmove_info.DeviceSerial)
						{
							bRemoveUSBControllerEntry= true;
							break;
						}
					}
				}

				if (bRemoveUSBControllerEntry)
				{
					iter= thisPtr->m_controllerInfos.erase(iter);
				}
				else
				{
					++iter;
				}
			}

			// Find the index of the first non bluetooth PSMove controller, if any
			int first_usb_psmove_index= -1;
			int list_index= 0;
			for (auto it= thisPtr->m_controllerInfos.begin(); it != thisPtr->m_controllerInfos.end(); ++it)
			{
				if (!it->IsBluetooth && it->ControllerType == PSMController_Move)
				{
					first_usb_psmove_index= list_index;
					break;
				}

				++list_index;
			}

			// Determine which controller should be selected
			if (first_usb_psmove_index != -1)
			{
				thisPtr->m_selectedControllerIndex= first_usb_psmove_index;
			}
			else
			{
				if (oldSelectedControllerIndex != -1)
				{
					int controllerCount= static_cast<int>(thisPtr->m_controllerInfos.size());

					// Maintain the same position in the list if possible
					if (controllerCount > 0)
					{
						thisPtr->m_selectedControllerIndex= 
							(oldSelectedControllerIndex < controllerCount) ? oldSelectedControllerIndex : controllerCount-1;
					}
					else
					{
						thisPtr->m_selectedControllerIndex= -1;
					}
				}
				else
				{
					thisPtr->m_selectedControllerIndex= (thisPtr->m_controllerInfos.size() > 0) ? 0 : -1;
				}
			}

			for (PSMControllerID controller_id : thisPtr->m_controllersStreams)
			{
				PSMRequestID request_id;
				PSM_StopControllerDataStreamAsync(controller_id, &request_id);
				PSM_EatResponse(request_id);

				PSM_FreeControllerListener(controller_id);
			}

			thisPtr->m_controllersStreams.clear();

			for (auto it = thisPtr->m_controllerInfos.begin(); it != thisPtr->m_controllerInfos.end(); ++it)
			{
				unsigned int data_stream_flags =
					PSMControllerDataStreamFlags::PSMStreamFlags_defaultStreamOptions;
				
				PSM_AllocateControllerListener(it->ControllerID);

				PSMRequestID request_id;
				PSM_StartControllerDataStreamAsync(it->ControllerID, data_stream_flags, &request_id);
				PSM_EatResponse(request_id);

				thisPtr->m_controllersStreams.push_back(it->ControllerID);
			}

            thisPtr->m_menuState= AppStage_ControllerSettings::idle;
        } break;

        case PSMResult_Error:
        case PSMResult_Canceled:
        case PSMResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_ControllerSettings::failedControllerListRequest;
        } break;
    }
}

int AppStage_ControllerSettings::find_controller_id_by_serial(std::string controller_serial) const
{
    int ControllerID= -1;

    for (const ControllerInfo &info : m_controllerInfos)
    {
        if (info.DeviceSerial == controller_serial)
        {
            ControllerID= info.ControllerID;
            break;
        }
    }

    return ControllerID;
}

void AppStage_ControllerSettings::request_set_controller_tracking_color_id(
    int ControllerID,
    PSMTrackingColorType tracking_color_type)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_LED_TRACKING_COLOR);
    request->mutable_set_led_tracking_color_request()->set_controller_id(ControllerID);
    request->mutable_set_led_tracking_color_request()->set_color_type(
        static_cast<PSMoveProtocol::TrackingColorType>(tracking_color_type));

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_ControllerSettings::request_set_parent_controller_id(
	int ControllerID,
	int ParentControllerID)
{
	if (ControllerID != -1 && ParentControllerID != -1)
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_ATTACHED_CONTROLLER);
		request->mutable_request_set_attached_controller()->set_child_controller_id(ControllerID);
		request->mutable_request_set_attached_controller()->set_parent_controller_id(ParentControllerID);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_EatResponse(request_id);
	}
}

void AppStage_ControllerSettings::request_set_controller_offsets(
	int ControllerID,
	OffsetSettings offset_settings)
{
	if (ControllerID != -1)
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_CONTROLLER_OFFSETS);

		PSMoveProtocol::Request_RequestSetControllerOffsets *mutable_request_set_controller_offsets = request->mutable_request_set_controller_offsets();
		PSMoveProtocol::Euler *mutable_offset_orientation = mutable_request_set_controller_offsets->mutable_offset_orientation();
		PSMoveProtocol::Euler *mutable_offset_world_orientation = mutable_request_set_controller_offsets->mutable_offset_world_orientation();
		PSMoveProtocol::Position *mutable_offset_position = mutable_request_set_controller_offsets->mutable_offset_position();
		PSMoveProtocol::Position *mutable_offset_scale = mutable_request_set_controller_offsets->mutable_offset_scale();

		request->mutable_request_set_controller_offsets()->set_controller_id(ControllerID);
		mutable_offset_orientation->set_x(offset_settings.offset_orientation.x);
		mutable_offset_orientation->set_y(offset_settings.offset_orientation.y);
		mutable_offset_orientation->set_z(offset_settings.offset_orientation.z);
		mutable_offset_world_orientation->set_x(offset_settings.offset_world_orientation.x);
		mutable_offset_world_orientation->set_y(offset_settings.offset_world_orientation.y);
		mutable_offset_world_orientation->set_z(offset_settings.offset_world_orientation.z);
		mutable_offset_position->set_x(offset_settings.offset_position.x);
		mutable_offset_position->set_y(offset_settings.offset_position.y);
		mutable_offset_position->set_z(offset_settings.offset_position.z);
		mutable_offset_scale->set_x(offset_settings.offset_scale.x);
		mutable_offset_scale->set_y(offset_settings.offset_scale.y);
		mutable_offset_scale->set_z(offset_settings.offset_scale.z);
		request->mutable_request_set_controller_offsets()->set_offset_magnetometer(offset_settings.offset_magnetometer);

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_EatResponse(request_id);
	}
}

void AppStage_ControllerSettings::show_position_filter_tooltip(const std::string name)
{
	if (name == "PassThru")
	{
		ImGui::SetTooltip(
			"Direct pass through of optical position.\n"
			"The most responsive position filter but does not smooth optical jitter."
		);
	}
	else if (name == "LowPassOptical")
	{
		ImGui::SetTooltip(
			"Optical smoothing filter using distance.\n"
			"Smooths smaller movements within short distances to reduce position jitter,\n"
			"but behaves like PassThru on larger quicker movements.\n"
			"(Recommended)"
		);
	}
	else if (name == "LowPassIMU")
	{
		ImGui::SetTooltip(
			"Predictive smoothing filter using device accelerometer.\n"
			"Uses the device's accelerometer to predict and smooth optical movement."
		);
	}
	else if (name == "LowPassExponential")
	{
		ImGui::SetTooltip(
			"Optical smoothing filter using exponential curve.\n"
			"Reduces optical jitter greatly but can also causes springy tracking and over prediction."
		);
	}
	else if (name == "ComplimentaryOpticalIMU")
	{
		ImGui::SetTooltip(
			"Optical smoothing filter using variance curve and IMU for smoothing.\n"
			"Smooths optical tracking and reduces optical noise by tracker projection, distance and IMU.\n"
			"Requires calibration.\n"
			"(Use 'Calibrate Optical Noise' to calibrate)"
		);
	}
	else if (name == "PositionKalman")
	{
		ImGui::SetTooltip(
			"Optical smoothing filter using kalman and IMU.\n"
			"Smooths optical tracking and reduces optical noise by tracker projection, distance and IMU.\n"
			"Requires calibration.\n"
			"(Use 'Calibrate Optical Noise' to calibrate / Experimental)"
		);
	}
	else if (name == "PositionExternalAttachment")
	{
		ImGui::SetTooltip(
			"Parents this controller to another and overwrites the optical tracking behavior.\n"
			"It's recommended to turn off 'Enable Optical Tracking' when using this filter.\n"
			"(Requires 'PSMoveServiceEx Virtual Device Manager')"
		);
	}
}

void AppStage_ControllerSettings::show_orientation_filter_tooltip(const std::string name)
{
	if (name == "PassThru")
	{
		ImGui::SetTooltip(
			"Direct pass through optical orientation filter.\n"
			"Only works with Morpheus HMDs and DualShock4 controllers.\n"
			"[Optical]"
		);
	}
	else if (name == "MadgwickARG")
	{
		ImGui::SetTooltip(
			"IMU orientation filter using madgwick.\n"
			"[Gyro]"
		);
	}
	else if (name == "MadgwickMARG")
	{
		ImGui::SetTooltip(
			"IMU orientation filter using madgwick.\n"
			"[Gyro; Accelerometer; Magnetometer]"
		);
	}
	else if (name == "ComplementaryMARG")
	{
		ImGui::SetTooltip(
			"Simple IMU orientation filter.\n"
			"[Gyro; Accelerometer; Magnetometer]\n"
			"(Recommended)"
		);
	}
	else if (name == "ComplementaryOpticalARG")
	{
		ImGui::SetTooltip(
			"Optical orientation filter using variance curve and madgwick.\n"
			"Smooths optical orintation and reduces optical orintation noise by tracker projection and distance.\n"
			"Only works with Morpheus HMDs and DualShock4 controllers.\n"
			"Requires calibration.\n"
			"[Optical; Gyro]\n"
			"(Use 'Calibrate Optical Noise' to calibrate)"
		);
	}
	else if (name == "OrientationKalman")
	{
		ImGui::SetTooltip(
			"Optical orientation filter using kalman.\n"
			"Smooths optical orintation and reduces optical orintation noise by tracker projection and distance.\n"
			"Requires calibration.\n"
			"[Optical; Gyro; Accelerometer; Magnetometer]\n"
			"(Use 'Calibrate Optical Noise' to calibrate / Experimental)"
		);
	}
	else if (name == "OrientationExternal")
	{
		ImGui::SetTooltip(
			"Allows external source for orientation data (e.g. OwOTrack, SlimeVR).\n"
			"(Requires 'PSMoveServiceEx Virtual Device Manager')"
		);
	}
}

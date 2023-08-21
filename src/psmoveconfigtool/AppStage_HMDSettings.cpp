//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_HMDAccelerometerCalibration.h"
#include "AppStage_HMDGyroscopeCalibration.h"
#include "AppStage_HMDModelCalibration.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "MathUtility.h"
#include "UIConstants.h"
#include "PSMoveProtocolInterface.h"
#include "PSMoveProtocol.pb.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_HMDSettings::APP_STAGE_NAME= "HMDSettings";

//-- constants -----
const int k_default_hmd_position_filter_index = 1; // LowPassOptical
const int k_default_morpheus_position_filter_index = 1; // LowPassOptical
const int k_default_morpheus_orientation_filter_index = 1; // MadgwickARG

const char* k_hmd_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "PositionKalman" };
const char* k_morpheus_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "OrientationKalman" };

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
AppStage_HMDSettings::AppStage_HMDSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_HMDSettings::inactive)
    , m_selectedHmdIndex(-1)
{ }

void AppStage_HMDSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
    m_selectedHmdIndex = -1;

    request_hmd_list();
}

void AppStage_HMDSettings::exit()
{
    m_menuState = AppStage_HMDSettings::inactive;
}

void AppStage_HMDSettings::update()
{
}
    
void AppStage_HMDSettings::render()
{
    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
        if (m_selectedHmdIndex >= 0)
        {
            const HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            // Display the tracking color being used for the controller
            glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

            switch (hmdInfo.TrackingColorType)
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

            switch (hmdInfo.HmdType)
            {
            case PSMoveProtocol::Morpheus:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawMorpheusModel(scale3);
                } break;
            case PSMoveProtocol::VirtualHMD:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawVirtualHMDModel(scale3, bulb_color);
                } break;
            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eHmdMenuState::pendingHmdListRequest:
    case eHmdMenuState::failedHmdListRequest:
        {
        } break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDSettings::renderUI()
{
    const char *k_window_title = "HMD Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eHmdMenuState::idle:
    {
		static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

        ImGui::SetNextWindowPosCenter();
		ImGui::SetNextWindowSize(ImVec2(550, fminf(lastWindowVec.y + 32.f, ImGui::GetIO().DisplaySize.y - 32)));
        ImGui::Begin(k_window_title, nullptr, window_flags & ~ImGuiWindowFlags_NoScrollbar);
		ImGui::BeginGroup();
		{
			if (m_hmdInfos.size() > 0)
			{
				HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

				if (m_selectedHmdIndex > 0)
				{
					if (ImGui::Button(" < ##HMDIndex"))
					{
						--m_selectedHmdIndex;
					}
					ImGui::SameLine();
				}
				else
				{
					ImGui::Button(" < ##HMDIndex");
				}
				ImGui::SameLine();
				if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
				{
					if (ImGui::Button(" > ##HMDIndex"))
					{
						++m_selectedHmdIndex;
					}
				}
				else
				{
					ImGui::Button(" > ##HMDIndex");
				}
				ImGui::SameLine();
				ImGui::Text("HMD: %d", m_selectedHmdIndex);

				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##InfoChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						ImGui::Text("HMD Information:");
						ImGui::Separator();

						ImGui::BulletText("HMD ID: %d", hmdInfo.HmdID);

						if (hmdInfo.HmdType == AppStage_HMDSettings::Morpheus)
						{
							switch (hmdInfo.TrackingColorType)
							{
							case PSMTrackingColorType_Magenta:
								ImGui::BulletText("Tracking Color: Magenta");
								break;
							case PSMTrackingColorType_Cyan:
								ImGui::BulletText("Tracking Color: Cyan");
								break;
							case PSMTrackingColorType_Yellow:
								ImGui::BulletText("Tracking Color: Yellow");
								break;
							case PSMTrackingColorType_Red:
								ImGui::BulletText("Tracking Color: Red");
								break;
							case PSMTrackingColorType_Green:
								ImGui::BulletText("Tracking Color: Green");
								break;
							case PSMTrackingColorType_Blue:
								ImGui::BulletText("Tracking Color: Blue");
								break;
							case PSMTrackingColorType_Custom0:
								ImGui::BulletText("Tracking Color: Custom0");
								break;
							case PSMTrackingColorType_Custom1:
								ImGui::BulletText("Tracking Color: Custom1");
								break;
							case PSMTrackingColorType_Custom2:
								ImGui::BulletText("Tracking Color: Custom2");
								break;
							case PSMTrackingColorType_Custom3:
								ImGui::BulletText("Tracking Color: Custom3");
								break;
							case PSMTrackingColorType_Custom4:
								ImGui::BulletText("Tracking Color: Custom4");
								break;
							case PSMTrackingColorType_Custom5:
								ImGui::BulletText("Tracking Color: Custom5");
								break;
							case PSMTrackingColorType_Custom6:
								ImGui::BulletText("Tracking Color: Custom6");
								break;
							case PSMTrackingColorType_Custom7:
								ImGui::BulletText("Tracking Color: Custom7");
								break;
							case PSMTrackingColorType_Custom8:
								ImGui::BulletText("Tracking Color: Custom8");
								break;
							case PSMTrackingColorType_Custom9:
								ImGui::BulletText("Tracking Color: Custom9");
								break;
							}
						}

						switch (hmdInfo.HmdType)
						{
						case AppStage_HMDSettings::Morpheus:
						{
							ImGui::BulletText("HMD Type: Morpheus");
							ImGui::BulletText("Device Path: ");
							ImGui::SameLine();
							ImGui::TextWrapped("%s", hmdInfo.DevicePath.c_str());
						} break;
						case AppStage_HMDSettings::VirtualHMD:
						{
							ImGui::BulletText("HMD Type: VirtualHMD");
						} break;
						default:
							assert(0 && "Unreachable");
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				// Combo box selection for hmd tracking color
				if (ImGui::CollapsingHeader("Settings", 0, true, false))
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##SettingsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						if (hmdInfo.HmdType == AppStage_HMDSettings::VirtualHMD)
						{
							int newTrackingColorType = hmdInfo.TrackingColorType;

							ImGui::PushItemWidth(195);
							if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0Custom0\0Custom1\0Custom2\0Custom3\0Custom4\0Custom5\0Custom6\0Custom7\0Custom8\0Custom9\0\0"))
							{
								hmdInfo.TrackingColorType = static_cast<PSMTrackingColorType>(newTrackingColorType);

								request_set_hmd_tracking_color_id(hmdInfo.HmdID, hmdInfo.TrackingColorType);

								// Re-request the controller list since the tracking colors could changed for other controllers
								request_hmd_list();
							}
							ImGui::PopItemWidth();
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
				{
					if (ImGui::CollapsingHeader("Filters", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##FiltersChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							ImGui::PushItemWidth(195);
							if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
							{
								hmdInfo.PositionFilterName = k_hmd_position_filter_names[hmdInfo.PositionFilterIndex];
								request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
							}
							if (ImGui::IsItemHovered())
							{
								show_position_filter_tooltip(hmdInfo.PositionFilterName);
							}

							if (ImGui::Combo("Orientation Filter", &hmdInfo.OrientationFilterIndex, k_morpheus_orientation_filter_names, UI_ARRAYSIZE(k_morpheus_orientation_filter_names)))
							{
								hmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[hmdInfo.OrientationFilterIndex];
								request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
							}
							if (ImGui::IsItemHovered())
							{
								show_orientation_filter_tooltip(hmdInfo.OrientationFilterName);
							}

							ImGui::ProgressBar(hmdInfo.PredictionTime / k_max_hmd_prediction_time, ImVec2(195.f - 55.f, 0.f), " ");
							ImGui::SameLine();
							ImGui::PushItemWidth(96);
							if (ImGui::InputFloat("Prediction Time (ms)##PredictionTime", &hmdInfo.PredictionTime, 0.005f, 0.025f, 3))
							{
								hmdInfo.PredictionTime = clampf(hmdInfo.PredictionTime, 0.f, k_max_hmd_prediction_time);
								request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
							}
							ImGui::PopItemWidth();

							ImGui::ProgressBar(hmdInfo.AngPredictionTime / k_max_hmd_prediction_time, ImVec2(195.f - 55.f, 0.f), " ");
							ImGui::SameLine();
							ImGui::PushItemWidth(96);
							if (ImGui::InputFloat("Angular Prediction Time (ms)##AngPredictionTime", &hmdInfo.AngPredictionTime, 0.005f, 0.025f, 3))
							{
								hmdInfo.AngPredictionTime = clampf(hmdInfo.AngPredictionTime, 0.f, k_max_hmd_prediction_time);
								request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.AngPredictionTime);
							}
							ImGui::PopItemWidth();

							ImGui::Separator();

							if (ImGui::Button("Reset Filter Defaults"))
							{
								hmdInfo.PredictionTime = 0.0f;
								hmdInfo.AngPredictionTime = 0.0f;
								hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
								hmdInfo.OrientationFilterIndex = k_default_morpheus_position_filter_index;
								hmdInfo.PositionFilterName = k_hmd_position_filter_names[k_default_hmd_position_filter_index];
								hmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[k_default_morpheus_position_filter_index];
								request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
								request_set_hmd_angular_prediction(hmdInfo.HmdID, hmdInfo.AngPredictionTime);
								request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
								request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
							}
							ImGui::PopItemWidth();
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}
				}
				else if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::VirtualHMD)
				{
					if (ImGui::CollapsingHeader("Filters", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##FiltersChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							ImGui::PushItemWidth(195);
							if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
							{
								hmdInfo.PositionFilterName = k_hmd_position_filter_names[hmdInfo.PositionFilterIndex];
								request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
							}
							if (ImGui::IsItemHovered())
							{
								show_position_filter_tooltip(hmdInfo.PositionFilterName);
							}

							ImGui::ProgressBar(hmdInfo.PredictionTime / k_max_hmd_prediction_time, ImVec2(195.f - 55.f, 0.f), " ");
							ImGui::SameLine();
							ImGui::PushItemWidth(96);
							if (ImGui::InputFloat("Prediction Time (ms)##PredictionTime", &hmdInfo.PredictionTime, 0.005f, 0.025f, 3))
							{
								hmdInfo.PredictionTime = clampf(hmdInfo.PredictionTime, 0.f, k_max_hmd_prediction_time);
								request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
							}
							ImGui::PopItemWidth();

							if (ImGui::Button("Reset Filter Defaults"))
							{
								hmdInfo.PredictionTime = 0.0f;
								hmdInfo.AngPredictionTime = 0.0f;
								hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
								hmdInfo.PositionFilterName = k_hmd_position_filter_names[k_default_hmd_position_filter_index];
								request_set_hmd_prediction(hmdInfo.HmdID, hmdInfo.PredictionTime);
								request_set_hmd_angular_prediction(hmdInfo.HmdID, hmdInfo.AngPredictionTime);
								request_set_position_filter(hmdInfo.HmdID, hmdInfo.PositionFilterName);
								request_set_orientation_filter(hmdInfo.HmdID, hmdInfo.OrientationFilterName);
							}
							ImGui::PopItemWidth();
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}
				}

				if (ImGui::CollapsingHeader("Filter Settings", 0, true, false))
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##FilterSettingsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
					ImGui::BeginGroup();
					{
						bool request_offset = false;
						bool settings_shown = false;

						if (hmdInfo.PositionFilterName == "PassThru" ||
							hmdInfo.PositionFilterName == "LowPassOptical")
						{
							settings_shown = true;

							ImGui::Text("Velocity Smoothing Factor: ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							float filter_velocity_smoothing_factor = hmdInfo.FilterVelocitySmoothingFactor;
							if (ImGui::InputFloat("##VelocitySmoothingFactor", &filter_velocity_smoothing_factor, 0.01f, 0.05f, 2))
							{
								hmdInfo.FilterVelocitySmoothingFactor = clampf(filter_velocity_smoothing_factor, 0.0f, 1.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();

							if (ImGui::IsItemHovered())
							{
								ImGui::SetTooltip(
									"The amount of velocity smoothing applied determines the reduction\n"
									"of motion jitter when prediction is applied.\n"
									"However, too low values (more smoothing) can lead to prediction latency and creates a rubberbanding effect.\n"
									"Using too high values (less smoothing) can result in a rough and erratic motion."
								);
							}
						}

						if (hmdInfo.PositionFilterName == "LowPassOptical")
						{
							settings_shown = true;

							ImGui::Text("Position Smoothing Distance: ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##LowPassOpticalSmoothingDistance", &hmdInfo.FilterLowPassOpticalDistance, 1.f, 5.f, 2))
							{
								hmdInfo.FilterLowPassOpticalDistance = clampf(hmdInfo.FilterLowPassOpticalDistance, 1.0f, (1 << 16));

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Position Smoothing Power (%%): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							float filter_lowpassoptical_smoothing = (1.f - hmdInfo.FilterLowPassOpticalSmoothing) * 100.f;
							if (ImGui::InputFloat("##LowPassOpticalSmoothingPower", &filter_lowpassoptical_smoothing, 1.f, 5.f, 2))
							{
								hmdInfo.FilterLowPassOpticalSmoothing = clampf(1.f - (filter_lowpassoptical_smoothing / 100.f), 0.1f, 1.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();
						}

						if (hmdInfo.OrientationFilterName == "MadgwickMARG" || hmdInfo.OrientationFilterName == "MadgwickARG")
						{
							settings_shown = true;

							ImGui::Text("Drift Correction Power (Beta): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							float filter_madgwick_beta = hmdInfo.FilterMadgwickBeta;
							if (ImGui::InputFloat("##MadgwickFilterMadgwickBeta", &filter_madgwick_beta, 0.01f, 0.05f, 2))
							{
								hmdInfo.FilterMadgwickBeta = clampf(filter_madgwick_beta, 0.0f, 1.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Use Stabilization: ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							bool filter_madgwick_stabilization = hmdInfo.FilterMadgwickStabilization;
							if (ImGui::Checkbox("##MadgwickFilterMadgwickStabilization", &filter_madgwick_stabilization))
							{
								hmdInfo.FilterMadgwickStabilization = filter_madgwick_stabilization;

								request_offset = true;
							}
							ImGui::PopItemWidth();


							if (ImGui::IsItemHovered())
							{
								ImGui::SetTooltip(
									"Stabilization will reduce orientation jitter\n"
									"when the head-mounted display is stable."
								);
							}

							if (hmdInfo.FilterMadgwickStabilization)
							{
								ImGui::Indent();
								{
									ImGui::Text("Minimum Drift Correction Power (Beta): ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									float filter_madgwick_stabilization_min_beta = hmdInfo.FilterMadgwickStabilizationMinBeta;
									if (ImGui::InputFloat("##MadgwickFilterMinimumBeta", &filter_madgwick_stabilization_min_beta, 0.01f, 0.05f, 2))
									{
										hmdInfo.FilterMadgwickStabilizationMinBeta = clampf(filter_madgwick_stabilization_min_beta, 0.f, 1.f);

										request_offset = true;
									}
									ImGui::PopItemWidth();
								}
								ImGui::Unindent();

								ImGui::Indent();
								{
									ImGui::Text("Beta Smoothing Factor: ");
									ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
									ImGui::PushItemWidth(120.f);
									float filter_madgwick_stabilization_smoothing_factor = hmdInfo.FilterMadgwickStabilizationSmoothingFactor;
									if (ImGui::InputFloat("##MadgwickFilterBetaSmoothingFactor", &filter_madgwick_stabilization_smoothing_factor, 0.01f, 0.05f, 2))
									{
										hmdInfo.FilterMadgwickStabilizationSmoothingFactor = clampf(filter_madgwick_stabilization_smoothing_factor, 0.0f, 1.f);

										request_offset = true;
									}
									ImGui::PopItemWidth();
								}
								ImGui::Unindent();
							}
						}

						if (!settings_shown)
						{
							ImGui::Text("There are no settings for these filters.");
						}

						ImGui::Separator();

						if (ImGui::Button("Reset Filter Settings Defaults"))
						{
							hmdInfo.FilterLowPassOpticalDistance = 10.f;
							hmdInfo.FilterLowPassOpticalSmoothing = 0.40f;
							hmdInfo.FilterMadgwickBeta = 0.5f;
							hmdInfo.FilterMadgwickStabilization = true;
							hmdInfo.FilterMadgwickStabilizationMinBeta = 0.02f;
							hmdInfo.FilterMadgwickStabilizationSmoothingFactor = 0.1f;
							hmdInfo.FilterVelocitySmoothingFactor = 0.25f;

							request_offset = true;
						}

						if (request_offset)
						{
							FilterSettings filterSettings;
							filterSettings.filter_lowpassoptical_distance = hmdInfo.FilterLowPassOpticalDistance;
							filterSettings.filter_lowpassoptical_smoothing = hmdInfo.FilterLowPassOpticalSmoothing;
							filterSettings.filter_madgwick_beta = hmdInfo.FilterMadgwickBeta;
							filterSettings.filter_madgwick_stabilization = hmdInfo.FilterMadgwickStabilization;
							filterSettings.filter_madgwick_stabilization_min_beta = hmdInfo.FilterMadgwickStabilizationMinBeta;
							filterSettings.filter_madgwick_stabilization_smoothing_factor = hmdInfo.FilterMadgwickStabilizationSmoothingFactor;
							filterSettings.filter_velocity_smoothing_factor = hmdInfo.FilterVelocitySmoothingFactor;

							request_set_hmd_filter_settings(hmdInfo.HmdID, filterSettings);
						}
					}
					ImGui::EndGroup();
					if (ImGui::IsItemVisible())
						lastChildVec = ImGui::GetItemRectSize();
					ImGui::EndChild();
				}

				if (hmdInfo.HmdType == AppStage_HMDSettings::eHMDType::Morpheus)
				{
					if (ImGui::CollapsingHeader("Calibration", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##CalibrationChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							if (ImGui::Button("Calibrate Gyroscope"))
							{
								m_app->getAppStage<AppStage_HMDGyroscopeCalibration>()->setBypassCalibrationFlag(false);
								m_app->setAppStage(AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME);
							}
							if (ImGui::Button("Calibrate Accelerometer"))
							{
								m_app->getAppStage<AppStage_HMDAccelerometerCalibration>()->setBypassCalibrationFlag(false);
								m_app->setAppStage(AppStage_HMDAccelerometerCalibration::APP_STAGE_NAME);
							}

							if (m_app->getIsLocalServer())
							{
								if (ImGui::Button("Calibrate LED Model"))
								{
									AppStage_HMDModelCalibration::enterStageAndCalibrate(m_app, m_selectedHmdIndex);
								}
							}
							else
							{
								ImGui::Button("Calibrate LED Model (Unavailable)");
								ImGui::Bullet();
								ImGui::SameLine();
								ImGui::PushTextWrapPos();
								ImGui::TextDisabled(
									"Calibrating the LED model requires a local server connection."
								);
								ImGui::PopTextWrapPos();
								ImGui::Spacing();
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}

					if (ImGui::CollapsingHeader("Tests", 0, true, false))
					{
						static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
						ImGui::BeginChild("##TestsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
						ImGui::BeginGroup();
						{
							if (ImGui::Button("Test Orientation"))
							{
								m_app->getAppStage<AppStage_HMDGyroscopeCalibration>()->setBypassCalibrationFlag(true);
								m_app->setAppStage(AppStage_HMDGyroscopeCalibration::APP_STAGE_NAME);
							}
							if (ImGui::Button("Test Accelerometer"))
							{
								m_app->getAppStage<AppStage_HMDAccelerometerCalibration>()->setBypassCalibrationFlag(true);
								m_app->setAppStage(AppStage_HMDAccelerometerCalibration::APP_STAGE_NAME);
							}
						}
						ImGui::EndGroup();
						if (ImGui::IsItemVisible())
							lastChildVec = ImGui::GetItemRectSize();
						ImGui::EndChild();
					}
				}

				if (ImGui::CollapsingHeader("Offsets", 0, true, false))
				{
					static ImVec2 lastChildVec = ImVec2(0.f, 4.f);
					ImGui::BeginChild("##OffsetsChild", ImVec2(0.f, lastChildVec.y + 16.f), true);
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
							if (ImGui::InputFloat("##LocalOffsetOrientationX", &hmdInfo.OffsetOrientation.x, 1.f, 5.f, 2))
							{
								while (hmdInfo.OffsetOrientation.x < -180.f)
									hmdInfo.OffsetOrientation.x += 360.f;
								while (hmdInfo.OffsetOrientation.x >= 180.f)
									hmdInfo.OffsetOrientation.x -= 360.f;

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Local Orientation Y (Yaw): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##LocalOffsetOrientationY", &hmdInfo.OffsetOrientation.y, 1.f, 5.f, 2))
							{
								while (hmdInfo.OffsetOrientation.y < -180.f)
									hmdInfo.OffsetOrientation.y += 360.f;
								while (hmdInfo.OffsetOrientation.y >= 180.f)
									hmdInfo.OffsetOrientation.y -= 360.f;

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Local Orientation Z (Pitch): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##LocalOffsetOrientationZ", &hmdInfo.OffsetOrientation.z, 1.f, 5.f, 2))
							{
								while (hmdInfo.OffsetOrientation.z < -180.f)
									hmdInfo.OffsetOrientation.z += 360.f;
								while (hmdInfo.OffsetOrientation.z >= 180.f)
									hmdInfo.OffsetOrientation.z -= 360.f;

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Separator();

							ImGui::Text("World Orientation X (Roll): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##WorldOffsetOrientationX", &hmdInfo.OffsetWorldOrientation.x, 1.f, 5.f, 2))
							{
								while (hmdInfo.OffsetWorldOrientation.x < -180.f)
									hmdInfo.OffsetWorldOrientation.x += 360.f;
								while (hmdInfo.OffsetWorldOrientation.x >= 180.f)
									hmdInfo.OffsetWorldOrientation.x -= 360.f;

								request_offset = true;
							}
							ImGui::PopItemWidth();
						}

						ImGui::Text("World Orientation Y (Yaw): ");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(120.f);
						if (ImGui::InputFloat("##WorldOffsetOrientationY", &hmdInfo.OffsetWorldOrientation.y, 1.f, 5.f, 2))
						{
							while (hmdInfo.OffsetWorldOrientation.y < -180.f)
								hmdInfo.OffsetWorldOrientation.y += 360.f;
							while (hmdInfo.OffsetWorldOrientation.y >= 180.f)
								hmdInfo.OffsetWorldOrientation.y -= 360.f;

							request_offset = true;
						}
						ImGui::PopItemWidth();

						if (iOffsetView == 1)
						{
							ImGui::Text("World Orientation Z (Pitch): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##WorldOffsetOrientationZ", &hmdInfo.OffsetWorldOrientation.z, 1.f, 5.f, 2))
							{
								while (hmdInfo.OffsetWorldOrientation.z < -180.f)
									hmdInfo.OffsetWorldOrientation.z += 360.f;
								while (hmdInfo.OffsetWorldOrientation.z >= 180.f)
									hmdInfo.OffsetWorldOrientation.z -= 360.f;

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Separator();

							ImGui::Text("Position X (Right): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetPositionX", &hmdInfo.OffsetPosition.x, 1.f, 5.f, 2))
							{
								hmdInfo.OffsetPosition.x = clampf(hmdInfo.OffsetPosition.x, -(1 << 16), (1 << 16));

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Position Y (Up): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetPositionY", &hmdInfo.OffsetPosition.y, 1.f, 5.f, 2))
							{
								hmdInfo.OffsetPosition.y = clampf(hmdInfo.OffsetPosition.y, -(1 << 16), (1 << 16));

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Position Z (Backward): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetPositionZ", &hmdInfo.OffsetPosition.z, 1.f, 5.f, 2))
							{
								hmdInfo.OffsetPosition.z = clampf(hmdInfo.OffsetPosition.z, -(1 << 16), (1 << 16));

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Separator();

							ImGui::Text("Scale X (Left/Right): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetScaleX", &hmdInfo.OffsetScale.x, 0.01f, 0.05f, 2))
							{
								hmdInfo.OffsetScale.x = clampf(hmdInfo.OffsetScale.x, 0.01f, 100.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Scale Y (Up/Down): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetScaleY", &hmdInfo.OffsetScale.y, 0.01f, 0.05f, 2))
							{
								hmdInfo.OffsetScale.y = clampf(hmdInfo.OffsetScale.y, 0.01f, 100.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();

							ImGui::Text("Scale Z (Forward/Backward): ");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(120.f);
							if (ImGui::InputFloat("##OffsetScaleZ", &hmdInfo.OffsetScale.z, 0.01f, 0.05f, 2))
							{
								hmdInfo.OffsetScale.z = clampf(hmdInfo.OffsetScale.z, 0.01f, 100.0f);

								request_offset = true;
							}
							ImGui::PopItemWidth();
						}

						ImGui::Separator();

						if (hmdInfo.OffsetPosition.x != 0.0f ||
							hmdInfo.OffsetPosition.y != 0.0f ||
							hmdInfo.OffsetPosition.z != 0.0f ||
							hmdInfo.OffsetScale.x != 1.0f ||
							hmdInfo.OffsetScale.y != 1.0f ||
							hmdInfo.OffsetScale.z != 1.0f)
						{
							ImGui::Bullet();
							ImGui::SameLine();
							ImGui::PushTextWrapPos();
							ImGui::TextDisabled(
								"HMD scale or position has been changed!\n"
								"Changing the scale or position can cause abnormal artifacts in pose previews!"
							);
							ImGui::PopTextWrapPos();

							ImGui::Separator();
						}

						if (ImGui::Button("Reset All"))
						{
							hmdInfo.OffsetOrientation.x = 0.f;
							hmdInfo.OffsetOrientation.y = 0.f;
							hmdInfo.OffsetOrientation.z = 0.f;
							hmdInfo.OffsetWorldOrientation.x = 0.f;
							hmdInfo.OffsetWorldOrientation.y = 0.f;
							hmdInfo.OffsetWorldOrientation.z = 0.f;
							hmdInfo.OffsetPosition.x = 0.f;
							hmdInfo.OffsetPosition.y = 0.f;
							hmdInfo.OffsetPosition.z = 0.f;
							hmdInfo.OffsetScale.x = 1.f;
							hmdInfo.OffsetScale.y = 1.f;
							hmdInfo.OffsetScale.z = 1.f;

							request_offset = true;
						}

						if (request_offset)
						{
							OffsetSettings offset;
							offset.offset_orientation = hmdInfo.OffsetOrientation;
							offset.offset_world_orientation = hmdInfo.OffsetWorldOrientation;
							offset.offset_position = hmdInfo.OffsetPosition;
							offset.offset_scale = hmdInfo.OffsetScale;

							request_set_hmd_offsets(hmdInfo.HmdID, offset);
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
				ImGui::Text("No connected HMDs found!");
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
    case eHmdMenuState::pendingHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for HMD list response...");

        ImGui::End();
    } break;
    case eHmdMenuState::failedHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get tracker list!");

        if (ImGui::Button("Retry"))
        {
            request_hmd_list();
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

bool AppStage_HMDSettings::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case PSMEventMessage::PSMEvent_hmdListUpdated:
        {
            bHandled = true;
            request_hmd_list();
        } break;
    }

    return bHandled;
}

void AppStage_HMDSettings::request_hmd_list()
{
    if (m_menuState != AppStage_HMDSettings::pendingHmdListRequest)
    {
        m_menuState = AppStage_HMDSettings::pendingHmdListRequest;
        m_selectedHmdIndex = -1;
        m_hmdInfos.clear();

        // Tell the psmove service that we we want a list of HMDs connected to this machine
        RequestPtr request(new PSMoveProtocol::Request());
        request->set_type(PSMoveProtocol::Request_RequestType_GET_HMD_LIST);

        PSMRequestID request_id;
        PSM_SendOpaqueRequest(&request, &request_id);
        PSM_RegisterCallback(request_id, AppStage_HMDSettings::handle_hmd_list_response, this);
    }
}

void AppStage_HMDSettings::request_set_orientation_filter(
    const int hmd_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_ORIENTATION_FILTER);

    request->mutable_request_set_hmd_orientation_filter()->set_hmd_id(hmd_id);
    request->mutable_request_set_hmd_orientation_filter()->set_orientation_filter(filter_name);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_position_filter(
    const int hmd_id,
    const std::string &filter_name)
{
    RequestPtr request(new PSMoveProtocol::Request());
    request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_POSITION_FILTER);

    request->mutable_request_set_hmd_position_filter()->set_hmd_id(hmd_id);
    request->mutable_request_set_hmd_position_filter()->set_position_filter(filter_name);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_hmd_prediction(const int hmd_id, float prediction_time)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_PREDICTION_TIME);

	PSMoveProtocol::Request_RequestSetHMDPredictionTime *calibration =
		request->mutable_request_set_hmd_prediction_time();

	calibration->set_hmd_id(hmd_id);
	calibration->set_prediction_time(prediction_time);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_hmd_angular_prediction(const int hmd_id, float prediction_time)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_ANG_PREDICTION_TIME);

	PSMoveProtocol::Request_RequestSetHmdOrientationPredictionTime *calibration =
		request->mutable_request_set_hmd_orientation_prediction_time();

	calibration->set_hmd_id(hmd_id);
	calibration->set_ang_prediction_time(prediction_time);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_hmd_tracking_color_id(
	int HmdID,
	PSMTrackingColorType tracking_color_type)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_LED_TRACKING_COLOR);
	request->mutable_set_hmd_led_tracking_color_request()->set_hmd_id(HmdID);
	request->mutable_set_hmd_led_tracking_color_request()->set_color_type(
		static_cast<PSMoveProtocol::TrackingColorType>(tracking_color_type));

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_hmd_filter_settings(
	const int HmdID,
	FilterSettings filterSettings
)
{
	RequestPtr request(new PSMoveProtocol::Request());
	request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_FILTER_SETTINGS);

	PSMoveProtocol::Request_RequestSetHmdFilterSettings *filter_settings =
		request->mutable_request_set_hmd_filter_settings();

	filter_settings->set_hmd_id(HmdID);
	filter_settings->set_filter_lowpassoptical_distance(filterSettings.filter_lowpassoptical_distance);
	filter_settings->set_filter_lowpassoptical_smoothing(filterSettings.filter_lowpassoptical_smoothing);
	filter_settings->set_filter_madgwick_beta(filterSettings.filter_madgwick_beta);
	filter_settings->set_filter_madgwick_stabilization(filterSettings.filter_madgwick_stabilization);
	filter_settings->set_filter_madgwick_stabilization_min_beta(filterSettings.filter_madgwick_stabilization_min_beta);
	filter_settings->set_filter_madgwick_stabilization_smoothing_factor(filterSettings.filter_madgwick_stabilization_smoothing_factor);
	filter_settings->set_filter_velocity_smoothing_factor(filterSettings.filter_velocity_smoothing_factor);

	PSMRequestID request_id;
	PSM_SendOpaqueRequest(&request, &request_id);
	PSM_EatResponse(request_id);
}

void AppStage_HMDSettings::request_set_hmd_offsets(
	int HmdID,
	OffsetSettings offset_settings)
{
	if (HmdID != -1)
	{
		RequestPtr request(new PSMoveProtocol::Request());
		request->set_type(PSMoveProtocol::Request_RequestType_SET_HMD_OFFSETS);

		PSMoveProtocol::Request_RequestSetHMDOffsets *mutable_request_set_hmd_offsets = request->mutable_request_set_hmd_offsets();
		PSMoveProtocol::Euler *mutable_offset_orientation = mutable_request_set_hmd_offsets->mutable_offset_orientation();
		PSMoveProtocol::Euler *mutable_offset_world_orientation = mutable_request_set_hmd_offsets->mutable_offset_world_orientation();
		PSMoveProtocol::Position *mutable_offset_position = mutable_request_set_hmd_offsets->mutable_offset_position();
		PSMoveProtocol::Position *mutable_offset_scale = mutable_request_set_hmd_offsets->mutable_offset_scale();

		request->mutable_request_set_hmd_offsets()->set_hmd_id(HmdID);
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

		PSMRequestID request_id;
		PSM_SendOpaqueRequest(&request, &request_id);
		PSM_EatResponse(request_id);
	}
}

void AppStage_HMDSettings::handle_hmd_list_response(
    const PSMResponseMessage *response,
    void *userdata)
{
    PSMResult ResultCode = response->result_code;
    PSMResponseHandle response_handle = response->opaque_response_handle;
    AppStage_HMDSettings *thisPtr = static_cast<AppStage_HMDSettings *>(userdata);

    switch (ResultCode)
    {
    case PSMResult_Success:
        {
            const PSMoveProtocol::Response *response = GET_PSMOVEPROTOCOL_RESPONSE(response_handle);

            for (int hmd_index = 0; hmd_index < response->result_hmd_list().hmd_entries_size(); ++hmd_index)
            {
                const auto &HmdResponse = response->result_hmd_list().hmd_entries(hmd_index);

                AppStage_HMDSettings::HMDInfo HmdInfo;

                HmdInfo.HmdID = HmdResponse.hmd_id();

                switch (HmdResponse.hmd_type())
                {
                case PSMoveProtocol::HMDType::Morpheus:
                    HmdInfo.HmdType = AppStage_HMDSettings::Morpheus;
                    break;
                case PSMoveProtocol::HMDType::VirtualHMD:
                    HmdInfo.HmdType = AppStage_HMDSettings::VirtualHMD;
                    break;
                default:
                    assert(0 && "unreachable");
                }

                HmdInfo.TrackingColorType = static_cast<PSMTrackingColorType>(HmdResponse.tracking_color_type());
                HmdInfo.DevicePath = HmdResponse.device_path();
				HmdInfo.PredictionTime = HmdResponse.prediction_time();
				HmdInfo.AngPredictionTime = HmdResponse.ang_prediction_time();
                HmdInfo.OrientationFilterName= HmdResponse.orientation_filter();
                HmdInfo.PositionFilterName = HmdResponse.position_filter();

				HmdInfo.OffsetOrientation.x = HmdResponse.offset_orientation().x();
				HmdInfo.OffsetOrientation.y = HmdResponse.offset_orientation().y();
				HmdInfo.OffsetOrientation.z = HmdResponse.offset_orientation().z();
				HmdInfo.OffsetWorldOrientation.x = HmdResponse.offset_world_orientation().x();
				HmdInfo.OffsetWorldOrientation.y = HmdResponse.offset_world_orientation().y();
				HmdInfo.OffsetWorldOrientation.z = HmdResponse.offset_world_orientation().z();
				HmdInfo.OffsetPosition.x = HmdResponse.offset_position().x();
				HmdInfo.OffsetPosition.y = HmdResponse.offset_position().y();
				HmdInfo.OffsetPosition.z = HmdResponse.offset_position().z();
				HmdInfo.OffsetScale.x = HmdResponse.offset_scale().x();
				HmdInfo.OffsetScale.y = HmdResponse.offset_scale().y();
				HmdInfo.OffsetScale.z = HmdResponse.offset_scale().z();

				HmdInfo.FilterLowPassOpticalDistance = HmdResponse.filter_lowpassoptical_distance();
				HmdInfo.FilterLowPassOpticalSmoothing = HmdResponse.filter_lowpassoptical_smoothing();
				HmdInfo.FilterMadgwickBeta = HmdResponse.filter_madgwick_beta();
				HmdInfo.FilterMadgwickStabilization = HmdResponse.filter_madgwick_stabilization();
				HmdInfo.FilterMadgwickStabilizationMinBeta = HmdResponse.filter_madgwick_stabilization_min_beta();
				HmdInfo.FilterMadgwickStabilizationSmoothingFactor = HmdResponse.filter_madgwick_stabilization_smoothing_factor();
				HmdInfo.FilterVelocitySmoothingFactor = HmdResponse.filter_velocity_smoothing_factor();

                if (HmdInfo.HmdType == AppStage_HMDSettings::Morpheus)
                {
                    HmdInfo.OrientationFilterIndex =
                        find_string_entry(
                            HmdInfo.OrientationFilterName.c_str(),
                            k_morpheus_orientation_filter_names,
                            UI_ARRAYSIZE(k_morpheus_orientation_filter_names));
                    if (HmdInfo.OrientationFilterIndex == -1)
                    {
                        HmdInfo.OrientationFilterName = k_morpheus_orientation_filter_names[0];
                        HmdInfo.OrientationFilterIndex = 0;
                    }
                }
                else
                {
                    HmdInfo.OrientationFilterName = "";
                    HmdInfo.OrientationFilterIndex = -1;
                }

                if (HmdInfo.HmdType == AppStage_HMDSettings::Morpheus ||
                    HmdInfo.HmdType == AppStage_HMDSettings::VirtualHMD)
                {
                    HmdInfo.PositionFilterIndex =
                        find_string_entry(
                            HmdInfo.PositionFilterName.c_str(),
                            k_hmd_position_filter_names,
                            UI_ARRAYSIZE(k_hmd_position_filter_names));
                    if (HmdInfo.PositionFilterIndex == -1)
                    {
                        HmdInfo.PositionFilterName = k_hmd_position_filter_names[0];
                        HmdInfo.PositionFilterIndex = 0;
                    }
                }
                else
                {
                    HmdInfo.PositionFilterName = "";
                    HmdInfo.PositionFilterIndex = -1;
                }

                thisPtr->m_hmdInfos.push_back(HmdInfo);
            }

            thisPtr->m_selectedHmdIndex = (thisPtr->m_hmdInfos.size() > 0) ? 0 : -1;
            thisPtr->m_menuState = AppStage_HMDSettings::idle;
        } break;

    case PSMResult_Error:
    case PSMResult_Canceled:
    case PSMResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_HMDSettings::failedHmdListRequest;
        } break;
    }
}


void AppStage_HMDSettings::show_position_filter_tooltip(const std::string name)
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
			"but behaves like PassThru on larger quicker movements."
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

void AppStage_HMDSettings::show_orientation_filter_tooltip(const std::string name)
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
			"Advanced IMU orientation filter using madgwick.\n"
			"[Gyroscope; Accelerometer]"
		);
	}
	else if (name == "MadgwickMARG")
	{
		ImGui::SetTooltip(
			"Advanced IMU orientation filter using madgwick algorithm.\n"
			"[Gyroscope; Accelerometer; Magnetometer]"
		);
	}
	else if (name == "ComplementaryMARG")
	{
		ImGui::SetTooltip(
			"Fast and simple IMU orientation filter.\n"
			"[Gyroscope; Accelerometer; Magnetometer]"
		);
	}
	else if (name == "ComplementaryOpticalARG")
	{
		ImGui::SetTooltip(
			"Optical orientation filter using variance curve and madgwick.\n"
			"Smooths optical orintation and reduces optical orintation noise by tracker projection and distance.\n"
			"Only works with Morpheus HMDs and DualShock4 controllers.\n"
			"Requires calibration.\n"
			"[Optical; Gyroscope]\n"
			"(Use 'Calibrate Optical Noise' to calibrate)"
		);
	}
	else if (name == "OrientationKalman")
	{
		ImGui::SetTooltip(
			"Optical orientation filter using kalman.\n"
			"Smooths optical orintation and reduces optical orintation noise by tracker projection and distance.\n"
			"Requires calibration.\n"
			"[Optical; Gyroscope; Accelerometer; Magnetometer]\n"
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

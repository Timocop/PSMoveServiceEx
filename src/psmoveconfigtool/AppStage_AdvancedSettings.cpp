//-- inludes -----
#include "AppStage_MainMenu.h"
#include "AppStage_AdvancedSettings.h"
#include "App.h"
#include "Camera.h"
#include "ProtocolVersion.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <iostream>

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
class Win32ServiceRestart 
{
public:
	BOOL psMoveServiceFound = FALSE;
	BOOL psMoveServiceAdminFound = FALSE;
	DWORD psMoveServiceId = 0;
	DWORD psMoveServiceAdminEId = 0;

	void CheckProcesses() 
	{
		psMoveServiceFound = FALSE;
		psMoveServiceAdminFound = FALSE;
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

					CloseHandle(hProcess);
				}

				if (stricmp(entry.szExeFile, "PSMoveServiceAdmin.exe") == 0)
				{
					psMoveServiceAdminEId = entry.th32ProcessID;
					HANDLE hProcess = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, entry.th32ProcessID);

					psMoveServiceAdminFound = TRUE;

					CloseHandle(hProcess);
				}

			}
		}

		CloseHandle(snapshot);
	}

	void RestartService() 
	{
		if (psMoveServiceId != 0)
		{
			char moduleFileName[MAXCHAR];
			HANDLE hProcess = OpenProcess(PROCESS_ALL_ACCESS, FALSE, psMoveServiceId);
			if (hProcess != NULL)
			{
				GetModuleFileNameEx(hProcess, NULL, moduleFileName, MAXCHAR);

				TerminateProcess(hProcess, 0);
				CloseHandle(hProcess);

				char drive[5];
				char dir[MAXCHAR];
				_splitpath_s(moduleFileName, drive, sizeof(drive), dir, sizeof(dir), NULL, 0, NULL, 0);

				std::string serviceDir = drive;
				serviceDir = serviceDir + dir;
				std::string servicePath = serviceDir + "PSMoveService.exe";

				SHELLEXECUTEINFO ShExecInfo;
				ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
				ShExecInfo.fMask = NULL;
				ShExecInfo.hwnd = NULL;
				ShExecInfo.lpVerb = NULL;
				ShExecInfo.lpFile = servicePath.c_str();
				ShExecInfo.lpParameters = NULL;
				ShExecInfo.lpDirectory = NULL;
				ShExecInfo.nShow = SW_NORMAL;
				ShExecInfo.hInstApp = NULL;

				BOOL create = ShellExecuteEx(&ShExecInfo);
			}
		}
	}
};

Win32ServiceRestart serviceRestart;
#endif


PSMoveConfig::PSMoveConfig(const std::string &fnamebase)
	: ConfigFileBase(fnamebase)
{
}

void 
PSMoveConfig::map_flatten(const boost::property_tree::ptree& pt, std::string key)
{
	std::string nkey;

	if (!key.empty() && pt.size() == 0)
	{
		cfg_map[key.c_str()] = pt.get_value<std::string>("");
	}

	if (!key.empty())
	{
		nkey = key + ".";
	}

	boost::property_tree::ptree::const_iterator end = pt.end();
	for (boost::property_tree::ptree::const_iterator it = pt.begin(); it != end; ++it)
	{
		map_flatten(it->second, nkey + it->first);

	}
}

const std::string
PSMoveConfig::getConfigPath()
{
	const char *homedir;
#ifdef _WIN32
	size_t homedir_buffer_req_size;
	char homedir_buffer[512];
	getenv_s(&homedir_buffer_req_size, homedir_buffer, "APPDATA");
	assert(homedir_buffer_req_size <= sizeof(homedir_buffer));
	homedir = homedir_buffer;
#else
	homedir = getenv("HOME");
	// if run as root, use system-wide data directory
	if (geteuid() == 0) {
		homedir = "/etc/psmoveservice";
	}
#endif

	boost::filesystem::path configpath(homedir);
	configpath /= "PSMoveService";
	boost::filesystem::create_directory(configpath);
	configpath /= ConfigFileBase + ".json";
	std::cout << "Config file name: " << configpath << std::endl;
	return configpath.string();
}

void
PSMoveConfig::save()
{
	boost::property_tree::write_json(getConfigPath(), config2ptree());
}

bool
PSMoveConfig::load()
{
	bool bLoadedOk = false;
	boost::property_tree::ptree pt;
	std::string configPath = getConfigPath();

	if (boost::filesystem::exists(configPath))
	{
		boost::property_tree::read_json(configPath, pt);
		ptree2config(pt);
		bLoadedOk = true;
	}

	return bLoadedOk;
}

const boost::property_tree::ptree
TrackerConfig::config2ptree()
{
	boost::property_tree::ptree pt;

	for (std::pair<std::string, std::string> p : cfg_map)
	{
		pt.put(p.first, p.second);
	}

	pt.put("virtual_tracker_count", virtual_tracker_count);
	pt.put("controller_position_smoothing", controller_position_smoothing);
	pt.put("controller_position_prediction", controller_position_prediction);
	pt.put("controller_position_prediction_history", controller_position_prediction_history);
	pt.put("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
	pt.put("optical_tracking_timeout", optical_tracking_timeout);
	pt.put("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	pt.put("tracker_sleep_ms", tracker_sleep_ms);

	pt.put("excluded_opposed_cameras", exclude_opposed_cameras);

	pt.put("min_valid_projection_area", min_valid_projection_area);
	pt.put("min_occluded_area_on_loss", min_occluded_area_on_loss);
	pt.put("min_points_in_contour", min_points_in_contour);
	pt.put("max_tracker_position_deviation", max_tracker_position_deviation);

	pt.put("disable_roi", disable_roi);
	pt.put("optimized_roi", optimized_roi);

	pt.put("global_forward_degrees", global_forward_degrees);

	return pt;
}

void
TrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	virtual_tracker_count = pt.get<int>("virtual_tracker_count", virtual_tracker_count);
	controller_position_smoothing = pt.get<float>("controller_position_smoothing", controller_position_smoothing);
	controller_position_prediction = pt.get<float>("controller_position_prediction", controller_position_prediction);
	controller_position_prediction_history = pt.get<int>("controller_position_prediction_history", controller_position_prediction_history);
	ignore_pose_from_one_tracker = pt.get<bool>("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
	optical_tracking_timeout = pt.get<int>("optical_tracking_timeout", optical_tracking_timeout);
	use_bgr_to_hsv_lookup_table = pt.get<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	tracker_sleep_ms = pt.get<int>("tracker_sleep_ms", tracker_sleep_ms);

	exclude_opposed_cameras = pt.get<bool>("excluded_opposed_cameras", exclude_opposed_cameras);

	min_valid_projection_area = pt.get<float>("min_valid_projection_area", min_valid_projection_area);
	min_occluded_area_on_loss = pt.get<float>("min_occluded_area_on_loss", min_occluded_area_on_loss);
	min_points_in_contour = pt.get<int>("min_points_in_contour", min_points_in_contour);
	max_tracker_position_deviation = pt.get<float>("max_tracker_position_deviation", max_tracker_position_deviation);

	disable_roi = pt.get<bool>("disable_roi", disable_roi);
	optimized_roi = pt.get<bool>("optimized_roi", optimized_roi);

	global_forward_degrees = pt.get<float>("global_forward_degrees", global_forward_degrees);
}

const boost::property_tree::ptree
ControllerConfig::config2ptree()
{
	boost::property_tree::ptree pt;

	for (std::pair<std::string, std::string> p : cfg_map)
	{
		pt.put(p.first, p.second);
	}

	pt.put("virtual_controller_count", virtual_controller_count);

	return pt;
}

void
ControllerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	virtual_controller_count = pt.get<int>("virtual_controller_count", virtual_controller_count);
}

const boost::property_tree::ptree
HMDConfig::config2ptree()
{
	boost::property_tree::ptree pt;

	for (std::pair<std::string, std::string> p : cfg_map)
	{
		pt.put(p.first, p.second);
	}

	pt.put("virtual_hmd_count", virtual_hmd_count);

	return pt;
}

void
HMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	virtual_hmd_count = pt.get<int>("virtual_hmd_count", virtual_hmd_count);
}

#ifdef _WIN32
class Win32Config {

public:
	void OpenConfigInExplorer()
	{
		std::string explorer = "explorer.exe";
		std::string config_path = getConfigPath();

		SHELLEXECUTEINFO ShExecInfo;
		ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
		ShExecInfo.fMask = NULL;
		ShExecInfo.hwnd = NULL;
		ShExecInfo.lpVerb = NULL;
		ShExecInfo.lpFile = explorer.c_str();
		ShExecInfo.lpParameters = config_path.c_str();
		ShExecInfo.lpDirectory = NULL;
		ShExecInfo.nShow = SW_NORMAL;
		ShExecInfo.hInstApp = NULL;

		BOOL create = ShellExecuteEx(&ShExecInfo);
	}

	void OpenConfigInEditor(std::string config) {
		std::string editor_path = "notepad.exe";
		std::string config_path = getConfigPath() + "\\" + config;

		SHELLEXECUTEINFO ShExecInfo;
		ShExecInfo.cbSize = sizeof(SHELLEXECUTEINFO);
		ShExecInfo.fMask = NULL;
		ShExecInfo.hwnd = NULL;
		ShExecInfo.lpVerb = NULL;
		ShExecInfo.lpFile = editor_path.c_str();
		ShExecInfo.lpParameters = config_path.c_str();
		ShExecInfo.lpDirectory = NULL;
		ShExecInfo.nShow = SW_NORMAL;
		ShExecInfo.hInstApp = NULL;

		BOOL create = ShellExecuteEx(&ShExecInfo);
	}

	const std::string
		getConfigPath()
	{
		const char *homedir;
#ifdef _WIN32
		size_t homedir_buffer_req_size;
		char homedir_buffer[512];
		getenv_s(&homedir_buffer_req_size, homedir_buffer, "APPDATA");
		assert(homedir_buffer_req_size <= sizeof(homedir_buffer));
		homedir = homedir_buffer;
#else
		homedir = getenv("HOME");
		// if run as root, use system-wide data directory
		if (geteuid() == 0) {
			homedir = "/etc/psmoveservice";
		}
#endif

		boost::filesystem::path configpath(homedir);
		configpath /= "PSMoveService";
		boost::filesystem::create_directory(configpath);
		return configpath.string();
	}
};

Win32Config configExec;
#endif

//-- statics ----
const char *AppStage_AdvancedSettings::APP_STAGE_NAME= "AdvancedSettings";

//-- public methods -----
AppStage_AdvancedSettings::AppStage_AdvancedSettings(App *app)
    : AppStage(app)
    , m_menuState(AppStage_AdvancedSettings::inactive)
{ }

bool AppStage_AdvancedSettings::init(int argc, char** argv)
{
    return true;
}

void AppStage_AdvancedSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

    // Only set the menu state if it hasn't been set already
    if (m_menuState == AppStage_AdvancedSettings::inactive)
    {
		//Load configs
		cfg_tracker = TrackerConfig();
		cfg_tracker.isLoaded = cfg_tracker.load();

		cfg_controller = ControllerConfig();
		cfg_controller.isLoaded = cfg_controller.load();

		cfg_hmd = HMDConfig();
		cfg_hmd.isLoaded = cfg_hmd.load();

		m_menuState = AppStage_AdvancedSettings::idle;
    }
}

void AppStage_AdvancedSettings::exit()
{
    // Upon normal exit, set the state to inactive
    m_menuState= AppStage_AdvancedSettings::inactive;
}

void AppStage_AdvancedSettings::renderUI()
{
	const char *k_window_title = "Advanced Settings";
	const ImGuiWindowFlags window_flags =
		ImGuiWindowFlags_ShowBorders |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoCollapse;

    switch(m_menuState)
    {
    case idle:
        {
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(600, 500));
			ImGui::Begin("Advanced Settings", nullptr, window_flags | ImGuiWindowFlags_MenuBar);

			// Tracker Manager Config
			if (ImGui::CollapsingHeader("Tracker Manager Config", 0, true, false))
			{
				{
					ImGui::Text("Virtual Trackers:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputInt("##VirtualTrackers", &cfg_tracker.virtual_tracker_count))
					{
						cfg_tracker.virtual_tracker_count = static_cast<int>(std::fmax(0, std::fmin(PSMOVESERVICE_MAX_TRACKER_COUNT, cfg_tracker.virtual_tracker_count)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"The number of trackers emulated in PSMoveService.\n"
							"Useful if you want to add your custom trackers that are not related to PlayStation Move."
						);
				}

				{
					ImGui::Text("Controller position smoothing:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##ControllerPositionSmoothing", &cfg_tracker.controller_position_smoothing, 0.05f, 0.1f, 2))
					{
						cfg_tracker.controller_position_smoothing = static_cast<float>(std::fmax(0.f, std::fmin(1.f, cfg_tracker.controller_position_smoothing)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"The amount of position smoothing all controllers will have.\n"
							"Should be a value between 0.00 and 1.00. Where 0.00 is no smoothing and 1.00 is maximum smoothing.\n"
							"Beware that smoothing can cause input lag. Adjust 'Controller position prediction' to reduce input lag.\n"
							"(The default value is 0)"
						);
				}

				{
					ImGui::Text("Controller position prediction:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##ControllerPositionPrediction", &cfg_tracker.controller_position_prediction, 0.05f, 0.1f, 2))
					{
						cfg_tracker.controller_position_prediction = static_cast<float>(std::fmax(0.f, std::fmin(100.f, cfg_tracker.controller_position_prediction)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"The amount of position prediction all controllers will have.\n"
							"Should be a value between 0.00 and 1.00. Whereas 0.00 is no prediction and 1.00 is very high prediction.\n"
							"The value can be set higher than 1.00 but you might experience over-prediction.\n"
							"Use 'Controller position smoothing' to remove position jitter when prediction is enabled.\n"
							"(The default value is 0)"
						);
				}

				ImGui::Indent();
				{
					ImGui::Text("Controller position prediction history:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputInt("##ControllerPositionPredictionHistory", &cfg_tracker.controller_position_prediction_history))
					{
						cfg_tracker.controller_position_prediction_history = static_cast<int>(std::fmax(1, std::fmin(50, cfg_tracker.controller_position_prediction_history)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"How many previous positions should be saved for calculations.\n"
							"Lower values makes prediction respond faster and higher slower.\n"
							"(The default value is 5)"
						);
				}
				ImGui::Unindent();

				{
					ImGui::Text("Ignore pose from one tracker:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::Checkbox("##IgnorePoseFromOneTracker", &cfg_tracker.ignore_pose_from_one_tracker);

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Ignores poses from one tracker and enforces triangulation from at least 2 trackers.\n"
							"This will greatly improve training quality and should always be enabled.\n"
							"This setting will be ignored if only one tracker is available.\n"
							"(The default value is TRUE)"
						);
				}

				{
					ImGui::Text("Optical tracking timeout:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputInt("##OpticalTrackingTimeout", &cfg_tracker.optical_tracking_timeout))
					{
						cfg_tracker.optical_tracking_timeout = static_cast<int>(std::fmax(0, std::fmin(99999, cfg_tracker.optical_tracking_timeout)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"When tracking is lost wait this amount of time until tracking is resumed.\n"
							"Do not set this value lower than 100ms otherwise you will experience random tracker timeouts.\n"
							"(The default value is 100)"
						);
				}

				{
					ImGui::Text("Tracker sleep (ms):");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputInt("##TrackerSleep", &cfg_tracker.tracker_sleep_ms))
					{
						cfg_tracker.tracker_sleep_ms = static_cast<int>(std::fmax(0, std::fmin(99999, cfg_tracker.tracker_sleep_ms)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"(The default value is 1)"
						);
				}

				{
					ImGui::Text("Use BGR to HSV lookup table:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::Checkbox("##UseBgrToHsvLookupTable", &cfg_tracker.use_bgr_to_hsv_lookup_table);

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"(The default value is TRUE)"
						);
				}

				{
					ImGui::Text("Exclude opposed trackers:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::Checkbox("##ExcludeOpposedTrackers", &cfg_tracker.exclude_opposed_cameras);

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Exclude triangulations from trackers that are facing each other.\n"
							"Enabling this can help get better triangulations between trackers\n"
							"and may result in better tracking but also increases potential tracking loss\n"
							"due to trackers being excluded.\n"
							"This is only good if you have 4 or more trackers.\n"
							"(The default value is FALSE)"
						);
				}

				{
					ImGui::Text("Minimum valid projection area:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##MinimumValidProjectionArea", &cfg_tracker.min_valid_projection_area, 1.f, 4.f, 2))
					{
						cfg_tracker.min_valid_projection_area = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.min_valid_projection_area)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Projection areas smaller than this will not be valid and will not be tracked.\n"
							"Using smaller values can help track tracking lights better on further distances\n"
							"but can also introduce more position jitter.\n"
							"(The legacy value is 16)\n"
							"(The default value is 6)"
						);
				}

				{
					ImGui::Text("Minimum occluded area on tracking loss:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##MinimumOccludedAreaOnLoss", &cfg_tracker.min_occluded_area_on_loss, 1.f, 4.f, 2))
					{
						cfg_tracker.min_occluded_area_on_loss = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.min_occluded_area_on_loss)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Occlusion areas are created when trackers lose their tracking projection.\n"
							"The tracker will not re-gain its projection if the projection is near the tracker's occlusion area.\n"
							"This will help avoid position jitter on continuous tracking loss.\n"
							"(The default value is 4)"
						);
				}

				{
					ImGui::Text("Minimum points in contour:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputInt("##MinimumPointsInContour", &cfg_tracker.min_points_in_contour))
					{
						cfg_tracker.min_points_in_contour = static_cast<int>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.min_points_in_contour)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"How many points a contour can have until it counts as a valid projection.\n"
							"A minimum of 4 points requires the projection to be a square and more than 6 points a sphere.\n"
							"If the minimum is 1 or lower then any pixel will count as valid.\n"
							"(The legacy value is 6)\n"
							"(The default value is 4)"
						);
				}

				{
					ImGui::Text("Maximum tracker position deviation:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##MaximumTrackerPositionDeviation", &cfg_tracker.max_tracker_position_deviation, 1.f, 4.f, 2))
					{
						cfg_tracker.max_tracker_position_deviation = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.max_tracker_position_deviation)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Trackers that deviate their triangulation position too much from other trackers will be disregarded.\n"
							"This will avoid trackers getting stuck on random color noise or other controllers.\n"
							"(The default value is 12)"
						);
				}

				{
					ImGui::Text("Enable ROI (region of interest):");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					bool roiEnabled = !cfg_tracker.disable_roi;
					if (ImGui::Checkbox("##EnableROI", &roiEnabled))
					{
						cfg_tracker.disable_roi = !roiEnabled;
					}

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Enables ROI (region of interest) which only analyzes parts around the target projection.\n"
							"This reduces CPU usage when controllers and head mount devices are visible to the tracker.\n"
							"However, you may experience tracking loss on rapid movement when running trackers on low FPS.\n"
							"(The default value is TRUE)"
						);
				}

				ImGui::Indent();
				{
					ImGui::Text("Optimized ROI (region of interest):");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::Checkbox("##OptimizedROI", &cfg_tracker.optimized_roi);

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Optimizes ROI (region of interest) for controllers that are not visible to the tracker\n"
							"to reduce CPU usage even more.\n"
							"(The default value is TRUE)"
						);
				}
				ImGui::Unindent();

				{
					ImGui::Text("Global forward degrees:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					if (ImGui::InputFloat("##GlobalForwardDegrees", &cfg_tracker.global_forward_degrees, 1.f, 4.f, 2))
					{
						cfg_tracker.global_forward_degrees = static_cast<float>(std::fmax(0.f, std::fmin(360.f, cfg_tracker.global_forward_degrees)));
					}
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"(The default value is 270)"
						);
				}
			}

			// Controller Manager Config
			if (ImGui::CollapsingHeader("Controller Manager Config", 0, true, false))
			{
				ImGui::Text("Virtual Controllers:");
				ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
				ImGui::PushItemWidth(100.f);
				if (ImGui::InputInt("##VirtualControllers", &cfg_controller.virtual_controller_count))
				{
					cfg_controller.virtual_controller_count = static_cast<int>(std::fmax(0, std::fmin(PSMOVESERVICE_MAX_CONTROLLER_COUNT, cfg_controller.virtual_controller_count)));
				}
				ImGui::PopItemWidth();

				if (ImGui::IsItemHovered())
					ImGui::SetTooltip(
						"The number of controllers emulated in PSMoveService.\n"
						"Useful if you want to add your custom controllers that are not related to PlayStation Move."
					);
				
			}

			// HMD Manager Config
			if (ImGui::CollapsingHeader("HMD Manager Config", 0, true, false))
			{
				ImGui::Text("Virtual HMDs:");
				ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
				ImGui::PushItemWidth(100.f);
				if (ImGui::InputInt("##VirtualHMD", &cfg_hmd.virtual_hmd_count))
				{
					cfg_hmd.virtual_hmd_count = static_cast<int>(std::fmax(0, std::fmin(PSMOVESERVICE_MAX_HMD_COUNT, cfg_hmd.virtual_hmd_count)));
				}
				ImGui::PopItemWidth();
				
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip(
						"The number of head mount devices emulated in PSMoveService.\n"
						"Useful if you want to add your custom head mount devices that are not related to PlayStation Move."
					);
			}

			if (ImGui::CollapsingHeader("Miscellaneous", 0, true, false))
			{
				if (ImGui::Button("Open PSMoveService Config Directory"))
				{
					configExec.OpenConfigInExplorer();
				}
			}

			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();

			ImGui::TextDisabled("Note: Restart PSMoveService to apply changes.");

			ImGui::Spacing();

            if (ImGui::Button("Back"))
            {
				m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

			ImGui::SameLine();

			if (ImGui::Button("Save Settings"))
			{
				if (cfg_tracker.isLoaded)
					cfg_tracker.save();

				if (cfg_controller.isLoaded)
					cfg_controller.save();

				if (cfg_hmd.isLoaded)
					cfg_hmd.save();
			}

#ifdef _WIN32
			ImGui::SameLine(0.f, 16.f);

			if (ImGui::Button("Restart PSMoveService"))
			{
				serviceRestart.CheckProcesses();
				serviceRestart.RestartService();
			}
#endif

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

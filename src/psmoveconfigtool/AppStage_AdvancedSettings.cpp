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
	pt.put("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
	pt.put("tracker_sync_mode", tracker_sync_mode);
	pt.put("optical_tracking_timeout", optical_tracking_timeout);
	pt.put("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	pt.put("thread_sleep_ms", thread_sleep_ms);
	pt.put("excluded_opposed_cameras", exclude_opposed_cameras);
	pt.put("min_valid_projection_area", min_valid_projection_area);
	
	pt.put("occluded_area_on_loss_size", occluded_area_on_loss_size);
	pt.put("occluded_area_ignore_num_trackers", occluded_area_ignore_num_trackers);
	pt.put("occluded_area_regain_projection_size", occluded_area_regain_projection_size);
	
	pt.put("projection_collision_avoid", projection_collision_avoid);
	pt.put("projection_collision_offset", projection_collision_offset);
	
	pt.put("average_position_cache_enabled", average_position_cache_enabled);
	pt.put("average_position_cache_cell_size", average_position_cache_cell_size);
	pt.put("average_position_cache_avg_size", average_position_cache_avg_size);
	pt.put("average_position_cache_limit", average_position_cache_limit);
	
	pt.put("min_points_in_contour", min_points_in_contour);
	pt.put("max_tracker_position_deviation", max_tracker_position_deviation);

	pt.put("disable_roi", disable_roi);
	pt.put("autoscale_roi", autoscale_roi);
	pt.put("optimized_roi", optimized_roi);
	pt.put("roi_size", roi_size);
	pt.put("roi_search_size", roi_search_size);
	pt.put("roi_edge_offset", roi_edge_offset);

	pt.put("position_interpolation", position_interpolation);
	pt.put("angular_interpolation", angular_interpolation);
	pt.put("thread_maximum_framrate", thread_maximum_framrate);

	pt.put("global_forward_degrees", global_forward_degrees);

	return pt;
}

void
TrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	virtual_tracker_count = pt.get<int>("virtual_tracker_count", virtual_tracker_count);
	ignore_pose_from_one_tracker = pt.get<bool>("ignore_pose_from_one_tracker", ignore_pose_from_one_tracker);
	tracker_sync_mode = pt.get<int>("tracker_sync_mode", tracker_sync_mode);
	optical_tracking_timeout = pt.get<int>("optical_tracking_timeout", optical_tracking_timeout);
	use_bgr_to_hsv_lookup_table = pt.get<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
	thread_sleep_ms = pt.get<int>("thread_sleep_ms", thread_sleep_ms);
	exclude_opposed_cameras = pt.get<bool>("excluded_opposed_cameras", exclude_opposed_cameras);
	min_valid_projection_area = pt.get<float>("min_valid_projection_area", min_valid_projection_area);
	
	occluded_area_on_loss_size = pt.get<float>("occluded_area_on_loss_size", occluded_area_on_loss_size);
	occluded_area_ignore_num_trackers = pt.get<int>("occluded_area_ignore_num_trackers", occluded_area_ignore_num_trackers);
	occluded_area_regain_projection_size = pt.get<float>("occluded_area_regain_projection_size", occluded_area_regain_projection_size);
	
	projection_collision_avoid = pt.get<bool>("projection_collision_avoid", projection_collision_avoid);
	projection_collision_offset = pt.get<float>("projection_collision_offset", projection_collision_offset);
	
	average_position_cache_enabled = pt.get<bool>("average_position_cache_enabled", average_position_cache_enabled);
	average_position_cache_cell_size = pt.get<float>("average_position_cache_cell_size", average_position_cache_cell_size);
	average_position_cache_avg_size = pt.get<float>("average_position_cache_avg_size", average_position_cache_avg_size);
	average_position_cache_limit = pt.get<float>("average_position_cache_limit", average_position_cache_limit);
	
	min_points_in_contour = pt.get<int>("min_points_in_contour", min_points_in_contour);
	max_tracker_position_deviation = pt.get<float>("max_tracker_position_deviation", max_tracker_position_deviation);

	disable_roi = pt.get<bool>("disable_roi", disable_roi);
	autoscale_roi = pt.get<bool>("autoscale_roi", autoscale_roi);
	optimized_roi = pt.get<bool>("optimized_roi", optimized_roi);
	roi_size = pt.get<int>("roi_size", roi_size);
	roi_search_size = pt.get<int>("roi_search_size", roi_search_size);
	roi_edge_offset = pt.get<int>("roi_edge_offset", roi_edge_offset);

	position_interpolation = pt.get<bool>("position_interpolation", position_interpolation);
	angular_interpolation = pt.get<bool>("angular_interpolation", angular_interpolation);
	thread_maximum_framrate = pt.get<int>("thread_maximum_framrate", thread_maximum_framrate);

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
	pt.put("enable_morpheus", enable_morpheus);

	return pt;
}

void
HMDConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	virtual_hmd_count = pt.get<int>("virtual_hmd_count", virtual_hmd_count);
	enable_morpheus = pt.get<bool>("enable_morpheus", enable_morpheus);
}

const boost::property_tree::ptree
DeviceConfig::config2ptree()
{
	boost::property_tree::ptree pt;

	for (std::pair<std::string, std::string> p : cfg_map)
	{
		pt.put(p.first, p.second);
	}

	pt.put("controller_reconnect_interval", controller_reconnect_interval);
	//pt.put("controller_poll_interval", controller_poll_interval);
	pt.put("tracker_reconnect_interval", tracker_reconnect_interval);
	//pt.put("tracker_poll_interval", tracker_poll_interval);
	pt.put("hmd_reconnect_interval", hmd_reconnect_interval);
	//pt.put("hmd_poll_interval", hmd_poll_interval);
	pt.put("gamepad_api_enabled", gamepad_api_enabled);
	pt.put("gamepad_api_xinput_only", gamepad_api_xinput_only);
	pt.put("platform_api_enabled", platform_api_enabled);

	return pt;
}

void
DeviceConfig::ptree2config(const boost::property_tree::ptree &pt)
{
	map_flatten(pt, "");

	controller_reconnect_interval = pt.get<int>("controller_reconnect_interval", controller_reconnect_interval);
	//controller_poll_interval = pt.get<int>("controller_poll_interval", controller_poll_interval);
	tracker_reconnect_interval = pt.get<int>("tracker_reconnect_interval", tracker_reconnect_interval);
	//tracker_poll_interval = pt.get<int>("tracker_poll_interval", tracker_poll_interval);
	hmd_reconnect_interval = pt.get<int>("hmd_reconnect_interval", hmd_reconnect_interval);
	//hmd_poll_interval = pt.get<int>("hmd_poll_interval", hmd_poll_interval);
	gamepad_api_enabled = pt.get<bool>("gamepad_api_enabled", gamepad_api_enabled);
	gamepad_api_xinput_only = pt.get<bool>("gamepad_api_xinput_only", gamepad_api_xinput_only);
	platform_api_enabled = pt.get<bool>("platform_api_enabled", platform_api_enabled);
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

		cfg_device = DeviceConfig();
		cfg_device.isLoaded = cfg_device.load();

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
			static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(550, fminf(lastWindowVec.y + 32.f, ImGui::GetIO().DisplaySize.y - 32)));
			ImGui::Begin("Advanced Settings", nullptr, window_flags);
			ImGui::BeginGroup();
			{
				// Tracker Manager Config
				if (ImGui::CollapsingHeader("Tracker Manager Config", 0, true, true))
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
								"The number of trackers emulated in PSMoveServiceEx.\n"
								"Useful if you want to add your custom trackers that are not related to PlayStation Move."
							);
					}

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
						ImGui::Text("Tracker synchronization mode:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 200.f);
						int tracker_sync_mode = cfg_tracker.tracker_sync_mode;
						ImGui::PushItemWidth(150.f);
						if (ImGui::Combo("##TrackerSyncMode", &tracker_sync_mode, "Wait All\0Fastest Available\0\0"))
						{
							cfg_tracker.tracker_sync_mode = tracker_sync_mode;
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Wait All:\n"
								"	Will do triangulation whenever all trackers are ready.\n"
								"	This synchronization mode allows the smoothest tracking possible.\n"
								"	Trackers running on different Hz are not supported in this mode\n"
								"	and fast trackers will wait for slow trackers.\n"
								"Fastest Available:\n"
								"	Will do triangulation whenever the fastest two or more trackers are ready.\n"
								"	This synchronization mode allows for fastest tracking possible but\n"
								"	reduces tracking quality and can cause jittering.\n"
								"	Trackers running on different Hz are supported in this mode.\n"
								"(The default value is 'WAIT ALL')"
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
								"The maximum amount of time we can wait for new tracker data for optical tracking.\n"
								"If the time exceeds the given value then the tracker positional tracking will be ignored.\n"
								"(The default value is 100)"
							);
					}

					{
						ImGui::Text("Processing thread sleep (ms):");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##ThreadSleep", &cfg_tracker.thread_sleep_ms))
						{
							cfg_tracker.thread_sleep_ms = static_cast<int>(std::fmax(1, std::fmin(99999, cfg_tracker.thread_sleep_ms)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"(The default value is 1)"
							);
					}

					{
						ImGui::Text("Maximum Processing Thread Framerate:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##MaximumProcessingThreadFramerate", &cfg_tracker.thread_maximum_framrate))
						{
							cfg_tracker.thread_maximum_framrate = static_cast<int>(std::fmax(30, std::fmin(99999, cfg_tracker.thread_maximum_framrate)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"(The default value is 200)"
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
						ImGui::Text("Occluded area size on tracking loss:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##OccludedAreaOnLossSize", &cfg_tracker.occluded_area_on_loss_size, 1.f, 4.f, 2))
						{
							cfg_tracker.occluded_area_on_loss_size = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.occluded_area_on_loss_size)));
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
						ImGui::Text("Occluded area number of ignored trackers:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##OccludedAreaIgnoreTrackers", &cfg_tracker.occluded_area_ignore_num_trackers, 1, 5))
						{
							cfg_tracker.occluded_area_ignore_num_trackers = static_cast<int>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.occluded_area_ignore_num_trackers)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"The number of trackers that will ignore occluded areas (sorted by biggest projection).\n"
								"(The default value is 0)"
							);
					}

					{
						ImGui::Text("Occluded area regain projection size:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##OccludedAreaRegainProjectionSize", &cfg_tracker.occluded_area_regain_projection_size, 1.f, 4.f, 2))
						{
							cfg_tracker.occluded_area_regain_projection_size = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.occluded_area_regain_projection_size)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"The tracker will regain the projection and remove its occlusion\n"
								"if the projection size is bigger than the giving value.\n"
								"(The default value is 32)"
							);
					}

					{
						ImGui::Text("Projection collision detection:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						ImGui::Checkbox("##PorjectionCollisionDetection", &cfg_tracker.projection_collision_avoid);
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Avoid collisions between projections.\n"
								"Enabling this can fix some color collisions such as color bleeding on the bulb edges.\n"
								"(The default value is TRUE)"
							);
					}

					ImGui::Indent();
					{
						ImGui::Text("Projection area offset:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##ProjectionCollisionOffset", &cfg_tracker.projection_collision_offset, 1.f, 4.f, 2))
						{
							cfg_tracker.projection_collision_offset = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.projection_collision_offset)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Adds an offset to the projection collision detection dead-zone.\n"
								"(The default value is 5)"
							);
					}
					ImGui::Unindent();

					{
						ImGui::Text("Cache average position offsets (runtime generated):");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						ImGui::Checkbox("##AveragePositionCache", &cfg_tracker.average_position_cache_enabled);
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Gives each tracker a calculated offset from previous cached average positions.\n"
								"Makes transitions between trackers smoother and reduces jitter.\n"
								"However, the detection of unwanted color noise could result in persistent bad tracking behavior!\n"
								"(The default value is TRUE)"
							);
					}

					ImGui::Indent();
					{
						ImGui::Text("Sample cell size:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##AveragePositionCacheCellSize", &cfg_tracker.average_position_cache_cell_size, 1.f, 4.f, 2))
						{
							cfg_tracker.average_position_cache_cell_size = static_cast<float>(std::fmax(1.f, std::fmin(99999.f, cfg_tracker.average_position_cache_cell_size)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"If there are no nearby samples by this distance, new ones will be created.\n"
								"The lower the value the more smoother and less jittery transitions between trackers will become.\n"
								"Too low of a value can result in increased CPU usage.\n"
								"(The default value is 5 (cm))"
							);
					}
					ImGui::Unindent();

					ImGui::Indent();
					{
						ImGui::Text("Sampling distance:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##AveragePositionCacheAvgSize", &cfg_tracker.average_position_cache_avg_size, 1.f, 4.f, 2))
						{
							cfg_tracker.average_position_cache_avg_size = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.average_position_cache_avg_size)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Overall distance to gather nearby samples to apply cached position offsets to the trackers.\n"
								"Multiple samples by this range will be averaged making transitions between trackers smoother.\n"
								"If the value is smaller than 'Sample cell size' then only the nearest sample will be used.\n"
								"(The default value is 30 (cm))"
							);
					}
					ImGui::Unindent();

					ImGui::Indent();
					{
						ImGui::Text("Maximum samples:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputFloat("##AveragePositionCacheLimit", &cfg_tracker.average_position_cache_limit, 5.f, 10.f, 2))
						{
							cfg_tracker.average_position_cache_limit = static_cast<float>(std::fmax(0.f, std::fmin(99999.f, cfg_tracker.average_position_cache_limit)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Maximum count of samples per tracker pair.\n"
								"If the number of samples exceeds the maximum count then old samples will be removed.\n"
								"Too high of a value can result in increased CPU usage.\n"
								"(The default value is 1000)"
							);
					}
					ImGui::Unindent();

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
								"(The default value is 15 (cm))"
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
						{
							ImGui::Text("Scale ROI (region of interest) by tracker resolution:");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::Checkbox("##ROIAutoscale", &cfg_tracker.autoscale_roi);

							if (ImGui::IsItemHovered())
								ImGui::SetTooltip(
									"Scales the ROI (region of interest) uniformly between trackers running with different resolutions.\n"
									"(The default value is TRUE)"
								);
						}

						{
							ImGui::Text("ROI (region of interest) size:");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(100.f);
							if (ImGui::InputInt("##ROISize", &cfg_tracker.roi_size, 1, 4))
							{
								cfg_tracker.roi_size = static_cast<int>(std::fmax(4, std::fmin(99999, cfg_tracker.roi_size)));
							}
							ImGui::PopItemWidth();

							if (ImGui::IsItemHovered())
								ImGui::SetTooltip(
									"Size of the ROI (region of interest).\n"
									"Small sizes can reduce CPU usage but can result in tracking loss on quick movement.\n"
									"(The default value is 32)"
								);
						}

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

						ImGui::Indent();
						{
							ImGui::Text("ROI (region of interest) search size:");
							ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
							ImGui::PushItemWidth(100.f);
							if (ImGui::InputInt("##ROISearchSize", &cfg_tracker.roi_search_size, 1, 4))
							{
								cfg_tracker.roi_search_size = static_cast<int>(std::fmax(4, std::fmin(99999, cfg_tracker.roi_search_size)));
							}
							ImGui::PopItemWidth();

							if (ImGui::IsItemHovered())
								ImGui::SetTooltip(
									"Size of the ROI (region of interest) that searches for a new projection if it has been previously lost.\n"
									"Small sizes can reduce CPU usage but can increase search time.\n"
									"(The default value is 164)"
								);
						}
						ImGui::Unindent();
					}
					ImGui::Unindent();

					{
						ImGui::Text("ROI (region of interest) tracker edge offset:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##ROIEdgeOffset", &cfg_tracker.roi_edge_offset, 1, 4))
						{
							cfg_tracker.roi_edge_offset = static_cast<int>(std::fmax(0, std::fmin(64, cfg_tracker.roi_edge_offset)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Adds an offset to the ROI (region of interest) on tracker edges.\n"
								"Adding an slight offset can help avoid tracking jitter nearby tracker edges.\n"
								"(The default value is 4)"
							);
					}

					{
						ImGui::Text("Position Interpolation:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::Checkbox("##PositionInterpolation", &cfg_tracker.position_interpolation);

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Interpolation smoothes motion between previous and current position.\n"
								"Interpolation adds a frame of delay because it has to wait for new data to become available.\n"
								"If you want more responsive motion, disable this feature.\n"
								"(The default value is TRUE)"
							);
					}

					{
						ImGui::Text("Angular Interpolation:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::Checkbox("##AngularInterpolation", &cfg_tracker.angular_interpolation);

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Interpolation smoothes motion between previous and current orientation.\n"
								"Interpolation adds a frame of delay because it has to wait for new data to become available.\n"
								"If you want more responsive motion, disable this feature.\n"
								"(The default value is TRUE)"
							);
					}

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
				if (ImGui::CollapsingHeader("Controller Manager Config", 0, true, true))
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
							"The number of controllers emulated in PSMoveServiceEx.\n"
							"Useful if you want to add your custom controllers that are not related to PlayStation Move."
						); 
				}

				// HMD Manager Config
				if (ImGui::CollapsingHeader("HMD Manager Config", 0, true, true))
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
							"The number of head mount devices emulated in PSMoveServiceEx.\n"
							"Useful if you want to add your custom head mount devices that are not related to PlayStation Move."
						);

					ImGui::Text("Enable Morpheus HMD:");
					ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
					ImGui::PushItemWidth(100.f);
					ImGui::Checkbox("##EnableMorpheusHMD", &cfg_hmd.enable_morpheus);
					ImGui::PopItemWidth();

					if (ImGui::IsItemHovered())
						ImGui::SetTooltip(
							"Enables the Morpheus Head-Mounted Display in PSMoveServiceEx.\n"
							"If you use other programs that use the Head-Mounted Display disable this setting to avoid conflicts."
						);
				}

				// Device Manager Config
				if (ImGui::CollapsingHeader("Device Manager Config", 0, true, false))
				{
					{
						ImGui::Text("Controller reconnect interval (ms):");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##ControllerReconnectInterval", &cfg_device.controller_reconnect_interval))
						{
							cfg_device.controller_reconnect_interval = static_cast<int>(std::fmax(0, std::fmin(999999, cfg_device.controller_reconnect_interval)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"(The default value is 1000)"
							);
					}

					{
						ImGui::Text("Tracker reconnect interval (ms):");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##TrackerReconnectInterval", &cfg_device.tracker_reconnect_interval))
						{
							cfg_device.tracker_reconnect_interval = static_cast<int>(std::fmax(0, std::fmin(999999, cfg_device.tracker_reconnect_interval)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"(The default value is 10000)"
							);
					}

					{
						ImGui::Text("HMD reconnect interval (ms):");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::PushItemWidth(100.f);
						if (ImGui::InputInt("##HMDReconnectInterval", &cfg_device.hmd_reconnect_interval))
						{
							cfg_device.hmd_reconnect_interval = static_cast<int>(std::fmax(0, std::fmin(999999, cfg_device.hmd_reconnect_interval)));
						}
						ImGui::PopItemWidth();

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"(The default value is 10000)"
							);
					}

					{
						ImGui::Text("Enable gamepad API:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::Checkbox("##GamepadApiEnabled", &cfg_device.gamepad_api_enabled);

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Enable to use gamepad API (e.g. PSMove or gamepad controller buttons) in PSMoveServiceEx (recommended).\n"
								"(The default value is TRUE)"
							);
					}

					{
						ImGui::Indent();
						ImGui::Text("XInput gamepads only:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::Checkbox("##GamepadApiXInputOnly", &cfg_device.gamepad_api_xinput_only);

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Enable to use XInput API only. (e.g. Xbox controllers)\n"
								"(The default value is TRUE)"
							);

						ImGui::Unindent();
					}

					{
						ImGui::Text("Enable platform API:");
						ImGui::SameLine(ImGui::GetWindowWidth() - 150.f);
						ImGui::Checkbox("##PlatformApiEnabled", &cfg_device.platform_api_enabled);

						if (ImGui::IsItemHovered())
							ImGui::SetTooltip(
								"Enable to use platform API (e.g. hotplug detection) in PSMoveServiceEx (recommended).\n"
								"(The default value is TRUE)"
							);
					}
				}

				if (ImGui::CollapsingHeader("Miscellaneous", 0, true, false))
				{
					if (ImGui::Button("Open PSMoveServiceEx Config Directory"))
					{
						configExec.OpenConfigInExplorer();
					}
				}

				ImGui::Spacing();
				ImGui::Separator();
				ImGui::Spacing();

				ImGui::TextDisabled("Note: Restart PSMoveServiceEx to apply changes.");

				ImGui::Spacing();

				if (ImGui::Button("Return to Main Menu"))
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

					if (cfg_device.isLoaded)
						cfg_device.save();
				}

#ifdef _WIN32
				ImGui::SameLine(0.f, 16.f);

				if (ImGui::Button("Restart PSMoveServiceEx"))
				{
					serviceRestart.CheckProcesses();
					serviceRestart.RestartService();
				}
#endif
			}
			ImGui::EndGroup();
			if (ImGui::IsItemVisible())
				lastWindowVec = ImGui::GetItemRectSize();

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

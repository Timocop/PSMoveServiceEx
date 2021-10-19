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
    // Always fallback to the main menu on disconnection
    //m_app->registerEventFallbackAppStage<AppStage_AdvancedSettings>(PSMEventMessage::PSMEvent_controllerListUpdated);

	printf("AppStage_AdvancedSettings::init\n");

    return true;
}

void AppStage_AdvancedSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

    // Only set the menu state if it hasn't been set already
    if (m_menuState == AppStage_AdvancedSettings::inactive)
    {
		m_menuState = AppStage_AdvancedSettings::idle;
    }

	printf("AppStage_AdvancedSettings::enter\n");

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
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoCollapse;

    switch(m_menuState)
    {
    case idle:
        {
			ImGui::SetNextWindowPosCenter();
			ImGui::SetNextWindowSize(ImVec2(350, 490));
			ImGui::Begin("Advanced Settings", nullptr, window_flags | ImGuiWindowFlags_MenuBar);

			if (ImGui::Button("Open Config Directory"))
			{
				configExec.OpenConfigInExplorer();
			}

			ImGui::Separator();

			if (ImGui::Button("Edit Controller Config"))
			{
				configExec.OpenConfigInEditor("ControllerManagerConfig.json");
			}

			if (ImGui::Button("Edit Tracker Config"))
			{
				configExec.OpenConfigInEditor("TrackerManagerConfig.json");
			}

			if (ImGui::Button("Edit HMD Config"))
			{
				configExec.OpenConfigInEditor("HMDManagerConfig.json");
			}

			ImGui::Separator();

			if (ImGui::Button("Edit Device Config"))
			{
				configExec.OpenConfigInEditor("DeviceManagerConfig.json");
			}

			if (ImGui::Button("Edit USB Config"))
			{
				configExec.OpenConfigInEditor("USBManagerConfig.json");
			}

			if (ImGui::Button("Edit Network Config"))
			{
				configExec.OpenConfigInEditor("NetworkManagerConfig.json");
			}

			ImGui::Spacing();
			ImGui::TextDisabled("Note: Restart PSMoveService to apply changes.");
			ImGui::Spacing();

            if (ImGui::Button("Back"))
            {
				m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }
    
            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

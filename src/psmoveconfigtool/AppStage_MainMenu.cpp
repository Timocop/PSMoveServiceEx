//-- inludes -----
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_ControllerSettings.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_ServiceSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_AdvancedSettings.h"
#include "App.h"
#include "Camera.h"
#include "ProtocolVersion.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_MainMenu::APP_STAGE_NAME= "MainMenu";

//-- public methods -----
AppStage_MainMenu::AppStage_MainMenu(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_MainMenu::inactive)
{ }

bool AppStage_MainMenu::init(int argc, char** argv)
{
    // Always fallback to the main menu on disconnection
    m_app->registerEventFallbackAppStage<AppStage_MainMenu>(PSMEventMessage::PSMEvent_disconnectedFromService);

    return true;
}

void AppStage_MainMenu::enter()
{
    m_app->setCameraType(_cameraFixed);
	m_app->getFixedCamera()->resetOrientation();

    // Only set the menu state if it hasn't been set already
    if (m_menuState == AppStage_MainMenu::inactive)
    {
        if (PSM_GetIsInitialized())
        {
            m_menuState= AppStage_MainMenu::connectedToService;
        }
        else
        {
            m_menuState= AppStage_MainMenu::startConnectionToService;
        }
    }
}

void AppStage_MainMenu::exit()
{
    // Upon normal exit, set the state to inactive
    m_menuState= AppStage_MainMenu::inactive;
}

void AppStage_MainMenu::renderUI()
{
    switch(m_menuState)
    {
    case connectedToService:
	{
		ImGuiWindowFlags window_flags =
			ImGuiWindowFlags_ShowBorders |
			ImGuiWindowFlags_NoResize |
			ImGuiWindowFlags_NoMove |
			ImGuiWindowFlags_NoScrollbar |
			ImGuiWindowFlags_NoCollapse;
		ImGui::SetNextWindowPosCenter();

		char szWindowTitle[255];
		snprintf(szWindowTitle, sizeof(szWindowTitle), "PSMove Config Tool v%s", PSM_RELEASE_VERSION_STRING);

		static ImVec2 lastWindowVec = ImVec2(0.f, 4.f);

		ImGui::SetNextWindowSize(ImVec2(300, fminf(lastWindowVec.y + 36.f, ImGui::GetIO().DisplaySize.y - 36.f)));
		ImGui::Begin(szWindowTitle, nullptr, window_flags);
		ImGui::BeginGroup();
		{
			if (ImGui::Button("Controller Settings"))
			{
				m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
			}

			if (ImGui::Button("Head-mounted Display Settings"))
			{
				m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
			}

			if (ImGui::Button("Tracker Settings"))
			{
				m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}

	#ifdef _WIN32
			ImGui::Separator();

			if (ImGui::Button("Advanced Settings"))
			{
				m_app->setAppStage(AppStage_AdvancedSettings::APP_STAGE_NAME);
			}

			ImGui::Separator();
	#endif

			if (ImGui::Button("Exit"))
			{
				m_app->requestShutdown();
			}
		}
		ImGui::EndGroup();
		if (ImGui::IsItemVisible())
			lastWindowVec = ImGui::GetItemRectSize();

        ImGui::End();
    } break;
    case pendingConnectToToService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Status", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);
            ImGui::Text("Connecting to PSMoveServiceEx...");
            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }
            ImGui::End();
        } break;
	case startConnectionToService:
    case failedConnectionToService:
	case disconnectedFromService:
        {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Connect", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

			if (m_menuState == failedConnectionToService)
			{
	            ImGui::Text("Failed to connect to PSMoveServiceEx!");
			}
			else if (m_menuState == disconnectedFromService)
			{
				ImGui::Text("Disconnected from PSMoveServiceEx!");
			}
            
			ImGui::PushItemWidth(125.f);
			if (ImGui::InputText("Server Address", m_app->m_serverAddress, sizeof(m_app->m_serverAddress), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank))
			{
				m_app->m_bIsServerLocal= (strncmp(m_app->m_serverAddress, PSMOVESERVICE_DEFAULT_ADDRESS, sizeof(m_app->m_serverAddress)) == 0);
			}

			ImGui::InputText("Server Port", m_app->m_serverPort, sizeof(m_app->m_serverPort), ImGuiInputTextFlags_CharsDecimal);
			ImGui::PopItemWidth();

            if (ImGui::Button("Connect"))
            {
                m_menuState= AppStage_MainMenu::pendingConnectToToService;
                m_app->reconnectToService();
            }

            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }

            ImGui::End();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

bool AppStage_MainMenu::onClientAPIEvent(
    PSMEventMessage::eEventType event, 
    PSMEventDataHandle opaque_event_handle)
{
    bool bHandeled= false;

    switch(event)
    {
    case PSMEventMessage::PSMEvent_connectedToService:
        {
            // Allow the user to access the menu now
            m_menuState= AppStage_MainMenu::connectedToService;
            bHandeled= true;
        } break;

    case PSMEventMessage::PSMEvent_failedToConnectToService:
        {
            // Tell the user that the connection attempt failed
            m_menuState= AppStage_MainMenu::failedConnectionToService;
            bHandeled= true;
        } break;

    case PSMEventMessage::PSMEvent_disconnectedFromService:
        {
            // Tell the user that we failed to connect
            m_menuState= AppStage_MainMenu::disconnectedFromService;

            // If we weren't running this stage, make sure we are now
            if (m_app->getCurrentAppStage() != this)
            {
                m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            bHandeled= true;
        } break;
    }

    return bHandeled; 
}
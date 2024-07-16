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
#include "AssetManager.h"

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
	static float waitCount;

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

		char szWindowTitle[255];
		snprintf(szWindowTitle, sizeof(szWindowTitle), "PSMove Config Tool v%s", PSM_RELEASE_VERSION_STRING);

		ImGui::SetNextWindowPosCenter();
		ImGui::Begin(szWindowTitle, nullptr, window_flags);
		ImGui::BeginGroup();
		{
			const ImVec2 _lastLoc = ImGui::GetCursorPos();
			if (ImGui::Button("      Controller Settings"))
			{
				m_app->setAppStage(AppStage_ControllerSettings::APP_STAGE_NAME);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconController()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			if (ImGui::Button("      Head-mounted Display Settings"))
			{
				m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconHmd()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			if (ImGui::Button("      Tracker Settings"))
			{
				m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconTracker()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

	#ifdef _WIN32
			ImGui::Separator();

			if (ImGui::Button("      Advanced Settings"))
			{
				m_app->setAppStage(AppStage_AdvancedSettings::APP_STAGE_NAME);
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconSettings()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));
	#endif

			ImGui::Separator();
			if (ImGui::Button("      Exit"))
			{
				m_app->requestShutdown();
			}
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

		}
		ImGui::EndGroup();
		ImGui::SetWindowSize(ImVec2(300, 0));
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
            ImGui::Begin("Status", nullptr, window_flags);

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
            ImGui::Text("Connecting to PSMoveServiceEx...");
			ImGui::Separator();
            if (ImGui::Button("      Exit"))
            {
                m_app->requestShutdown();
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));


			ImGui::SetWindowSize(ImVec2(300, 0));
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
            ImGui::Begin("Connect", nullptr, window_flags);

			if (m_menuState == failedConnectionToService)
			{
				ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
				ImGui::Text("Failed to connect to PSMoveServiceEx!");
				ImGui::Separator();
			}
			else if (m_menuState == disconnectedFromService)
			{
				ImGui::Image(AssetManager::getInstance()->getIconWarning()->getImTextureId(), ImVec2(32, 32));
				ImGui::SameLine();
				ImGui::Text("Disconnected from PSMoveServiceEx!");
				ImGui::Separator();
			}

			ImGui::PushItemWidth(125.f);
			if (ImGui::InputText("Server Address", m_app->m_serverAddress, sizeof(m_app->m_serverAddress), ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank))
			{
				m_app->m_bIsServerLocal= (strncmp(m_app->m_serverAddress, PSMOVESERVICE_DEFAULT_ADDRESS, sizeof(m_app->m_serverAddress)) == 0);
			}

			ImGui::InputText("Server Port", m_app->m_serverPort, sizeof(m_app->m_serverPort), ImGuiInputTextFlags_CharsDecimal);
			ImGui::PopItemWidth();

			ImGui::Separator();
            if (ImGui::Button("      Connect"))
            {
                m_menuState= AppStage_MainMenu::pendingConnectToToService;
                m_app->reconnectToService();
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconConnect()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));

			ImGui::SameLine();
            if (ImGui::Button("      Exit"))
            {
                m_app->requestShutdown();
            }
			ImGui::GetWindowDrawList()->AddImage(AssetManager::getInstance()->getIconBan()->getImTextureId(),
				ImVec2(ImGui::GetItemRectMin().x + 2, ImGui::GetItemRectMin().y + 2),
				ImVec2(ImGui::GetItemRectMin().x + ImGui::GetItemRectSize().y - 2, ImGui::GetItemRectMin().y + ImGui::GetItemRectSize().y - 2));


			ImGui::SetWindowSize(ImVec2(300, 0));
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
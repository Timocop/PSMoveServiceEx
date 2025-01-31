//-- inludes -----
#include "AppStage_ServiceSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_ServiceSettings::APP_STAGE_NAME= "ServiceSettings";

//-- constants -----

//-- public methods -----
AppStage_ServiceSettings::AppStage_ServiceSettings(App *app) 
    : AppStage(app)
{ }

void AppStage_ServiceSettings::enter()
{
    m_app->setCameraType(_cameraFixed);
	m_app->getFixedCamera()->resetOrientation();
}

void AppStage_ServiceSettings::exit()
{
}

void AppStage_ServiceSettings::update()
{
}

void AppStage_ServiceSettings::renderUI()
{
    ImGuiWindowFlags window_flags = 
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize | 
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    ImGui::SetNextWindowPosCenter();
    ImGui::Begin("Service Settings", nullptr, window_flags);

	ImGui::Separator();
    if (ImGui::Button("Return to Main Menu"))
    {
        m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
    }

	ImGui::SetWindowSize(ImVec2(300, 0));
    ImGui::End();
}
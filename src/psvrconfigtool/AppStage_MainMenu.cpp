//-- inludes -----
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_HMDSettings.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

//-- statics ----
const char *AppStage_MainMenu::APP_STAGE_NAME= "MainMenu";

//-- public methods -----
AppStage_MainMenu::AppStage_MainMenu(App *app) 
    : AppStage(app)
{ }

bool AppStage_MainMenu::init(int argc, char** argv)
{
    return true;
}

void AppStage_MainMenu::enter()
{
    m_app->setCameraType(_cameraFixed);
}

void AppStage_MainMenu::renderUI()
{
    if (PSVR_GetIsInitialized())
    {
        ImGuiWindowFlags window_flags = 
            ImGuiWindowFlags_ShowBorders |
            ImGuiWindowFlags_NoResize | 
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoCollapse;
        ImGui::SetNextWindowPosCenter();
        ImGui::Begin("Main Menu", nullptr, ImVec2(300, 400), k_background_alpha, window_flags);
         
        if (ImGui::Button("HMD Settings"))
        {
            m_app->setAppStage(AppStage_HMDSettings::APP_STAGE_NAME);
        }    
    
        if (ImGui::Button("Tracker Settings"))
        {
            m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }
    
        if (ImGui::Button("Exit"))
        {
            m_app->requestShutdown();
        }
    
        ImGui::End();
    }
    else
    {
            ImGuiWindowFlags window_flags = 
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize | 
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse;
            ImGui::SetNextWindowPosCenter();
            ImGui::Begin("Error", nullptr, ImVec2(300, 150), k_background_alpha, window_flags);

	        ImGui::Text("Failed to initialize PSVRSERVICE!");
            
            if (ImGui::Button("Exit"))
            {
                m_app->requestShutdown();
            }

            ImGui::End();
    }
}
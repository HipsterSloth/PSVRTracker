//-- inludes -----
#include "AppStage_HMDSettings.h"
#include "AppStage_HMDAccelerometerTest.h"
#include "AppStage_HMDGyroscopeTest.h"
#include "AppStage_HMDTrackingTest.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_HMDSettings::APP_STAGE_NAME= "HMDSettings";

//-- constants -----
const int k_default_hmd_position_filter_index = 3; // LowPassExponential
const int k_default_virtual_orientation_filter_index = 0; // PassThru
const int k_default_morpheus_position_filter_index = 4; // ComplimentaryOpticalIMU
const int k_default_morpheus_orientation_filter_index = 2; // ComplementaryOpticalARG

const char* k_hmd_position_filter_names[] = { "PassThru", "LowPassOptical", "LowPassIMU", "LowPassExponential", "ComplimentaryOpticalIMU", "KalmanPose" };
const char* k_morpheus_orientation_filter_names[] = { "PassThru", "MadgwickARG", "ComplementaryOpticalARG", "KalmanPose" };
const char* k_virtual_orientation_filter_names[] = { "PassThru", "KalmanPose" };

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

            switch (hmdInfo.hmd_info.tracking_color_type)
            {
            case PSVRTrackingColorType_Magenta:
                bulb_color = glm::vec3(1.f, 0.f, 1.f);
                break;
            case PSVRTrackingColorType_Cyan:
                bulb_color = glm::vec3(0.f, 1.f, 1.f);
                break;
            case PSVRTrackingColorType_Yellow:
                bulb_color = glm::vec3(1.f, 1.f, 0.f);
                break;
            case PSVRTrackingColorType_Red:
                bulb_color = glm::vec3(1.f, 0.f, 0.f);
                break;
            case PSVRTrackingColorType_Green:
                bulb_color = glm::vec3(0.f, 1.f, 0.f);
                break;
            case PSVRTrackingColorType_Blue:
                bulb_color = glm::vec3(0.f, 0.f, 1.f);
                break;
            default:
                break;
            }

            switch (hmdInfo.hmd_info.hmd_type)
            {
            case PSVRHmd_Morpheus:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(2.f, 2.f, 2.f));
                    drawMorpheusModel(scale3, glm::vec3(1.f, 1.f, 1.f));
                } break;
            case PSVRHmd_Virtual:
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
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(350, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_hmdInfos.size() > 0)
        {
            HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

            if (m_selectedHmdIndex > 0)
            {
                if (ImGui::Button("<##HMDIndex"))
                {
                    --m_selectedHmdIndex;
                }
                ImGui::SameLine();
            }
            ImGui::Text("HMD: %d", m_selectedHmdIndex);
            if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
            {
                ImGui::SameLine();
                if (ImGui::Button(">##HMDIndex"))
                {
                    ++m_selectedHmdIndex;
                }
            }
            
            // Combo box selection for hmd tracking color
            if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Virtual)
            {
                int newTrackingColorType = hmdInfo.hmd_info.tracking_color_type;

                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Tracking Color", &newTrackingColorType, "Magenta\0Cyan\0Yellow\0Red\0Green\0Blue\0\0"))
                {
                    hmdInfo.hmd_info.tracking_color_type = static_cast<PSVRTrackingColorType>(newTrackingColorType);

                    PSVR_SetHmdTrackingColorID(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.tracking_color_type);

                    // Re-request the controller list since the tracking colors could changed for other controllers
                    request_hmd_list();
                }
                ImGui::PopItemWidth();
            }
            else if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus)
            {
                switch (hmdInfo.hmd_info.tracking_color_type)
                {
                case PSVRTrackingColorType_Magenta:
                    ImGui::BulletText("Tracking Color: Magenta");
                    break;
                case PSVRTrackingColorType_Cyan:
                    ImGui::BulletText("Tracking Color: Cyan");
                    break;
                case PSVRTrackingColorType_Yellow:
                    ImGui::BulletText("Tracking Color: Yellow");
                    break;
                case PSVRTrackingColorType_Red:
                    ImGui::BulletText("Tracking Color: Red");
                    break;
                case PSVRTrackingColorType_Green:
                    ImGui::BulletText("Tracking Color: Green");
                    break;
                case PSVRTrackingColorType_Blue:
                    ImGui::BulletText("Tracking Color: Blue");
                    break;
                }
            }

            ImGui::BulletText("HMD ID: %d", hmdInfo.hmd_info.hmd_id);

            switch (hmdInfo.hmd_info.hmd_type)
            {
            case PSVRHmd_Morpheus:
                {
                    ImGui::BulletText("HMD Type: Morpheus");
                    ImGui::TextWrapped("Device Path: %s", hmdInfo.hmd_info.device_path);
                } break;
            case PSVRHmd_Virtual:
                {
                    ImGui::BulletText("HMD Type: VirtualHMD");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            if (m_selectedHmdIndex > 0)
            {
                if (ImGui::Button("Previous HMD"))
                {
                    --m_selectedHmdIndex;
                }
            }

            if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
            {
                if (ImGui::Button("Next HMD"))
                {
                    ++m_selectedHmdIndex;
                }
            }

            if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus)
            {
                if (ImGui::Button("Test Accelerometer"))
                {
                    m_app->setAppStage(AppStage_HMDAccelerometerTest::APP_STAGE_NAME);
                }
            }

            if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus)
            {
                if (ImGui::Button("Test Orientation"))
                {
                    m_app->setAppStage(AppStage_HMDGyroscopeTest::APP_STAGE_NAME);
                }
            }

            if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus || 
                hmdInfo.hmd_info.hmd_type == PSVRHmd_Virtual)
            {
                if (ImGui::Button("Test LED Model"))
                {
                    AppStage_HMDTrackingTest::enterStageAndTestTracking(m_app, m_selectedHmdIndex);
                }
            }

            if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus)
            {		
                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
                {
                    strncpy(hmdInfo.hmd_info.position_filter, k_hmd_position_filter_names[hmdInfo.PositionFilterIndex], sizeof(hmdInfo.hmd_info.position_filter));
                    PSVR_SetHmdPositionFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.position_filter);
                }
                if (ImGui::Combo("Orientation Filter", &hmdInfo.OrientationFilterIndex, k_morpheus_orientation_filter_names, UI_ARRAYSIZE(k_morpheus_orientation_filter_names)))
                {
                    strncpy(hmdInfo.hmd_info.orientation_filter, k_morpheus_orientation_filter_names[hmdInfo.OrientationFilterIndex], sizeof(hmdInfo.hmd_info.orientation_filter));
                    PSVR_SetHmdOrientationFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.orientation_filter);
                }
                if (ImGui::SliderFloat("Prediction Time", &hmdInfo.hmd_info.prediction_time, 0.f, k_max_hmd_prediction_time))
                {
                    PSVR_SetHmdPredictionTime(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.prediction_time);
                }
                if (ImGui::Button("Reset Filter Defaults"))
                {
                    hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
                    hmdInfo.OrientationFilterIndex = k_default_morpheus_position_filter_index;
                    strncpy(hmdInfo.hmd_info.position_filter, k_hmd_position_filter_names[k_default_hmd_position_filter_index], sizeof(hmdInfo.hmd_info.position_filter));
                    strncpy(hmdInfo.hmd_info.orientation_filter, k_morpheus_orientation_filter_names[k_default_morpheus_position_filter_index], sizeof(hmdInfo.hmd_info.orientation_filter));
                    PSVR_SetHmdPositionFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.position_filter);
                    PSVR_SetHmdOrientationFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.orientation_filter);
                }
                ImGui::PopItemWidth();
            }				
            else if (hmdInfo.hmd_info.hmd_type == PSVRHmd_Virtual)
            {
                ImGui::PushItemWidth(195);
                if (ImGui::Combo("Position Filter", &hmdInfo.PositionFilterIndex, k_hmd_position_filter_names, UI_ARRAYSIZE(k_hmd_position_filter_names)))
                {
                    strncpy(hmdInfo.hmd_info.position_filter, k_hmd_position_filter_names[hmdInfo.PositionFilterIndex], sizeof(hmdInfo.hmd_info.position_filter));
                    PSVR_SetHmdPositionFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.position_filter);
                }
                if (ImGui::Combo("Orientation Filter", &hmdInfo.OrientationFilterIndex, k_virtual_orientation_filter_names, UI_ARRAYSIZE(k_virtual_orientation_filter_names)))
                {
                    strncpy(hmdInfo.hmd_info.orientation_filter, k_virtual_orientation_filter_names[hmdInfo.OrientationFilterIndex], sizeof(hmdInfo.hmd_info.orientation_filter));
                    PSVR_SetHmdOrientationFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.orientation_filter);
                }
                if (ImGui::SliderFloat("Prediction Time", &hmdInfo.hmd_info.prediction_time, 0.f, k_max_hmd_prediction_time))
                {
                    PSVR_SetHmdPredictionTime(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.prediction_time);
                }
                if (ImGui::Button("Reset Filter Defaults"))
                {
                    hmdInfo.PositionFilterIndex = k_default_hmd_position_filter_index;
                    hmdInfo.OrientationFilterIndex = k_default_virtual_orientation_filter_index;
                    strncpy(hmdInfo.hmd_info.position_filter, k_hmd_position_filter_names[k_default_hmd_position_filter_index], sizeof(hmdInfo.hmd_info.position_filter));
                    strncpy(hmdInfo.hmd_info.orientation_filter, k_virtual_orientation_filter_names[k_default_virtual_orientation_filter_index], sizeof(hmdInfo.hmd_info.orientation_filter));
                    PSVR_SetHmdPositionFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.position_filter);
                    PSVR_SetHmdOrientationFilter(hmdInfo.hmd_info.hmd_id, hmdInfo.hmd_info.orientation_filter);
                }
                ImGui::PopItemWidth();
            }
        }
        else
        {
            ImGui::Text("No HMDs");
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

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
    PSVREventType event_type)
{
    bool bHandled = false;

    switch (event_type)
    {
    case PSVREvent_hmdListUpdated:
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

        PSVRHmdList hmd_list;
        if (PSVR_GetHmdList(&hmd_list) == PSVRResult_Success)
        {
            handle_hmd_list_response(hmd_list);
        }
        else
        {
            m_menuState = AppStage_HMDSettings::failedHmdListRequest;
        }
    }
}

void AppStage_HMDSettings::handle_hmd_list_response(
    const PSVRHmdList &hmd_list)
{
    for (int hmd_index = 0; hmd_index < hmd_list.count; ++hmd_index)
    {
        const PSVRClientHMDInfo &HmdResponse = hmd_list.hmds[hmd_index];
        AppStage_HMDSettings::HMDInfo HmdInfo;

        HmdInfo.hmd_info= HmdResponse;

        if (HmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus)
        {
            HmdInfo.OrientationFilterIndex =
                find_string_entry(
                    HmdResponse.orientation_filter,
                    k_morpheus_orientation_filter_names,
                    UI_ARRAYSIZE(k_morpheus_orientation_filter_names));
            if (HmdInfo.OrientationFilterIndex == -1)
            {
                HmdInfo.OrientationFilterIndex = 0;
            }
        }
        else if (HmdInfo.hmd_info.hmd_type == PSVRHmd_Virtual)
        {
            HmdInfo.OrientationFilterIndex =
                find_string_entry(
                    HmdResponse.orientation_filter,
                    k_virtual_orientation_filter_names,
                    UI_ARRAYSIZE(k_virtual_orientation_filter_names));
            if (HmdInfo.OrientationFilterIndex == -1)
            {
                HmdInfo.OrientationFilterIndex = 0;
            }
        }
        else
        {
            HmdInfo.OrientationFilterIndex = -1;
        }

        if (HmdInfo.hmd_info.hmd_type == PSVRHmd_Morpheus ||
            HmdInfo.hmd_info.hmd_type == PSVRHmd_Virtual)
        {
            HmdInfo.PositionFilterIndex =
                find_string_entry(
                    HmdInfo.hmd_info.position_filter,
                    k_hmd_position_filter_names,
                    UI_ARRAYSIZE(k_hmd_position_filter_names));
            if (HmdInfo.PositionFilterIndex == -1)
            {
                HmdInfo.PositionFilterIndex = 0;
            }
        }
        else
        {
            HmdInfo.PositionFilterIndex = -1;
        }

        m_hmdInfos.push_back(HmdInfo);
    }

    m_selectedHmdIndex = (m_hmdInfos.size() > 0) ? 0 : -1;
    m_menuState = AppStage_HMDSettings::idle;
}
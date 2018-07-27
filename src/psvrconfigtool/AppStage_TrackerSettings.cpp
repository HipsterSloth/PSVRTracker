//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TrackerTest.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_MonoCalibration.h"
#include "AppStage_StereoCalibration.h"
#include "AppStage_HMDTrackingTest.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "PSVRClient_CAPI.h"
#include "Renderer.h"
#include "UIConstants.h"

#include <glm/gtc/matrix_transform.hpp>
#include <imgui.h>

//-- statics ----
const char *AppStage_TrackerSettings::APP_STAGE_NAME= "CameraSettings";

//-- constants -----

//-- public methods -----
AppStage_TrackerSettings::AppStage_TrackerSettings(App *app) 
    : AppStage(app)
    , m_menuState(AppStage_TrackerSettings::inactive)
    , m_selectedTrackerIndex(-1)
    , m_selectedHmdIndex(-1)
{ }

void AppStage_TrackerSettings::enter()
{
    m_app->setCameraType(_cameraFixed);

    request_tracker_list();
}

void AppStage_TrackerSettings::exit()
{
}

void AppStage_TrackerSettings::update()
{
}
    
void AppStage_TrackerSettings::render()
{
    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        if (m_selectedTrackerIndex >= 0)
        {
            const PSVRClientTrackerInfo &trackerInfo = m_trackerInfos.trackers[m_selectedTrackerIndex];

            switch (trackerInfo.tracker_type)
            {
            case PSVRTracker_PS3Eye:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS3EyeModel(scale3);
                } break;
            case PSVRTracker_PS4Camera:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS4CameraModel(scale3);
                } break;
            case PSVRTracker_GenericMonoCamera:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS3EyeModel(scale3);
                } break;
            case PSVRTracker_GenericStereoCamera:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawGenericStereoCameraModel(scale3);
                } break;

            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::pendingHmdListRequest:
    case eTrackerMenuState::failedHmdListRequest:
    {
    } break;

    default:
        assert(0 && "unreachable");
    }
}

const PSVRClientTrackerInfo *AppStage_TrackerSettings::getSelectedTrackerInfo() const
{
    return
        (m_selectedTrackerIndex != -1)
        ? &m_trackerInfos.trackers[m_selectedTrackerIndex]
        : nullptr;
}
    
void AppStage_TrackerSettings::set_selectedTrackerIndex(int index)
{
    m_selectedTrackerIndex = 
        (index != -1 && index < m_trackerInfos.count)
        ? index
        : m_selectedTrackerIndex;
}
    
int AppStage_TrackerSettings::get_tracker_count() const
{
    return static_cast<int>(m_trackerInfos.count); 
}

int AppStage_TrackerSettings::get_tracker_Index() const
{
    return m_selectedTrackerIndex;
}

const PSVRClientHMDInfo *AppStage_TrackerSettings::get_selected_hmd()
{
    const PSVRClientHMDInfo *hmd = NULL;

    if (m_selectedHmdIndex != -1)
    {
        hmd = &m_hmdInfos.hmds[m_selectedHmdIndex];
    }

    return hmd;
}

void AppStage_TrackerSettings::renderUI()
{
    const char *k_window_title = "Tracker Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(350, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_trackerInfos.count > 0)
        {
            const PSVRClientTrackerInfo &trackerInfo = m_trackerInfos.trackers[m_selectedTrackerIndex];

            if (m_selectedTrackerIndex > 0)
            {
                if (ImGui::Button("<##TrackerIndex"))
                {
                    --m_selectedTrackerIndex;
                }
            }
            else {
                if (ImGui::Button("<##TrackerIndex"))
                {
                    m_selectedTrackerIndex = static_cast<int>(m_trackerInfos.count) -1;
                }
            }
            ImGui::SameLine();
            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
            ImGui::SameLine();
            if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.count))
            {
                if (ImGui::Button(">##TrackerIndex"))
                {
                    ++m_selectedTrackerIndex;
                }
            }
            else {
                if (ImGui::Button(">##TrackerIndex"))
                {
                    m_selectedTrackerIndex = 0;
                }
            }

            ImGui::BulletText("Tracker ID: %d", trackerInfo.tracker_id);

            switch (trackerInfo.tracker_type)
            {
            case PSVRTracker_PS3Eye:
                {
                    ImGui::BulletText("Camera Type: PS3 Eye");
                } break;
            case PSVRTracker_PS4Camera:
                {
                    ImGui::BulletText("Camera Type: PS4 Camera");
                } break;
            case PSVRTracker_GenericMonoCamera:
                {
                    ImGui::BulletText("Camera Type: Generic Mono");
                } break;
			case PSVRTracker_GenericStereoCamera:
                {
                    ImGui::BulletText("Camera Type: Generic Stereo");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            switch (trackerInfo.tracker_driver)
            {
            case PSVRDriver_LIBUSB:
                {
                    ImGui::BulletText("Controller Type: LIBUSB");
                } break;
            case PSVRDriver_GENERIC_WEBCAM:
                {
                    ImGui::BulletText("Controller Type: Generic Webcam");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::BulletText("Device Path: ");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", trackerInfo.device_path);

            ImGui::Spacing();

            if (ImGui::Button("Test Video Feed"))
            {
                m_app->setAppStage(AppStage_TrackerTest::APP_STAGE_NAME);
            }

            if (trackerInfo.tracker_intrinsics.intrinsics_type == PSVR_MONO_TRACKER_INTRINSICS)
            {
                if (ImGui::Button("Calibrate Mono Tracker"))
                {
                    m_app->getAppStage<AppStage_MonoCalibration>()->setBypassCalibrationFlag(false);
                    m_app->setAppStage(AppStage_MonoCalibration::APP_STAGE_NAME);
                }
                if (ImGui::Button("Test Mono Calibration"))
                {
                    m_app->getAppStage<AppStage_MonoCalibration>()->setBypassCalibrationFlag(true);
                    m_app->setAppStage(AppStage_MonoCalibration::APP_STAGE_NAME);
                }
            }
            else if (trackerInfo.tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS)
            {
                if (ImGui::Button("Calibrate Stereo Tracker"))
                {
                    m_app->getAppStage<AppStage_StereoCalibration>()->setBypassCalibrationFlag(false);
                    m_app->setAppStage(AppStage_StereoCalibration::APP_STAGE_NAME);
                }
                if (ImGui::Button("Test Stereo Calibration"))
                {
                    m_app->getAppStage<AppStage_StereoCalibration>()->setBypassCalibrationFlag(true);
                    m_app->setAppStage(AppStage_StereoCalibration::APP_STAGE_NAME);
                }
            }
        }
        else
        {
            ImGui::Text("No trackers");
        }

        ImGui::Separator();

        if (m_trackerInfos.count > 0)
        {
            if (m_hmdInfos.count > 0)
            {
                int hmdID = (m_selectedHmdIndex != -1) ? m_hmdInfos.hmds[m_selectedHmdIndex].hmd_id : -1;

                ImGui::Separator();

                if (m_selectedHmdIndex > 0)
                {
                    if (ImGui::Button("<##HMD"))
                    {
                        --m_selectedHmdIndex;
                    }
                    ImGui::SameLine();
                }

                if (m_selectedHmdIndex != -1)
                {
                    const PSVRClientHMDInfo &hmdInfo = m_hmdInfos.hmds[m_selectedHmdIndex];
                    const char *colors[] = { "Magenta","Cyan","Yellow","Red","Green","Blue" };

                    if (hmdInfo.hmd_type == PSVRHmd_Morpheus)
                    {
                        if (0 <= hmdInfo.tracking_color_type && hmdInfo.tracking_color_type < PSVRTrackingColorType_MaxColorTypes) 
                        {
                            ImGui::Text("HMD: %d (Morpheus) - %s",
                                m_selectedHmdIndex,
                                colors[hmdInfo.tracking_color_type]);
                        }
                        else
                        {
                            ImGui::Text("HMD: %d (Morpheus)", m_selectedHmdIndex);
                        }
                    }
                    else if (hmdInfo.hmd_type == PSVRHmd_Virtual)
                    {
                        if (0 <= hmdInfo.tracking_color_type && hmdInfo.tracking_color_type < PSVRTrackingColorType_MaxColorTypes) 
                        {
                            ImGui::Text("HMD: %d (Virtual) - %s",
                                m_selectedHmdIndex,
                                colors[hmdInfo.tracking_color_type]);
                        }
                        else
                        {
                            ImGui::Text("HMD: %d (Virtual)", m_selectedHmdIndex);
                        }
                    }
                }

                if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.count))
                {
                    ImGui::SameLine();
                    if (ImGui::Button(">##HMD"))
                    {
                        ++m_selectedHmdIndex;
                    }
                }

                if (ImGui::Button("Calibrate HMD Tracking Colors"))
                {                        
                    const PSVRClientHMDInfo *hmd = get_selected_hmd();
                    if (hmd != nullptr) 
                    {
                        m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(hmd->hmd_id);
                        m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(hmd->tracking_color_type);
                    }

                    m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
                }

                if (ImGui::Button("Test Tracking Pose##HMDTrackingPose"))
                {
                    AppStage_HMDTrackingTest::enterStageAndTestTracking(m_app, hmdID);
                }
                ImGui::SameLine();
                if (ImGui::Button("Test Tracking Video##HMDTrackingVideo"))
                {
                    m_app->getAppStage<AppStage_HMDTrackingTest>()->set_tracker_id(m_selectedTrackerIndex);
                    AppStage_HMDTrackingTest::enterStageAndTestTracking(m_app, hmdID);
                }
            }
        }

        ImGui::Separator();

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;
    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::pendingHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for server response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::failedHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to get server response!");

        if (ImGui::Button("Retry"))
        {
            request_tracker_list();
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

bool AppStage_TrackerSettings::onClientAPIEvent(
    PSVREventType event_type)
{
    bool bHandled = false;

    switch (event_type)
    {
    case PSVREventType::PSVREvent_trackerListUpdated:
        {
            bHandled = true;
            request_tracker_list();
        } break;
    }

    return bHandled;
}

void AppStage_TrackerSettings::request_tracker_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingTrackerListRequest)
    {
        m_menuState = AppStage_TrackerSettings::pendingTrackerListRequest;

        // Tell the PSVR service that we we want a list of trackers connected to this machine
        PSVRTrackerList tracker_list;
        if (PSVR_GetTrackerList(&tracker_list) == PSVRResult_Success)
        {
            handle_tracker_list_response(tracker_list);
        }
        else
        {
            m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        }
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    const PSVRTrackerList &tracker_list)
{
    int oldSelectedTrackerIndex= m_selectedTrackerIndex;

    m_selectedTrackerIndex = -1;
    m_trackerInfos= tracker_list;

    if (oldSelectedTrackerIndex != -1)
    {
        // Maintain the same position in the list if possible
        m_selectedTrackerIndex= 
            (oldSelectedTrackerIndex < m_trackerInfos.count) 
            ? oldSelectedTrackerIndex
            : 0;
    }
    else
    {
        m_selectedTrackerIndex= (m_trackerInfos.count > 0) ? 0 : -1;
    }

    // Request the list of controllers next
    request_hmd_list();
}

void AppStage_TrackerSettings::request_hmd_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingHmdListRequest)
    {
        m_menuState = AppStage_TrackerSettings::pendingHmdListRequest;

        // Tell the PSVR service that we we want a list of HMDs connected to this machine
        PSVRHmdList hmd_list;
        if (PSVR_GetHmdList(&hmd_list) == PSVRResult_Success)
        {
            handle_hmd_list_response(hmd_list);
        }
        else
        {
            m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        }
    }
}

void AppStage_TrackerSettings::handle_hmd_list_response(
    const PSVRHmdList &hmd_list)
{
    int oldSelectedHmdIndex = m_selectedHmdIndex;

    m_hmdInfos= hmd_list;

    if (oldSelectedHmdIndex != -1)
    {
        // Maintain the same position in the list if possible
        m_selectedHmdIndex =
            (oldSelectedHmdIndex < m_hmdInfos.count)
            ? oldSelectedHmdIndex
            : -1;
    }
    else
    {
        m_selectedHmdIndex = (m_hmdInfos.count > 0) ? 0 : -1;
    }

    m_menuState = AppStage_TrackerSettings::idle;
}
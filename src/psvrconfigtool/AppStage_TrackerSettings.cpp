//-- inludes -----
#include "AppStage_TrackerSettings.h"
#include "AppStage_TestTracker.h"
#include "AppStage_ColorCalibration.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_DistortionCalibration.h"
#include "AppStage_StereoCalibration.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "PSVRClient_CAPI.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSVRProtocolInterface.h"
#include "PSVRProtocol.pb.h"

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
    , m_selectedControllerIndex(-1)
    , m_selectedHmdIndex(-1)
    , m_gotoControllerColorCalib(false)
    , m_gotoHMDColorCalib(false)
    , m_gotoTestControllerTracking(false)
    , m_gotoTrackingControllerVideo(false)
    , m_gotoTestHmdTracking(false)
    , m_gotoTrackingHmdVideo(false)
    , m_gotoTrackingVideoALL(false)
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
            const PSVRClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

            switch (trackerInfo.tracker_type)
            {
            case PSVRTracker_PS3Eye:
                {
                    glm::mat4 scale3 = glm::scale(glm::mat4(1.f), glm::vec3(3.f, 3.f, 3.f));
                    drawPS3EyeModel(scale3);
                } break;
            case PSVRTracker_VirtualStereoCamera:
                {
                    glm::mat4 left = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(-5.f, 0.f, 0.f)), glm::vec3(3.f, 3.f, 3.f));
                    glm::mat4 right = glm::scale(glm::translate(glm::mat4(1.f), glm::vec3(5.f, 0.f, 0.f)), glm::vec3(3.f, 3.f, 3.f));

                    drawPS3EyeModel(left);
                    drawPS3EyeModel(right);
                } break;

            default:
                assert(0 && "Unreachable");
            }
        }
    } break;

    case eTrackerMenuState::pendingSearchForNewTrackersRequest:
    case eTrackerMenuState::pendingTrackerListRequest:
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::pendingControllerListRequest:
    case eTrackerMenuState::failedControllerListRequest:
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
        ? &m_trackerInfos[m_selectedTrackerIndex]
        : nullptr;
}
    
void AppStage_TrackerSettings::set_selectedTrackerIndex(int index)
{
    m_selectedTrackerIndex = 
        (index != -1 && index < m_trackerInfos.size())
        ? index
        : m_selectedTrackerIndex;
}
    
void AppStage_TrackerSettings::set_selectedControllerIndex(int index)
{
    m_selectedControllerIndex =
        (index > -2 && index < m_controllerInfos.size())
        ? index
        : m_selectedControllerIndex;
}

int AppStage_TrackerSettings::get_tracker_count() const
{
    return static_cast<int>(m_trackerInfos.size()); 
}

int AppStage_TrackerSettings::get_tracker_Index() const
{
    return m_selectedTrackerIndex;
}

int AppStage_TrackerSettings::get_controller_count() const
{
    return static_cast<int>(m_controllerInfos.size());
}

const AppStage_TrackerSettings::ControllerInfo * AppStage_TrackerSettings::get_controller_info(int index) const
{
    return &m_controllerInfos[index];
}

const AppStage_TrackerSettings::ControllerInfo *AppStage_TrackerSettings::get_selected_controller() {
    const ControllerInfo *controller = NULL;

    if (m_selectedControllerIndex != -1)
    {
        const AppStage_TrackerSettings::ControllerInfo &controllerInfo =
            m_controllerInfos[m_selectedControllerIndex];

        controller = &controllerInfo;
    }

    return controller;
}

const AppStage_TrackerSettings::HMDInfo *AppStage_TrackerSettings::get_selected_hmd()
{
    const HMDInfo *hmd = NULL;

    if (m_selectedHmdIndex != -1)
    {
        const AppStage_TrackerSettings::HMDInfo &hmdinfo =
            m_hmdInfos[m_selectedHmdIndex];

        hmd = &hmdinfo;
    }

    return hmd;
}

void AppStage_TrackerSettings::renderUI()
{
    const char *k_window_title = "Tracker Settings";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_AlwaysAutoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eTrackerMenuState::idle:
    {
        ImGui::SetNextWindowPosCenter();
        //ImGui::SetNextWindowSize(ImVec2(300, 400));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        //###HipsterSloth $TODO The tracker restart currently takes longer than it does
        // just to close and re-open the service.
        // For now let's just disable this until we can make this more performant.
        //if (ImGui::Button("Refresh Tracker List"))
        //{
        //    request_search_for_new_trackers();
        //}

        //ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
            const PSVRClientTrackerInfo &trackerInfo = m_trackerInfos[m_selectedTrackerIndex];

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
                    m_selectedTrackerIndex = static_cast<int>(m_trackerInfos.size()) -1;
                }
            }
            ImGui::SameLine();
            ImGui::Text("Tracker: %d", m_selectedTrackerIndex);
            ImGui::SameLine();
            if (m_selectedTrackerIndex + 1 < static_cast<int>(m_trackerInfos.size()))
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
            case PSVRTracker_VirtualStereoCamera:
                {
                    ImGui::BulletText("Camera Type: Virtual Stereo");
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
            case PSVRDriver_CL_EYE:
                {
                    ImGui::BulletText("Controller Type: CLEye");
                } break;
            case PSVRDriver_CL_EYE_MULTICAM:
                {
                    ImGui::BulletText("Controller Type: CLEye(Multicam SDK)");
                } break;
            case PSVRDriver_GENERIC_WEBCAM:
                {
                    ImGui::BulletText("Controller Type: Generic Webcam");
                } break;
            default:
                assert(0 && "Unreachable");
            }

            ImGui::BulletText("Shared Mem Name: %s", trackerInfo.shared_memory_name);
            ImGui::BulletText("Device Path: ");
            ImGui::SameLine();
            ImGui::TextWrapped("%s", trackerInfo.device_path);

            if (m_app->getIsLocalServer())
            {
                if (ImGui::Button("Test Video Feed"))
                {
                    m_app->setAppStage(AppStage_TestTracker::APP_STAGE_NAME);
                }

                if (trackerInfo.tracker_intrinsics.intrinsics_type == PSVRTrackerIntrinsics::PSVR_STEREO_TRACKER_INTRINSICS)
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
                else
                {
                    if (ImGui::Button("Calibrate Tracker Distortion"))
                    {
                        m_app->setAppStage(AppStage_DistortionCalibration::APP_STAGE_NAME);
                    }
                }
            }
            else
            {
                ImGui::TextDisabled("Test Video Feed");
                ImGui::TextDisabled("Calibrate Tracker Distortion");
            }
        }
        else
        {
            ImGui::Text("No trackers");
        }

        ImGui::Separator();

        if (m_trackerInfos.size() > 0)
        {
            if (m_controllerInfos.size() > 0)
            {
                if (m_selectedControllerIndex >= 0)
                {
                    if (ImGui::Button("<##Controller"))
                    {
                        --m_selectedControllerIndex;
                    }
                    ImGui::SameLine();
                }
                
                if (m_selectedControllerIndex != -1)
                {
                    const AppStage_TrackerSettings::ControllerInfo &controllerInfo = 
                        m_controllerInfos[m_selectedControllerIndex];

                    if (controllerInfo.ControllerType == PSVRController_Move ||
                        controllerInfo.ControllerType == PSVRController_Virtual)
                    { 
                        const char * szControllerLabel= (controllerInfo.ControllerType == PSVRController_Move) ? "PSVR" : "Virtual";

                        if (0 <= controllerInfo.TrackingColorType && controllerInfo.TrackingColorType < PSVRTrackingColorType_MaxColorTypes) 
                        {
                            const char *colors[] = { "Magenta","Cyan","Yellow","Red","Green","Blue" };

                            ImGui::Text("Controller: %d (%s) - %s", 
                                m_selectedControllerIndex,
                                szControllerLabel,
                                colors[controllerInfo.TrackingColorType]);
                        }
                        else 
                        {
                            ImGui::Text("Controller: %d (%s)", m_selectedControllerIndex, szControllerLabel);
                        }
                    }
                    else
                    {
                        ImGui::Text("Controller: %d (DualShock4)", m_selectedControllerIndex);
                    }
                }
                else
                {
                    ImGui::Text("Controller: <ALL>");
                }

                if (m_selectedControllerIndex + 1 < static_cast<int>(m_controllerInfos.size()))
                {
                    ImGui::SameLine();
                    if (ImGui::Button(">##Controller"))
                    {
                        ++m_selectedControllerIndex;
                    }
                }

                {
                    int controllerID = (m_selectedControllerIndex != -1) ? m_controllerInfos[m_selectedControllerIndex].ControllerID : -1;

                    if (m_app->getIsLocalServer())
                    {
                        if (ImGui::Button("Calibrate Controller Tracking Colors") || m_gotoControllerColorCalib)
                        {
                            const ControllerInfo *controller = get_selected_controller();
                            if (controller != NULL) {
                                m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(controller->ControllerID);
                                m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(-1);
                                m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(controller->TrackingColorType);
                            }
                            m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
                        }

                        if (ImGui::Button("Compute Tracker Poses Using Controller"))
                        {
                            AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithController(m_app, controllerID);
                        }

                    }
                    else
                    {
                        ImGui::TextDisabled("Calibrate Controller Tracking Colors");
                        ImGui::TextDisabled("Compute Tracker Poses Using Controller");
                    }

                    if (ImGui::Button("Test Tracking Pose##ControllerTrackingPose") || m_gotoTestControllerTracking)
                    {
                        if (m_gotoTestControllerTracking) m_gotoTestControllerTracking = false;
                        AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, controllerID, -1);
                    }
                    ImGui::SameLine();
                    if (ImGui::Button("Test Tracking Video##ControllerTrackingVideo") || m_gotoTrackingControllerVideo)
                    {
                        if (m_gotoTrackingControllerVideo) m_gotoTrackingControllerVideo = false;
                        m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
                        AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, controllerID, -1);
                    }
                    if (m_gotoTrackingVideoALL)
                    {
                        m_gotoTrackingVideoALL = false;
                        m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
                        AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, -1);
                    }
                }
            }

            if (m_hmdInfos.size() > 0)
            {
                int hmdID = (m_selectedHmdIndex != -1) ? m_hmdInfos[m_selectedHmdIndex].HmdID : -1;

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
                    const AppStage_TrackerSettings::HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];
                    const char *colors[] = { "Magenta","Cyan","Yellow","Red","Green","Blue" };

                    if (hmdInfo.HmdType == PSVRHmd_Morpheus)
                    {
                        if (0 <= hmdInfo.TrackingColorType && hmdInfo.TrackingColorType < PSVRTrackingColorType_MaxColorTypes) 
                        {
                            ImGui::Text("HMD: %d (Morpheus) - %s",
                                m_selectedHmdIndex,
                                colors[hmdInfo.TrackingColorType]);
                        }
                        else
                        {
                            ImGui::Text("HMD: %d (Morpheus)", m_selectedHmdIndex);
                        }
                    }
                    else if (hmdInfo.HmdType == PSVRHmd_Virtual)
                    {
                        if (0 <= hmdInfo.TrackingColorType && hmdInfo.TrackingColorType < PSVRTrackingColorType_MaxColorTypes) 
                        {
                            ImGui::Text("HMD: %d (Virtual) - %s",
                                m_selectedHmdIndex,
                                colors[hmdInfo.TrackingColorType]);
                        }
                        else
                        {
                            ImGui::Text("HMD: %d (Virtual)", m_selectedHmdIndex);
                        }
                    }
                }

                if (m_selectedHmdIndex + 1 < static_cast<int>(m_hmdInfos.size()))
                {
                    ImGui::SameLine();
                    if (ImGui::Button(">##HMD"))
                    {
                        ++m_selectedHmdIndex;
                    }
                }

                if (m_app->getIsLocalServer())
                {
                    if (ImGui::Button("Calibrate HMD Tracking Colors") || m_gotoHMDColorCalib)
                    {
                        const HMDInfo *hmd = get_selected_hmd();
                        if (hmd != NULL) 
                        {
                            m_app->getAppStage<AppStage_ColorCalibration>()->set_override_controller_id(-1);
                            m_app->getAppStage<AppStage_ColorCalibration>()->set_override_hmd_id(hmd->HmdID);
                            m_app->getAppStage<AppStage_ColorCalibration>()->set_override_tracking_color(hmd->TrackingColorType);
                        }

                        m_app->setAppStage(AppStage_ColorCalibration::APP_STAGE_NAME);
                    }

                    if (m_selectedHmdIndex != -1)
                    {
                        const AppStage_TrackerSettings::HMDInfo &hmdInfo = m_hmdInfos[m_selectedHmdIndex];

                        if (hmdInfo.HmdType == PSVRHmd_Virtual)
                        {
                            if (ImGui::Button("Compute Tracker Poses Using HMD"))
                            {
                                AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithHMD(m_app, hmdID);
                            }
                        }
                    }
                }
                else
                {
                    ImGui::TextDisabled("Calibrate HMD Tracking Colors");
                    ImGui::TextDisabled("Compute Tracker Poses");
                }

                if (ImGui::Button("Test Tracking Pose##HMDTrackingPose") || m_gotoTestHmdTracking)
                {
                    m_gotoTestHmdTracking = false;
                    AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, hmdID);
                }
                ImGui::SameLine();
                if (ImGui::Button("Test Tracking Video##HMDTrackingVideo") || m_gotoTrackingHmdVideo)
                {
                    m_gotoTrackingHmdVideo = false;
                    m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
                    AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, hmdID);
                }
                if (m_gotoTrackingVideoALL)
                {
                    m_gotoTrackingVideoALL = false;
                    m_app->getAppStage<AppStage_ComputeTrackerPoses>()->set_tracker_id(m_selectedTrackerIndex);
                    AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(m_app, -1, -1);
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
    case eTrackerMenuState::pendingControllerListRequest:
    case eTrackerMenuState::pendingHmdListRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(300, 150));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for server response...");

        ImGui::End();
    } break;
    case eTrackerMenuState::failedTrackerListRequest:
    case eTrackerMenuState::failedControllerListRequest:
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
    PSVREventMessage::eEventType event, 
    PSVREventDataHandle opaque_event_handle)
{
    bool bHandled = false;

    switch (event)
    {
    case  PSVREventMessage::PSVREvent_controllerListUpdated:
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
        PSVRRequestID requestId;
        PSVR_GetTrackerListAsync(&requestId);
        PSVR_RegisterCallback(requestId, AppStage_TrackerSettings::handle_tracker_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_tracker_list_response(
    const PSVRResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    switch (response_message->result_code)
    {
    case PSVRResult_Success:
        {
            assert(response_message->payload_type == PSVRResponseMessage::_responsePayloadType_TrackerList);
            const PSVRTrackerList &tracker_list= response_message->payload.tracker_list;
            int oldSelectedTrackerIndex= thisPtr->m_selectedTrackerIndex;

            thisPtr->m_selectedTrackerIndex = -1;
            thisPtr->m_trackerInfos.clear();

            for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
            {
                const PSVRClientTrackerInfo &TrackerInfo = tracker_list.trackers[tracker_index];

                thisPtr->m_trackerInfos.push_back(TrackerInfo);
            }

            if (oldSelectedTrackerIndex != -1)
            {
                // Maintain the same position in the list if possible
                thisPtr->m_selectedTrackerIndex= 
                    (oldSelectedTrackerIndex < thisPtr->m_trackerInfos.size()) 
                    ? oldSelectedTrackerIndex
                    : 0;
            }
            else
            {
                thisPtr->m_selectedTrackerIndex= (thisPtr->m_trackerInfos.size() > 0) ? 0 : -1;
            }

            // Request the list of controllers next
            thisPtr->request_controller_list();
        } break;

    case PSVRResult_Error:
    case PSVRResult_Canceled:
    case PSVRResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TrackerSettings::failedTrackerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_controller_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingControllerListRequest)
    {
        m_menuState= AppStage_TrackerSettings::pendingControllerListRequest;

        // Tell the PSVR service that we we want a list of controllers connected to this machine
        RequestPtr request(new PSVRProtocol::Request());
        request->set_type(PSVRProtocol::Request_RequestType_GET_CONTROLLER_LIST);

        // Don't need the usb controllers
        request->mutable_request_get_controller_list()->set_include_usb_controllers(false);

        PSVRRequestID request_id;
        PSVR_SendOpaqueRequest(&request, &request_id);
        PSVR_RegisterCallback(request_id, AppStage_TrackerSettings::handle_controller_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_controller_list_response(
    const PSVRResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr= static_cast<AppStage_TrackerSettings *>(userdata);

    const PSVRResult ResultCode = response_message->result_code;
    const PSVRResponseHandle response_handle = response_message->opaque_response_handle;

    switch(ResultCode)
    {
        case PSVRResult_Success:
        {
            const PSVRProtocol::Response *response= GET_PSVRPROTOCOL_RESPONSE(response_handle);
            int oldSelectedControllerIndex= thisPtr->m_selectedControllerIndex;

            thisPtr->m_controllerInfos.clear();

            for (int controller_index= 0; controller_index < response->result_controller_list().controllers_size(); ++controller_index)
            {
                const auto &ControllerResponse= response->result_controller_list().controllers(controller_index);

                AppStage_TrackerSettings::ControllerInfo ControllerInfo;

                ControllerInfo.ControllerID= ControllerResponse.controller_id();
                ControllerInfo.TrackingColorType = (PSVRTrackingColorType)ControllerResponse.tracking_color_type();

                switch(ControllerResponse.controller_type())
                {
                case PSVRProtocol::PSVR:
                    ControllerInfo.ControllerType = PSVRController_Move;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSVRProtocol::PSNAVI:
                    ControllerInfo.ControllerType = PSVRController_Navi;
                    break;
                case PSVRProtocol::PSDUALSHOCK4:
                    ControllerInfo.ControllerType = PSVRController_DualShock4;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                case PSVRProtocol::VIRTUALCONTROLLER:
                    ControllerInfo.ControllerType = PSVRController_Virtual;
                    thisPtr->m_controllerInfos.push_back(ControllerInfo);
                    break;
                default:
                    assert(0 && "unreachable");
                }			                
            }

            if (oldSelectedControllerIndex != -1)
            {
                // Maintain the same position in the list if possible
                thisPtr->m_selectedControllerIndex= 
                    (oldSelectedControllerIndex < thisPtr->m_controllerInfos.size()) 
                    ? oldSelectedControllerIndex
                    : -1;
            }
            else
            {
                thisPtr->m_selectedControllerIndex= (thisPtr->m_controllerInfos.size() > 0) ? 0 : -1;
            }

            // Request the list of HMDs next
            thisPtr->request_hmd_list();
        } break;

        case PSVRResult_Error:
        case PSVRResult_Canceled:
        case PSVRResult_Timeout:
        { 
            thisPtr->m_menuState= AppStage_TrackerSettings::failedControllerListRequest;
        } break;
    }
}

void AppStage_TrackerSettings::request_hmd_list()
{
    if (m_menuState != AppStage_TrackerSettings::pendingHmdListRequest)
    {
        m_menuState = AppStage_TrackerSettings::pendingHmdListRequest;

        // Tell the PSVR service that we we want a list of HMDs connected to this machine
        RequestPtr request(new PSVRProtocol::Request());
        request->set_type(PSVRProtocol::Request_RequestType_GET_HMD_LIST);

        PSVRRequestID request_id;
        PSVR_SendOpaqueRequest(&request, &request_id);
        PSVR_RegisterCallback(request_id, AppStage_TrackerSettings::handle_hmd_list_response, this);
    }
}

void AppStage_TrackerSettings::handle_hmd_list_response(
    const PSVRResponseMessage *response_message,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    const PSVRResult ResultCode = response_message->result_code;
    const PSVRResponseHandle response_handle = response_message->opaque_response_handle;

    switch (ResultCode)
    {
    case PSVRResult_Success:
    {
        const PSVRProtocol::Response *response = GET_PSVRPROTOCOL_RESPONSE(response_handle);
        int oldSelectedHmdIndex = thisPtr->m_selectedHmdIndex;

        thisPtr->m_hmdInfos.clear();

        for (int hmd_index = 0; hmd_index < response->result_hmd_list().hmd_entries_size(); ++hmd_index)
        {
            const auto &HmdResponse = response->result_hmd_list().hmd_entries(hmd_index);

            AppStage_TrackerSettings::HMDInfo HmdInfo;

            HmdInfo.HmdID = HmdResponse.hmd_id();
            HmdInfo.TrackingColorType = (PSVRTrackingColorType)HmdResponse.tracking_color_type();

            switch (HmdResponse.hmd_type())
            {
            case PSVRProtocol::Morpheus:
                HmdInfo.HmdType = PSVRHmd_Morpheus;
                thisPtr->m_hmdInfos.push_back(HmdInfo);
                break;
            case PSVRProtocol::VirtualHMD:
                HmdInfo.HmdType = PSVRHmd_Virtual;
                thisPtr->m_hmdInfos.push_back(HmdInfo);
                break;
            default:
                assert(0 && "unreachable");
            }
        }

        if (oldSelectedHmdIndex != -1)
        {
            // Maintain the same position in the list if possible
            thisPtr->m_selectedHmdIndex =
                (oldSelectedHmdIndex < thisPtr->m_hmdInfos.size())
                ? oldSelectedHmdIndex
                : -1;
        }
        else
        {
            thisPtr->m_selectedHmdIndex = (thisPtr->m_hmdInfos.size() > 0) ? 0 : -1;
        }

        thisPtr->m_menuState = AppStage_TrackerSettings::idle;
    } break;

    case PSVRResult_Error:
    case PSVRResult_Canceled:
    case PSVRResult_Timeout:
    {
        thisPtr->m_menuState = AppStage_TrackerSettings::failedControllerListRequest;
    } break;
    }
}

void AppStage_TrackerSettings::request_search_for_new_trackers()
{
    // Tell the PSVR service that we want see if new trackers are connected.
    RequestPtr request(new PSVRProtocol::Request());
    request->set_type(PSVRProtocol::Request_RequestType_SEARCH_FOR_NEW_TRACKERS);

    m_menuState = AppStage_TrackerSettings::pendingSearchForNewTrackersRequest;
    m_selectedTrackerIndex = -1;
    m_trackerInfos.clear();

    PSVRRequestID request_id;
    PSVR_SendOpaqueRequest(&request, &request_id);
    PSVR_RegisterCallback(request_id, AppStage_TrackerSettings::handle_search_for_new_trackers_response, this);
}

void AppStage_TrackerSettings::handle_search_for_new_trackers_response(
    const PSVRResponseMessage *response,
    void *userdata)
{
    AppStage_TrackerSettings *thisPtr = static_cast<AppStage_TrackerSettings *>(userdata);

    thisPtr->request_tracker_list();
}

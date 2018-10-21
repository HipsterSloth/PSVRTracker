//-- inludes -----
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
#include "AppSubStage_CalibrateWithMat.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "Logger.h"
#include "MathUtility.h"
#include "MathTypeConversion.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>
#include <sstream>

//-- statics ----
const char *AppStage_ComputeTrackerPoses::APP_STAGE_NAME = "ComputeTrackerPoses";

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_psmove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static void drawController(const PSVRController *controllerView, const glm::mat4 &transform, const PSVRTrackingColorType trackingColorType);
static void drawHMD(const PSVRHeadMountedDisplay *hmdView, const glm::mat4 &transform, const PSVRTrackingColorType trackingColorType);

//-- public methods -----
AppStage_ComputeTrackerPoses::AppStage_ComputeTrackerPoses(App *app)
    : AppStage(app)
    , m_menuState(AppStage_ComputeTrackerPoses::inactive)
    , m_renderTrackerIndex(0)
    , m_pCalibrateWithMat(new AppSubStage_CalibrateWithMat(this))
    , m_bSkipCalibration(false)
    , m_ShowTrackerVideoId(-1)
    , m_bShowAlignment(false)
    , m_AlignmentOffset(0.f)
    , m_overrideControllerId(-1)
    , m_overrideHmdId(-1)
{ 
    m_renderTrackerIter = m_trackerViews.end();
}

AppStage_ComputeTrackerPoses::~AppStage_ComputeTrackerPoses()
{
    delete m_pCalibrateWithMat;
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithController(App *app, PSVRControllerID reqeusted_controller_id)
{
    AppStage_ComputeTrackerPoses *appStage= app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = false;
    appStage->m_overrideControllerId = reqeusted_controller_id;
    appStage->m_overrideHmdId = -1;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndCalibrateTrackersWithHMD(class App *app, PSVRHmdID reqeusted_hmd_id)
{
    AppStage_ComputeTrackerPoses *appStage = app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = false;
    appStage->m_overrideControllerId = -1;
    appStage->m_overrideHmdId = reqeusted_hmd_id;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enterStageAndTestTrackers(App *app, PSVRControllerID reqeusted_controller_id, PSVRHmdID requested_hmd_id)
{
    AppStage_ComputeTrackerPoses *appStage = app->getAppStage<AppStage_ComputeTrackerPoses>();
    appStage->m_bSkipCalibration = true;
    appStage->m_overrideControllerId = reqeusted_controller_id;
    appStage->m_overrideHmdId = requested_hmd_id;

    app->setAppStage(AppStage_ComputeTrackerPoses::APP_STAGE_NAME);
}

void AppStage_ComputeTrackerPoses::enter()
{
    // Only get the controller list if
    // A) No specific controller or HMD was requests
    // B) A specific controller was requested
    if ((m_overrideControllerId == -1 && m_overrideHmdId == -1) || 
        m_overrideControllerId != -1)
    {
        // Kick off this async request chain:
        // controller list request
        // -> controller start request
        // -> hmd list request (if no specific controller was requested)
        // -> hmd start request (if no specific controller was requested)
        // -> tracker list request
        // -> tracker start request
        request_controller_list();
    }
    else
    {
        // Kick off this async request chain 
        // hmd list request
        // -> hmd start request
        // -> tracker list request
        // -> tracker start request
        request_hmd_list();
    }

    m_app->setCameraType(_cameraFixed);
}

void AppStage_ComputeTrackerPoses::exit()
{
    release_devices();

    setState(eMenuState::inactive);
}

void AppStage_ComputeTrackerPoses::update()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        update_tracker_video();
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->update();

            if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepSuccess)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::testTracking);
            }
            else if (m_pCalibrateWithMat->getMenuState() == AppSubStage_CalibrateWithMat::calibrateStepFailed)
            {
                setState(AppStage_ComputeTrackerPoses::eMenuState::calibrateStepFailed);
            }
        }
        break;
    case eMenuState::testTracking:
        break;
    case eMenuState::showTrackerVideo:
        update_tracker_video();
        break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::render()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        {
            render_tracker_video();
        } break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->render();
        break;
    case eMenuState::testTracking:
        {
            // Draw the chaperone origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            // Draw the frustum for each tracking camera.
            // The frustums are defined in PSMove tracking space.
            // We need to transform them into chaperone space to display them along side the HMD.
            for (t_tracker_state_map_iterator tracker_iter = m_trackerViews.begin(); tracker_iter != m_trackerViews.end(); ++tracker_iter)
            {
                const PSVRTracker *trackerView = tracker_iter->second.trackerView;
                const int tracker_id= trackerView->tracker_info.tracker_id;
                const PSVRPosef trackerPose = trackerView->tracker_info.tracker_pose;
                const glm::mat4 trackerMat4 = PSVR_posef_to_glm_mat4(trackerPose);

                PSVRFrustum frustum;
                PSVR_GetTrackerFrustum(tracker_id, &frustum);

                // use color depending on tracking status
                glm::vec3 color= does_tracker_see_any_device(trackerView) ? k_psmove_frustum_color : k_psmove_frustum_color_no_track;

                drawTextAtWorldPosition(glm::mat4(1.f), PSVR_vector3f_to_glm_vec3(trackerPose.Position), "#%d", tracker_id);
                drawTransformedFrustum(glm::mat4(1.f), &frustum, color);

                drawTransformedAxes(trackerMat4, 20.f);
            }

            // Draw each controller model
            for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
            {
                const PSVRController *controllerView = controller_iter->second.controllerView;
                const PSVRTrackingColorType trackingColorType= controller_iter->second.trackingColorType;

                PSVRPosef controllerPose;
                PSVRPhysicsData physicsData;
                switch (controllerView->ControllerType)
                {
                case PSVRControllerType::PSVRController_Move:
                    controllerPose = controllerView->ControllerState.PSMoveState.Pose;
                    physicsData= controllerView->ControllerState.PSMoveState.PhysicsData;
                    break;
                case PSVRControllerType::PSVRController_DualShock4:
                    controllerPose = controllerView->ControllerState.DS4State.Pose;
                    physicsData= controllerView->ControllerState.DS4State.PhysicsData;
                    break;
                }
                glm::mat4 controllerMat4 = PSVR_posef_to_glm_mat4(controllerPose);

                if (m_controllerViews.size() > 1)
                {
                    drawTextAtWorldPosition(glm::mat4(1.f), PSVR_vector3f_to_glm_vec3(controllerPose.Position), "#%d", controllerView->ControllerID);
                }
                drawController(controllerView, controllerMat4, trackingColorType);
                drawTransformedAxes(controllerMat4, 10.f);

                // Draw the acceleration and velocity arrows
                {
                    const glm::mat4 originMat4= glm::translate(glm::mat4(1.f), PSVR_vector3f_to_glm_vec3(controllerPose.Position));
                    const glm::vec3 vel_endpoint = PSVR_vector3f_to_glm_vec3(physicsData.LinearVelocityCmPerSec);
                    const glm::vec3 acc_endpoint = PSVR_vector3f_to_glm_vec3(physicsData.LinearAccelerationCmPerSecSqr)*PSVR_CENTIMETERS_TO_METERS;
                    
                    const float vel= glm::length(vel_endpoint);
                    if (vel > k_positional_epsilon)
                    {
                        drawArrow(originMat4, glm::vec3(0.f), vel_endpoint, 0.1f, glm::vec3(0.f, 1.f, 1.f));
                        //drawTextAtWorldPosition(originMat4, vel_endpoint, "v=%.2fcm/s", vel);
                    }

                    const float acc = glm::length(acc_endpoint);
                    if (acc > k_positional_epsilon)
                    {
                        drawArrow(originMat4, glm::vec3(0.f), acc_endpoint, 0.1f, glm::vec3(1.f, 1.f, 0.f));
                        //drawTextAtWorldPosition(originMat4, acc_endpoint, "a=%.2fm/s^2", acc);
                    }
                }
            }

            // Draw each HMD model
            for (t_hmd_state_map_iterator hmd_iter = m_hmdViews.begin(); hmd_iter != m_hmdViews.end(); ++hmd_iter)
            {
                const PSVRHeadMountedDisplay *hmdView = hmd_iter->second.hmdView;
                const PSVRTrackingColorType trackingColorType= hmd_iter->second.trackingColorType;

                PSVRPosef hmdPose;
                PSVR_GetHmdPose(hmdView->HmdID, &hmdPose);
                glm::mat4 hmdMat4 = PSVR_posef_to_glm_mat4(hmdPose);

                if (m_hmdViews.size() > 1)
                {
                    drawTextAtWorldPosition(glm::mat4(1.f), PSVR_vector3f_to_glm_vec3(hmdPose.Position), "#%d", hmdView->HmdID);
                }

                drawHMD(hmdView, hmdMat4, trackingColorType);
                drawTransformedAxes(hmdMat4, 10.f);
            }

        } break;
    case eMenuState::showTrackerVideo:
        {
            render_tracker_video();
        } break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;

    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            switch (m_menuState)
            {
            case eMenuState::failedControllerStartRequest:
                ImGui::Text("Failed controller stream start!");
                break;
            case eMenuState::failedHmdStartRequest:
                ImGui::Text("Failed HMD stream start!");
                break;
            case eMenuState::failedTrackerStartRequest:
                ImGui::Text("Failed tracker stream start!");
                break;
            }

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::verifyTrackers:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(500.f, (m_trackerViews.size() > 0) ? 150.f : 100.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Verify that your tracking cameras can see the tracking origin");
            ImGui::Separator();

            if (m_trackerViews.size() > 1)
            {
                ImGui::Text("Tracker #%d", m_renderTrackerIndex);

                if (ImGui::Button("Previous Tracker"))
                {
                    go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    go_next_tracker();
                }
            }

            if (ImGui::Button("Looks Good!"))
            {
                setState(eMenuState::calibrateWithMat);
            }

            if (ImGui::Button("Hmm... Something is wrong."))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }

        // Alignment marker: x
        if (ImGui::IsKeyReleased(120)) m_bShowAlignment = !m_bShowAlignment;
        // Move alignment window up: Up
        if (m_bShowAlignment && ImGui::IsKeyPressed(119))
        {
            m_AlignmentOffset -= (ImGui::GetIO().DisplaySize.y / 2 > -m_AlignmentOffset) 
                ? 1.f 
                : -ImGui::GetIO().DisplaySize.y;
        }
        // Move alignment window down: Down
        if (m_bShowAlignment && ImGui::IsKeyPressed(115))
        {
            m_AlignmentOffset += (ImGui::GetIO().DisplaySize.y / 2 > m_AlignmentOffset) 
                ? 1.f 
                : -ImGui::GetIO().DisplaySize.y;
        }
        // Move alignment window to center: Z
        if (m_bShowAlignment && ImGui::IsKeyPressed(122)) m_AlignmentOffset = 0;

        // Tracker Alignment Marker
        if (m_bShowAlignment)
        {
            float prevAlpha = ImGui::GetStyle().Alpha;
            ImGui::GetStyle().Alpha = 0.f;

            float align_window_size = 30.f;
            float x0 = (ImGui::GetIO().DisplaySize.x - align_window_size) / 2;
            float y0 = (ImGui::GetIO().DisplaySize.y - align_window_size) / 2 + m_AlignmentOffset;

            ImGui::SetNextWindowPos(ImVec2(x0, y0));
            ImGui::SetNextWindowSize(ImVec2(align_window_size, align_window_size));
            ImGui::Begin("Alignment Window", nullptr,
                ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_ShowBorders |
                ImGuiWindowFlags_NoResize |
                ImGuiWindowFlags_NoMove |
                ImGuiWindowFlags_NoScrollbar |
                ImGuiWindowFlags_NoCollapse);

            ImU32 line_colour = ImColor(0x00, 0x00, 0x00, 200);
            float line_thickness = 2.f;

            ImGui::GetWindowDrawList()->AddLine(ImVec2(x0, y0)
                , ImVec2(x0 + align_window_size, y0 + align_window_size)
                , line_colour
                , line_thickness
            );
            ImGui::GetWindowDrawList()->AddLine(
                ImVec2(x0 + align_window_size, y0)
                , ImVec2(x0, y0 + align_window_size)
                , line_colour
                , line_thickness
            );

            ImGui::End();
            ImGui::GetStyle().Alpha = prevAlpha;
        }

        break;

    case eMenuState::selectCalibrationMethod:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - 500.f / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(500.f, 150.f));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Select a calibration method");
            ImGui::Separator();

            if (ImGui::Button("Calibration Mat"))
            {
                setState(eMenuState::calibrateWithMat);
            }

            ImGui::End();
        } break;

    case eMenuState::calibrateWithMat:
        {
            m_pCalibrateWithMat->renderUI();
        } break;

    case eMenuState::testTracking:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(250.f, 260.f));
            ImGui::Begin("Test Tracking Pose", nullptr, window_flags);

            // display per tracker UI
            for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
            {
                const PSVRTracker *trackerView = iter->second.trackerView;

                ImGui::PushItemWidth(125.f);
                if (does_tracker_see_any_device(trackerView))
                {
                    ImGui::Text("Tracker #%d: OK", trackerView->tracker_info.tracker_id);
                }
                else 
                {
                    ImGui::Text("Tracker #%d: FAIL", trackerView->tracker_info.tracker_id);
                }
                ImGui::PopItemWidth();

                ImGui::SameLine();

                ImGui::PushItemWidth(100.f);
                ImGui::PushID(trackerView->tracker_info.tracker_id);
                if (ImGui::Button("Tracker Video") || trackerView->tracker_info.tracker_id == m_ShowTrackerVideoId)
                {
                    m_ShowTrackerVideoId = -1;
                    m_renderTrackerIter = iter;
                    setState(eMenuState::showTrackerVideo);
                }
                ImGui::PopID();
                ImGui::PopItemWidth();
            }

            ImGui::Separator();

            if (!m_bSkipCalibration)
            {
                ImGui::Text("Calibration Complete");

                if (ImGui::Button("Redo Calibration"))
                {
                    setState(eMenuState::verifyTrackers);
                }
            }

            if (ImGui::Button("Tracker Settings"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }
        break;

    case eMenuState::showTrackerVideo:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(200, 130));
            ImGui::Begin("Test Tracking Video", nullptr, window_flags);

            //ImGui::Text("Tracker ID: #%d", m_renderTrackerIter->second.trackerView->tracker_info.tracker_id);

            if (m_trackerViews.size() > 1)
            {
                if (ImGui::Button("<##Previous Tracker"))
                {
                    go_previous_tracker();
                }
                ImGui::SameLine();
                int TrackerID = m_renderTrackerIter->second.trackerView->tracker_info.tracker_id;
                ImGui::Text("Tracker ID: #%d", TrackerID);
                m_app->getAppStage<AppStage_TrackerSettings>()->setSelectedTrackerIndex(TrackerID);
                ImGui::SameLine();
                if (ImGui::Button(">##Next Tracker"))
                {
                    go_next_tracker();
                }
            } 
            else {
                ImGui::Text("Tracker ID: 0");
            }
            
            if (ImGui::Button("Color Calibration"))
            {
                if (m_overrideHmdId != -1)
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoHMDColorCalib(true);
                }
                else
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoControllerColorCalib(true);
                }
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (ImGui::Button("Test Tracking Pose"))
            {
                setState(eMenuState::testTracking);
            }

            if (ImGui::Button("Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }
        break;

    case eMenuState::calibrateStepFailed:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Calibration Failed");

            if (ImGui::Button("Restart Calibration"))
            {
                setState(eMenuState::verifyTrackers);
            }

            if (ImGui::Button("Cancel"))
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        }
        break;

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::setState(eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);

        m_menuState = newState;
    }
}

void AppStage_ComputeTrackerPoses::onExitState(eMenuState newState)
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->exit();
        break;
    case eMenuState::testTracking:
        m_app->setCameraType(_cameraFixed);
        break;
    case eMenuState::showTrackerVideo:
        break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::onEnterState(eMenuState newState)
{
    switch (newState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::failedControllerStartRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::verifyTrackers:
        m_renderTrackerIter = m_trackerViews.begin();
        break;
    case eMenuState::selectCalibrationMethod:
        break;
    case eMenuState::calibrateWithMat:
        m_pCalibrateWithMat->enter();
        break;
    case eMenuState::testTracking:
        {
            for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
            {
                PSVRController *controllerView = controller_iter->second.controllerView;

                switch (controllerView->ControllerType)
                {
                case PSVRController_Move:
                    controllerView->ControllerState.PSMoveState.bPoseResetButtonEnabled= true;
                    break;
                case PSVRController_DualShock4:
                    controllerView->ControllerState.DS4State.bPoseResetButtonEnabled= true;
                    break;
                }
            }

            m_app->setCameraType(_cameraOrbit);
        }
        break;
    case eMenuState::showTrackerVideo:
        break;
    case eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ComputeTrackerPoses::update_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end())
    {
        const int tracker_id= m_renderTrackerIter->second.trackerView->tracker_info.tracker_id;

        const unsigned char *buffer= nullptr;
        if (PSVR_GetTrackerVideoFrameBuffer(tracker_id, PSVRVideoFrameSection_Primary, &buffer) == PSVRResult_Success)
        {
            m_renderTrackerIter->second.textureAsset->copyBufferIntoTexture(buffer);
        }
    }
}

void AppStage_ComputeTrackerPoses::render_tracker_video()
{
    if (m_renderTrackerIter != m_trackerViews.end() &&
        m_renderTrackerIter->second.textureAsset != nullptr)
    {
        drawFullscreenTexture(m_renderTrackerIter->second.textureAsset->texture_id);
    }
}

void AppStage_ComputeTrackerPoses::go_next_tracker()
{
    const int trackerCount = static_cast<int>(m_trackerViews.size());

    if (trackerCount > 1)
    {
        m_renderTrackerIndex = (m_renderTrackerIndex + 1) % trackerCount;

        // Find the tracker iterator that corresponds to the render index we want to show
        for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
        {
            if (iter->second.listIndex == m_renderTrackerIndex)
            {
                m_renderTrackerIter = iter;
            }
        }
    }
}

void AppStage_ComputeTrackerPoses::go_previous_tracker()
{
    const int trackerCount = static_cast<int>(m_trackerViews.size());

    if (trackerCount > 1)
    {
        m_renderTrackerIndex = (m_renderTrackerIndex + trackerCount - 1) % trackerCount;

        // Find the tracker iterator that corresponds to the render index we want to show
        for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
        {
            if (iter->second.listIndex == m_renderTrackerIndex)
            {
                m_renderTrackerIter = iter;
            }
        }
    }
}

int AppStage_ComputeTrackerPoses::get_tracker_count() const
{
    return static_cast<int>(m_trackerViews.size());
}

int AppStage_ComputeTrackerPoses::get_render_tracker_index() const
{
    return m_renderTrackerIndex;
}

PSVRTracker *AppStage_ComputeTrackerPoses::get_render_tracker_view() const
{
    return (m_trackerViews.size() > 0) ? m_renderTrackerIter->second.trackerView : nullptr;
}

PSVRController *AppStage_ComputeTrackerPoses::get_calibration_controller_view() const
{
    return (m_controllerViews.size() > 0) ? m_controllerViews.begin()->second.controllerView : nullptr;
}

PSVRHeadMountedDisplay *AppStage_ComputeTrackerPoses::get_calibration_hmd_view() const
{
    return (m_hmdViews.size() > 0) ? m_hmdViews.begin()->second.hmdView : nullptr;
}

void AppStage_ComputeTrackerPoses::release_devices()
{
    for (t_controller_state_map_iterator iter = m_controllerViews.begin(); iter != m_controllerViews.end(); ++iter)
    {
        ControllerState &controllerState = iter->second;

        if (controllerState.controllerView != nullptr)
        {
            PSVR_StopControllerDataStream(controllerState.controllerView->ControllerID);
            PSVR_FreeControllerListener(controllerState.controllerView->ControllerID);
        }
    }
    m_controllerViews.clear();

    for (t_hmd_state_map_iterator iter = m_hmdViews.begin(); iter != m_hmdViews.end(); ++iter)
    {
        HMDState &hmdState = iter->second;

        if (hmdState.hmdView != nullptr)
        {
            PSVR_StopHmdDataStream(hmdState.hmdView->HmdID);
            PSVR_FreeHmdListener(hmdState.hmdView->HmdID);
        }
    }
    m_hmdViews.clear();

    for (t_tracker_state_map_iterator iter = m_trackerViews.begin(); iter != m_trackerViews.end(); ++iter)
    {
        TrackerState &trackerState = iter->second;

        if (trackerState.textureAsset != nullptr)
        {
            delete trackerState.textureAsset;
        }

        if (trackerState.trackerView != nullptr)
        {
            const int tracker_id= trackerState.trackerView->tracker_info.tracker_id;

            PSVR_CloseTrackerVideoStream(tracker_id);
            PSVR_StopTrackerDataStream(tracker_id);
            PSVR_FreeTrackerListener(tracker_id);
        }
    }
    m_trackerViews.clear();

    m_renderTrackerIndex= 0;
    m_renderTrackerIter = m_trackerViews.end();
}

void AppStage_ComputeTrackerPoses::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_ComputeTrackerPoses::request_controller_list()
{
	bool bStartedAnyControllers = false;

	m_controllerViews.clear();

	// Request a list of controllers back from the server
	PSVRControllerList controller_list;
	if (PSVR_GetControllerList(false, &controller_list) == PSVRResult_Success)
	{
		if (m_overrideControllerId == -1)
		{
			// Start all psmove and dual shock 4 controllers
			for (int list_index = 0; list_index < controller_list.count; ++list_index)
			{
				const PSVRClientControllerInfo &controller= controller_list.controllers[list_index];
				PSVRControllerType controllerType= controller.controller_type;

				if (controllerType == PSVRController_Move ||
					controllerType == PSVRController_DualShock4)
				{
					int trackedControllerId = controller.controller_id;
					const PSVRTrackingColorType trackingColorType= controller.tracking_color_type;

					if (request_start_controller_stream(trackedControllerId, list_index, trackingColorType))
					{
						bStartedAnyControllers = true;
					}
				}
			}

		}
		else
		{
			int trackedControllerId = -1;
			int trackedControllerListIndex = -1;
			PSVRTrackingColorType trackingColorType;

			// Start only the selected controller
			for (int list_index = 0; list_index < controller_list.count; ++list_index)
			{
				const PSVRClientControllerInfo &controller= controller_list.controllers[list_index];

				if (controller.controller_id == m_overrideControllerId)
				{
					trackingColorType = controller.tracking_color_type;
					trackedControllerId = controller.controller_id;
					trackedControllerListIndex = list_index;
					break;
				}
			}

			if (trackedControllerId != -1)
			{
				if (request_start_controller_stream(trackedControllerId, trackedControllerListIndex, trackingColorType))
				{
					bStartedAnyControllers = true;
				}
			}
		}
    }

	if (bStartedAnyControllers)
	{
        if (m_overrideControllerId != -1)
        {
            // If we requested a specific controller to test, 
            // that means we don't care about testing any HMDs
            request_tracker_list();
        }
        else
        {
            // Move on to the HMDs
            request_hmd_list();
        }
	}
	else
	{
		setState(AppStage_ComputeTrackerPoses::failedControllerStartRequest);
	}
}

bool AppStage_ComputeTrackerPoses::request_start_controller_stream(
    PSVRControllerID ControllerID,
    int listIndex,
    PSVRTrackingColorType trackingColorType)
{
    ControllerState controllerState;

    // Allocate a new controller view
    PSVR_AllocateControllerListener(ControllerID);
    controllerState.listIndex = listIndex;
    controllerState.controllerView = PSVR_GetController(ControllerID);
    controllerState.trackingColorType = trackingColorType;

    // Add the controller to the list of controllers we're monitoring
    assert(m_controllerViews.find(ControllerID) == m_controllerViews.end());
    m_controllerViews.insert(t_id_controller_state_pair(ControllerID, controllerState));

    unsigned int flags =
        PSVRStreamFlags_includePositionData |
        PSVRStreamFlags_includeCalibratedSensorData |
        PSVRStreamFlags_includeRawTrackerData |
        PSVRStreamFlags_includePhysicsData;

    // If we are jumping straight to testing, we want the ROI optimization on
    if (!m_bSkipCalibration)
    {
        flags|= PSVRStreamFlags_disableROI;
    }

    // Start off getting getting projection data from tracker 0
    PSVR_SetControllerDataStreamTrackerIndex(controllerState.controllerView->ControllerID, 0);

    // Start receiving data from the controller
	return PSVR_StartControllerDataStream(controllerState.controllerView->ControllerID, flags) == PSVRResult_Success;

}

void AppStage_ComputeTrackerPoses::request_hmd_list()
{
	bool bStartedAnyHMDs = false;

	m_hmdViews.clear();

	// Request a list of controllers back from the server
	PSVRHmdList hmd_list;
	if (PSVR_GetHmdList(&hmd_list) == PSVRResult_Success)
	{
        if (m_overrideHmdId == -1)
        {
            // Start all head mounted displays
            for (int list_index = 0; list_index < hmd_list.count; ++list_index)
            {
				PSVRClientHMDInfo &hmd= hmd_list.hmds[list_index];

                if (hmd.hmd_type == PSVRHmd_Morpheus ||
                    hmd.hmd_type == PSVRHmd_Virtual)
                {
                    int trackedHmdId = hmd.hmd_id;
                    const PSVRTrackingColorType trackingColorType= hmd.tracking_color_type;

					if (request_start_hmd_stream(trackedHmdId, list_index, trackingColorType))
					{
						bStartedAnyHMDs= true;
					}
                }
            }
        }
        else
        {
            int trackedHmdId = -1;
            int trackedHmdListIndex = -1;
            PSVRTrackingColorType trackingColorType;

            // Start only the selected HMD
            for (int list_index = 0; list_index < hmd_list.count; ++list_index)
            {
				PSVRClientHMDInfo &hmd= hmd_list.hmds[list_index];

                if (hmd.hmd_id == m_overrideHmdId)
                {
                    trackingColorType = hmd.tracking_color_type;
                    trackedHmdId = hmd.hmd_id;
                    trackedHmdListIndex = list_index;
                    break;
                }
            }

            if (trackedHmdId != -1)
            {
                bStartedAnyHMDs= request_start_hmd_stream(trackedHmdId, trackedHmdListIndex, trackingColorType);
            }
        }
	}
	else
	{
		
	}

    if (bStartedAnyHMDs)
    {
        request_tracker_list();
    }
	else
	{
		setState(AppStage_ComputeTrackerPoses::failedHmdStartRequest);
	}
}

bool AppStage_ComputeTrackerPoses::request_start_hmd_stream(
    PSVRHmdID HmdID,
    int listIndex,
    PSVRTrackingColorType trackingColorType)
{
    HMDState hmdState;

    // Allocate a new HMD view
    PSVR_AllocateHmdListener(HmdID);
    hmdState.listIndex = listIndex;
    hmdState.hmdView = PSVR_GetHmd(HmdID);
    hmdState.trackingColorType = trackingColorType;

    // Add the hmd to the list of HMDs we're monitoring
    assert(m_hmdViews.find(HmdID) == m_hmdViews.end());
    m_hmdViews.insert(t_id_hmd_state_pair(HmdID, hmdState));

    unsigned int flags =
        PSVRStreamFlags_includePositionData |
        PSVRStreamFlags_includeRawTrackerData;

    // Start off getting getting projection data from tracker 0
    PSVR_SetHmdDataStreamTrackerIndex(hmdState.hmdView->HmdID, 0);

    // Start receiving data from the controller
	return PSVR_StartHmdDataStream(hmdState.hmdView->HmdID, flags) == PSVRResult_Success;
}

void AppStage_ComputeTrackerPoses::request_tracker_list()
{
	bool bStartedAnyTrackers= false;

	m_trackerViews.clear();

    // Tell the psmove service that we we want a list of trackers connected to this machine
	PSVRTrackerList tracker_list;
	if (PSVR_GetTrackerList(&tracker_list) == PSVRResult_Success)
	{
        for (int tracker_index = 0; tracker_index < tracker_list.count; ++tracker_index)
        {
			if (request_tracker_start_stream(&tracker_list.trackers[tracker_index], tracker_index))
			{
				bStartedAnyTrackers= true;
			}
        }
	}

	if (bStartedAnyTrackers)
	{
		handle_all_devices_ready();
	}
	else
	{
		setState(eMenuState::failedTrackerStartRequest);
	}
}

bool AppStage_ComputeTrackerPoses::request_tracker_start_stream(
    const PSVRClientTrackerInfo *TrackerInfo,
    int listIndex)
{
    TrackerState trackerState;
	bool bStartedTracker= false;

    // Allocate a new tracker view
    const int tracker_id= TrackerInfo->tracker_id;
    trackerState.listIndex = listIndex;
    PSVR_AllocateTrackerListener(tracker_id, TrackerInfo);
    trackerState.trackerView = PSVR_GetTracker(tracker_id);
    trackerState.textureAsset = nullptr;

    // Add the tracker to the list of trackers we're monitoring
    assert(m_trackerViews.find(TrackerInfo->tracker_id) == m_trackerViews.end());
    m_trackerViews.insert(t_id_tracker_state_pair(TrackerInfo->tracker_id, trackerState));

    // Request data to start streaming to the tracker
	if (PSVR_StartTrackerDataStream(TrackerInfo->tracker_id) == PSVRResult_Success)
	{
        // Get the tracker state associated with the tracker id
        t_tracker_state_map_iterator trackerStateEntry = m_trackerViews.find(TrackerInfo->tracker_id);
        assert(trackerStateEntry != m_trackerViews.end());

        // The context holds everything a handler needs to evaluate a response
        TrackerState &trackerState = trackerStateEntry->second;
        PSVRClientTrackerInfo &trackerInfo= trackerState.trackerView->tracker_info;

        // Open the shared memory that the video stream is being written to
        if (PSVR_OpenTrackerVideoStream(trackerInfo.tracker_id) == PSVRResult_Success)
        {
            // Create a texture to render the video frame to
            trackerState.textureAsset = new TextureAsset();

            bStartedTracker=
				trackerState.textureAsset->init(
					static_cast<unsigned int>(trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_width),
					static_cast<unsigned int>(trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_height),
					GL_RGB, // texture format
					GL_BGR, // buffer format
					nullptr);
        }
	}

	return bStartedTracker;
}

void AppStage_ComputeTrackerPoses::request_set_tracker_pose(
    const PSVRPosef *pose,
    PSVRTracker *TrackerView)
{
	PSVR_SetTrackerPose(TrackerView->tracker_info.tracker_id, pose);
}

void AppStage_ComputeTrackerPoses::handle_all_devices_ready()
{
    if (!m_bSkipCalibration)
    {
        setState(eMenuState::verifyTrackers);
    }
    else
    {
        setState(eMenuState::testTracking);
    }
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_device(const PSVRTracker *trackerView)
{
    return does_tracker_see_any_controller(trackerView) || does_tracker_see_any_hmd(trackerView);
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_controller(const PSVRTracker *trackerView)
{
    bool bTrackerSeesAnyController = false;
    for (t_controller_state_map_iterator controller_iter = m_controllerViews.begin(); controller_iter != m_controllerViews.end(); ++controller_iter)
    {
        const PSVRController *controllerView = controller_iter->second.controllerView;
        const int tracker_id= trackerView->tracker_info.tracker_id;

        glm::vec3 color;
        if (controllerView->ControllerType == PSVRControllerType::PSVRController_Move &&
            controllerView->ControllerState.PSMoveState.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyController= 
                (controllerView->ControllerState.PSMoveState.RawTrackerData.ValidTrackerBitmask | 
                 (1 << tracker_id)) > 0;
            break;
        }
        else if (controllerView->ControllerType == PSVRControllerType::PSVRController_DualShock4 &&
                 controllerView->ControllerState.DS4State.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyController= 
                (controllerView->ControllerState.DS4State.RawTrackerData.ValidTrackerBitmask | 
                 (1 << tracker_id)) > 0;
            break;
        }
    }

    return bTrackerSeesAnyController;
}

bool AppStage_ComputeTrackerPoses::does_tracker_see_any_hmd(const PSVRTracker *trackerView)
{
    bool bTrackerSeesAnyHmd = false;
    for (t_hmd_state_map_iterator hmd_iter = m_hmdViews.begin(); hmd_iter != m_hmdViews.end(); ++hmd_iter)
    {
        const PSVRHeadMountedDisplay *hmdView = hmd_iter->second.hmdView;
        const int tracker_id= trackerView->tracker_info.tracker_id;

        glm::vec3 color;
        if (hmdView->HmdType == PSVRHmd_Morpheus &&
            hmdView->HmdState.MorpheusState.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyHmd= 
                (hmdView->HmdState.MorpheusState.RawTrackerData.ValidTrackerBitmask | 
                 (1 << tracker_id)) > 0;
            break;
        }
        else if (hmdView->HmdType == PSVRHmd_Virtual &&
                 hmdView->HmdState.VirtualHMDState.bIsCurrentlyTracking)
        {
            bTrackerSeesAnyHmd= 
                (hmdView->HmdState.VirtualHMDState.RawTrackerData.ValidTrackerBitmask | 
                 (1 << tracker_id)) > 0;
            break;
        }
    }

    return bTrackerSeesAnyHmd;
}

//-- private methods -----
static void drawController(
    const PSVRController *controllerView, 
    const glm::mat4 &transform, 
    const PSVRTrackingColorType trackingColorType)
{
    glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

    switch (trackingColorType)
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

    switch(controllerView->ControllerType)
    {
    case PSVRController_Move:
        drawPSMoveModel(transform, bulb_color);
        break;
    case PSVRController_DualShock4:
        drawPSDualShock4Model(transform, bulb_color);
        break;
    }
}

static void drawHMD(
    const PSVRHeadMountedDisplay *hmdView, 
    const glm::mat4 &transform, 
    const PSVRTrackingColorType trackingColorType)
{
    glm::vec3 bulb_color = glm::vec3(1.f, 1.f, 1.f);

    switch (trackingColorType)
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

    switch(hmdView->HmdType)
    {
    case PSVRHmd_Morpheus:
        drawMorpheusModel(transform, bulb_color);
        break;
    case PSVRHmd_Virtual:
        drawVirtualHMDModel(transform, bulb_color);
        break;
    }
}
//-- inludes -----
#include "AppStage_TestTracker.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "AssetManager.h"
#include "App.h"
#include "Camera.h"
#include "ClientLog.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "PSVRServiceInterface.h"
#include "PSVRProtocol.pb.h"
#include "SharedTrackerState.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_TestTracker::APP_STAGE_NAME = "TestTracker";

//-- constants -----

//-- private methods -----

//-- public methods -----
AppStage_TestTracker::AppStage_TestTracker(App *app)
    : AppStage(app)
    , m_menuState(AppStage_TestTracker::inactive)
    , m_bStreamIsActive(false)
    , m_tracker_view(nullptr)
    , m_current_section(PSVRVideoFrameSection_Primary)
    , m_section_count(1)
    , m_video_texture(nullptr)
{ }

void AppStage_TestTracker::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSVRClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    assert(m_tracker_view == nullptr);
	PSVR_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
	m_tracker_view = PSVR_GetTracker(trackerInfo->tracker_id);

    assert(!m_bStreamIsActive);
    request_tracker_start_stream();
}

void AppStage_TestTracker::exit()
{
    m_menuState = AppStage_TestTracker::inactive;

    PSVR_FreeTrackerListener(m_tracker_view->tracker_info.tracker_id);
    m_tracker_view = nullptr;
}

void AppStage_TestTracker::update()
{
    // Try and read the next video frame from shared memory
    if (m_video_texture != nullptr)
    {
        if (PSVR_PollTrackerVideoStream(m_tracker_view->tracker_info.tracker_id) == PSVRResult_Success)
        {
			const unsigned char *buffer= nullptr;
			if (PSVR_GetTrackerVideoFrameBuffer(
                    m_tracker_view->tracker_info.tracker_id, m_current_section, &buffer) == PSVRResult_Success)
			{
				m_video_texture->copyBufferIntoTexture(buffer);
			}
        }
    }
}

void AppStage_TestTracker::render()
{
    // If there is a video frame available to render, show it
    if (m_video_texture != nullptr)
    {
        unsigned int texture_id = m_video_texture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_TestTracker::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Video Feed Test";
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
        const int panel_height= (m_section_count > 1) ? 80 : 50;

        ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, panel_height));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_section_count > 1)
        {
            if (m_current_section == PSVRVideoFrameSection_Left)
            {
                if (ImGui::Button("Show Right Frame"))
                {
                    m_current_section= PSVRVideoFrameSection_Right;
                }
            }
            else if (m_current_section == PSVRVideoFrameSection_Right)
            {
                if (ImGui::Button("Show Left Frame"))
                {
                    m_current_section= PSVRVideoFrameSection_Left;
                }
            }
        }

        if (ImGui::Button("Return to Tracker Settings"))
        {
            if (m_bStreamIsActive)
            {
                const AppStage_TrackerSettings *trackerSettings =
                    m_app->getAppStage<AppStage_TrackerSettings>();
                const PSVRClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();

                request_tracker_stop_stream();
            }
            else
            {
                m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }
        }              

        ImGui::End();
    } break;

    case eTrackerMenuState::pendingTrackerStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to start...");

        ImGui::End();
    } break;

    case eTrackerMenuState::failedTrackerStartStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to start tracker stream!");

        if (ImGui::Button("Ok"))
        {
            m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        }

        if (ImGui::Button("Return to Main Menu"))
        {
            m_app->setAppStage(AppStage_MainMenu::APP_STAGE_NAME);
        }

        ImGui::End();
    } break;

    case eTrackerMenuState::pendingTrackerStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for tracker stream to stop...");

        ImGui::End();
    } break;

    case eTrackerMenuState::failedTrackerStopStreamRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Failed to stop tracker stream!");

        if (ImGui::Button("Ok"))
        {
            m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
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

void AppStage_TestTracker::request_tracker_start_stream()
{
    if (m_menuState != AppStage_TestTracker::pendingTrackerStartStreamRequest)
    {
        m_menuState = AppStage_TestTracker::pendingTrackerStartStreamRequest;

        // Tell the PSVR service that we want to start streaming data from the tracker
		PSVRRequestID requestID;
		PSVR_StartTrackerDataStreamAsync(
			m_tracker_view->tracker_info.tracker_id, 
			&requestID);
		PSVR_RegisterCallback(requestID, AppStage_TestTracker::handle_tracker_start_stream_response, this);
    }
}

void AppStage_TestTracker::handle_tracker_start_stream_response(
    const PSVRResponseMessage *response,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    switch (response->result_code)
    {
    case PSVRResult_Success:
        {
            PSVRTracker *trackerView= thisPtr->m_tracker_view;

            thisPtr->m_bStreamIsActive = true;
            thisPtr->m_menuState = AppStage_TestTracker::idle;

            // Open the shared memory that the vidoe stream is being written to
            if (PSVR_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSVRResult_Success &&
                PSVR_GetTrackerVideoFrameSectionCount(trackerView->tracker_info.tracker_id, &thisPtr->m_section_count) == PSVRResult_Success)
            {
                PSVRVector2f screenSize;
                PSVR_GetTrackerScreenSize(trackerView->tracker_info.tracker_id, &screenSize);
                const unsigned int frameWidth = static_cast<unsigned int>(screenSize.x);
                const unsigned int frameHeight = static_cast<unsigned int>(screenSize.y);

                // Create a texture to render the video frame to
                thisPtr->m_video_texture = new TextureAsset();
                thisPtr->m_video_texture->init(
                    frameWidth,
                    frameHeight,
                    GL_RGB, // texture format
                    GL_BGR, // buffer format
                    nullptr);
            }
        } break;

    case PSVRResult_Error:
    case PSVRResult_Canceled:
	case PSVRResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStartStreamRequest;
        } break;
    }
}

void AppStage_TestTracker::request_tracker_stop_stream()
{
    if (m_bStreamIsActive && m_menuState != AppStage_TestTracker::pendingTrackerStopStreamRequest)
    {
        m_menuState = AppStage_TestTracker::pendingTrackerStopStreamRequest;

        // Tell the PSVR service that we want to stop streaming data from the tracker
		PSVRRequestID requestId;
		PSVR_StopTrackerDataStreamAsync(m_tracker_view->tracker_info.tracker_id, &requestId);
		PSVR_RegisterCallback(requestId, AppStage_TestTracker::handle_tracker_stop_stream_response, this);
    }
}

void AppStage_TestTracker::handle_tracker_stop_stream_response(
    const PSVRResponseMessage *response,
    void *userdata)
{
    AppStage_TestTracker *thisPtr = static_cast<AppStage_TestTracker *>(userdata);

    // In either case consider the stream as now inactive
    thisPtr->m_bStreamIsActive = false;

    switch (response->result_code)
    {
    case PSVRResult_Success:
        {
            thisPtr->m_menuState = AppStage_TestTracker::inactive;

            // Close the shared memory buffer
			PSVR_CloseTrackerVideoStream(thisPtr->m_tracker_view->tracker_info.tracker_id);

            // Free the texture we were rendering to
            if (thisPtr->m_video_texture != nullptr)
            {
                delete thisPtr->m_video_texture;
                thisPtr->m_video_texture = nullptr;
            }

            // After closing the stream, we should go back to the tracker settings
            thisPtr->m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
        } break;

    case PSVRResult_Error:
    case PSVRResult_Canceled:
	case PSVRResult_Timeout:
        {
            thisPtr->m_menuState = AppStage_TestTracker::failedTrackerStopStreamRequest;
        } break;
    }
}

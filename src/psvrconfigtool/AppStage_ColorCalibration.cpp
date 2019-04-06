//-- inludes -----
#include "AppStage_ColorCalibration.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "AssetManager.h"
#include "App.h"
#include "Camera.h"
#include "Logger.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <thread>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_ColorCalibration::APP_STAGE_NAME = "ColorCalibration";

//-- constants -----
static const char *k_video_display_mode_names[] = {
    "BGR",
    "HSV",
    "Masked"
};

static const char *k_tracking_color_names[] = {
    "Magenta",
    "Cyan",
    "Yellow",
    "Red",
    "Green",
    "Blue"
};

//-- private definitions -----
class VideoBufferState
{
public:
    VideoBufferState(PSVRTracker *trackerView)
        : videoTexture(nullptr)
        , bgrBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
        , maskedBuffer(nullptr)
    {
        PSVRVector2f screenSize;
        PSVR_GetTrackerScreenSize(trackerView->tracker_info.tracker_id, &screenSize);
        const int frameWidth = static_cast<int>(screenSize.x);
        const int frameHeight = static_cast<int>(screenSize.y);

        // Create a texture to render the video frame to
        videoTexture = new TextureAsset();
        videoTexture->init(
            frameWidth,
            frameHeight,
            GL_RGB, // texture format
            GL_BGR, // buffer format
            nullptr);

        cpuScratchBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        bgrBuffer = new cv::UMat(frameHeight, frameWidth, CV_8UC3);
        hsvBuffer = new cv::UMat(frameHeight, frameWidth, CV_8UC3);
        gsLowerBuffer = new cv::UMat(frameHeight, frameWidth, CV_8UC1);
        gsUpperBuffer = new cv::UMat(frameHeight, frameWidth, CV_8UC1);
        maskedBuffer = new cv::UMat(frameHeight, frameWidth, CV_8UC3);
    }

    virtual ~VideoBufferState()
    {
        if (cpuScratchBuffer != nullptr)
        {
            delete cpuScratchBuffer;
            cpuScratchBuffer = nullptr;
        }

        if (maskedBuffer != nullptr)
        {
            delete maskedBuffer;
            maskedBuffer = nullptr;
        }

        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
            gsLowerBuffer = nullptr;
        }

        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
            gsUpperBuffer = nullptr;
        }

        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
            hsvBuffer = nullptr;
        }

        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
            bgrBuffer = nullptr;
        }

        if (videoTexture != nullptr)
        {
            delete videoTexture;
            videoTexture = nullptr;
        }
    }

    TextureAsset *videoTexture;
    cv::Mat *cpuScratchBuffer; // source video frame
    cv::UMat *bgrBuffer; // source video frame
    cv::UMat *hsvBuffer; // source frame converted to HSV color space
    cv::UMat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::UMat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::UMat *maskedBuffer; // bgr image ANDed together with grayscale mask
};

//-- public methods -----
AppStage_ColorCalibration::AppStage_ColorCalibration(App *app)
    : AppStage(app)
    , m_overrideControllerId(-1)
    , m_masterControllerView(nullptr)
    , m_areAllControllerStreamsActive(false)
    , m_lastMasterControllerSeqNum(-1)
    , m_overrideHmdId(-1)
    , m_hmdView(nullptr)
    , m_isHmdStreamActive(false)
    , m_lastHmdSeqNum(-1)
    , m_trackerView(nullptr)
    , m_menuState(AppStage_ColorCalibration::inactive)
    , m_video_buffer_state(nullptr)
    , m_videoDisplayMode(AppStage_ColorCalibration::eVideoDisplayMode::mode_bgr)
	, m_trackerModeIndex(0)
    , m_bTurnOnAllControllers(false)
    , m_bAutoChangeController(false)
    , m_bAutoChangeColor(false)
    , m_bAutoChangeTracker(false)
    , m_bAutoCalibrate(false)
    , m_bShowWindows(true)
    , m_bShowAlignment(false)
    , m_bShowAlignmentColor(false)
    , m_AlignmentOffset(0.f)
    , m_masterTrackingColorType(PSVRTrackingColorType_Magenta)
{ 
    memset(&m_colorPresetTable, 0, sizeof(m_colorPresetTable));
	memset(m_videoProperties, 0, sizeof(m_videoProperties));
}

void AppStage_ColorCalibration::enter()
{
    m_app->getAppStage<AppStage_TrackerSettings>()->gotoControllerColorCalib(false);
    m_app->getAppStage<AppStage_TrackerSettings>()->gotoHMDColorCalib(false);
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSVRClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTracker();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    tracker_count = trackerSettings->getTrackerCount();
    tracker_index = trackerSettings->getSelectedTrackerIndex();

	// Get the current tracker mode
	char current_mode_name[MAX_PSVR_TRACKER_MODE_NAME_LENGTH];
	PSVR_GetTrackerMode(trackerInfo->tracker_id, current_mode_name, sizeof(current_mode_name));

	// Extract the list of available tracker modes
	// and determine what the current mode index is
	m_trackerModeIndex= -1;
	for (int mode_index = 0; mode_index < trackerInfo->mode_count; ++mode_index)
	{
		if (strncmp(current_mode_name, trackerInfo->mode_list[mode_index], MAX_PSVR_TRACKER_MODE_NAME_LENGTH) == 0)
		{
			m_trackerModeIndex= mode_index;
			break;
		}
	}

    // Use the tracker selected from the tracker settings menu
    assert(m_trackerView == nullptr);
    PSVR_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
    m_trackerView = PSVR_GetTracker(trackerInfo->tracker_id);

    if (m_overrideHmdId != -1)
    {
        assert(m_hmdView == nullptr);
        PSVR_AllocateHmdListener(m_overrideHmdId);
        m_hmdView = PSVR_GetHmd(m_overrideHmdId);
        m_isHmdStreamActive = false;
        m_lastHmdSeqNum = -1;
    }
    else
    {
        // Assume that we can bind to controller 0 if no controller override is given
        const int masterControllerID = (m_overrideControllerId != -1) ? m_overrideControllerId : 0;

        m_controllerViews.clear();
        m_controllerTrackingColorTypes.clear();

        for (int list_index = 0; list_index < trackerSettings->getControllerCount(); ++list_index)
        {
            const PSVRClientControllerInfo *controller_info= trackerSettings->getControllerInfo(list_index);
            PSVR_AllocateControllerListener(controller_info->controller_id);
            PSVRController *controllerView= PSVR_GetController(controller_info->controller_id);

            if (masterControllerID == controller_info->controller_id)
            {
                assert(m_masterControllerView == nullptr);
                m_masterControllerView= controllerView;
            }

            m_controllerViews.push_back(controllerView);
            m_controllerTrackingColorTypes.push_back(controller_info->tracking_color_type);
        }
        
        m_areAllControllerStreamsActive = false;
        m_lastMasterControllerSeqNum = -1;
        m_bTurnOnAllControllers= false;

        m_bAutoChangeController = (m_bAutoChangeController) ? m_bAutoChangeController : false;
    }

    m_bAutoChangeColor = (m_bAutoChangeColor) ? m_bAutoChangeColor : false;
    m_bAutoChangeTracker = (m_bAutoChangeTracker) ? m_bAutoChangeTracker : false;

    // Request to start the tracker
    // Wait for the tracker response before requesting the controller
    assert(m_video_buffer_state == nullptr);
    request_tracker_start_stream();

    // In parallel, Get the settings for the selected tracker
    request_tracker_get_settings();
}

void AppStage_ColorCalibration::exit()
{
    setState(AppStage_ColorCalibration::inactive);

    release_devices();
}

void AppStage_ColorCalibration::update()
{
    bool bControllerDataUpdatedThisFrame= false;

    if (m_menuState == eMenuState::waitingForStreamStartResponse)
    {
        if (m_areAllControllerStreamsActive && m_masterControllerView->OutputSequenceNum != m_lastMasterControllerSeqNum)
        {
            request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            setState(eMenuState::manualConfig);
        }
        else if (m_isHmdStreamActive && m_hmdView->OutputSequenceNum != m_lastHmdSeqNum)
        {
            setState(eMenuState::manualConfig);
        }
    }

    // Try and read the next video frame from shared memory
    if (m_video_buffer_state != nullptr)
    {
        const unsigned char *video_buffer= nullptr;
        if (PSVR_GetTrackerVideoFrameBuffer(m_trackerView->tracker_info.tracker_id, PSVRVideoFrameSection_Primary, &video_buffer) == PSVRResult_Success)
        {
            PSVRVector2f screenSize;
            PSVR_GetTrackerScreenSize(m_trackerView->tracker_info.tracker_id, &screenSize);
            const unsigned int frameWidth = static_cast<unsigned int>(screenSize.x);
            const unsigned int frameHeight = static_cast<unsigned int>(screenSize.y);
            const unsigned char *display_buffer = video_buffer;
            const PSVR_HSVColorRange &preset = getColorPreset();

            // Copy the video frame buffer into the bgr opencv buffer
            {
                const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

                videoBufferMat.copyTo(*m_video_buffer_state->bgrBuffer);
            }

            // Convert the video buffer to the HSV color space
            cv::cvtColor(*m_video_buffer_state->bgrBuffer, *m_video_buffer_state->hsvBuffer, cv::COLOR_BGR2HSV);

            // Clamp the HSV image, taking into account wrapping the hue angle
            {
                const float hue_min = preset.hue_range.center - preset.hue_range.range;
                const float hue_max = preset.hue_range.center + preset.hue_range.range;
                const float saturation_min = clampf(preset.saturation_range.center - preset.saturation_range.range, 0, 255);
                const float saturation_max = clampf(preset.saturation_range.center + preset.saturation_range.range, 0, 255);
                const float value_min = clampf(preset.value_range.center - preset.value_range.range, 0, 255);
                const float value_max = clampf(preset.value_range.center + preset.value_range.range, 0, 255);

                if (hue_min < 0)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(180 + hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else if (hue_max > 180)
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(0, saturation_min, value_min),
                        cv::Scalar(clampf(hue_max - 180, 0, 180), saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(clampf(hue_min, 0, 180), saturation_min, value_min),
                        cv::Scalar(180, saturation_max, value_max),
                        *m_video_buffer_state->gsUpperBuffer);
                    cv::bitwise_or(
                        *m_video_buffer_state->gsLowerBuffer, 
                        *m_video_buffer_state->gsUpperBuffer, 
                        *m_video_buffer_state->gsLowerBuffer);
                }
                else
                {
                    cv::inRange(
                        *m_video_buffer_state->hsvBuffer,
                        cv::Scalar(hue_min, saturation_min, value_min),
                        cv::Scalar(hue_max, saturation_max, value_max),
                        *m_video_buffer_state->gsLowerBuffer);
                }
            }

            // Mask out the original video frame with the HSV filtered mask
            *m_video_buffer_state->maskedBuffer = cv::Scalar(0, 0, 0);
            cv::bitwise_and(
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->bgrBuffer, 
                *m_video_buffer_state->maskedBuffer, 
                *m_video_buffer_state->gsLowerBuffer);

            switch (m_videoDisplayMode)
            {
            case AppStage_ColorCalibration::mode_bgr:
                m_video_buffer_state->bgrBuffer->copyTo(*m_video_buffer_state->cpuScratchBuffer);
                break;
            case AppStage_ColorCalibration::mode_hsv:
                m_video_buffer_state->hsvBuffer->copyTo(*m_video_buffer_state->cpuScratchBuffer);
                break;
            case AppStage_ColorCalibration::mode_masked:
                m_video_buffer_state->maskedBuffer->copyTo(*m_video_buffer_state->cpuScratchBuffer);
                break;
            default:
                assert(0 && "unreachable");
                break;
            }

            // Display the selected buffer
            display_buffer = m_video_buffer_state->cpuScratchBuffer->data;
            m_video_buffer_state->videoTexture->copyBufferIntoTexture(display_buffer);
        }
    }
}

void AppStage_ColorCalibration::render()
{
    // If there is a video frame available to render, show it
    if (m_video_buffer_state != nullptr)
    {
        unsigned int texture_id = m_video_buffer_state->videoTexture->texture_id;

        if (texture_id != 0)
        {
            drawFullscreenTexture(texture_id);
        }
    }
}

void AppStage_ColorCalibration::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Color Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    int auto_calib_sleep = 150;

    const AppStage_TrackerSettings *trackerSettings = m_app->getAppStage<AppStage_TrackerSettings>();
    const PSVRClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTracker();

    switch (m_menuState)
    {
    case eMenuState::manualConfig:
    {
        // Video Control Panel
        if (m_bShowWindows)
        {
            ImGui::SetNextWindowPos(ImVec2(10.f, 10.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 280));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (ImGui::Button("Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }
            
            if (ImGui::Button("Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            if (m_video_buffer_state != nullptr)
            {
                if (ImGui::Button("<##Filter"))
                {
                    m_videoDisplayMode =
                        static_cast<eVideoDisplayMode>(
                        (m_videoDisplayMode + eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES - 1)
                        % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
                }
                ImGui::SameLine();
                if (ImGui::Button(">##Filter"))
                {
                    m_videoDisplayMode =
                        static_cast<eVideoDisplayMode>(
                        (m_videoDisplayMode + 1) % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
                }
                ImGui::SameLine();
                ImGui::Text("Video [F]ilter Mode: %s", k_video_display_mode_names[m_videoDisplayMode]);

				if (trackerInfo->mode_count > 0)
				{
					if (ImGui::Button("-##TrackerMode"))
					{
						int new_mode_index = (m_trackerModeIndex + 1) % trackerInfo->mode_count;

						if (request_tracker_set_mode(trackerInfo->mode_list[new_mode_index]))
						{
							m_trackerModeIndex= (int)new_mode_index;
						}
					}
					ImGui::SameLine();
					if (ImGui::Button("+##TrackerMode"))
					{
						size_t new_mode_index= (m_trackerModeIndex + trackerInfo->mode_count - 1) % trackerInfo->mode_count;

						if (request_tracker_set_mode(trackerInfo->mode_list[new_mode_index]))
						{
							m_trackerModeIndex= (int)new_mode_index;
						}
					}
				}
				ImGui::SameLine();
                ImGui::Text("Tracker Mode: %s", trackerInfo->mode_list[m_trackerModeIndex]);

				if (canDecVideoProperty(PSVRVideoProperty_Exposure))
				{
					if (ImGui::Button("-##Exposure"))
					{
						request_tracker_set_video_property(
							PSVRVideoProperty_Exposure, 
							m_videoProperties[PSVRVideoProperty_Exposure] - getVideoPropertyStepSize(PSVRVideoProperty_Exposure));
					}
					ImGui::SameLine();
				}
				if (canIncVideoProperty(PSVRVideoProperty_Exposure))
				{
					if (ImGui::Button("+##Exposure"))
					{
						request_tracker_set_video_property(
							PSVRVideoProperty_Exposure, 
							m_videoProperties[PSVRVideoProperty_Exposure] + getVideoPropertyStepSize(PSVRVideoProperty_Exposure));
					}
					ImGui::SameLine();
				}
                ImGui::Text("Exposure: %d", m_videoProperties[PSVRVideoProperty_Exposure]);

				if (canDecVideoProperty(PSVRVideoProperty_Gain))
				{
					if (ImGui::Button("-##Gain"))
					{
						request_tracker_set_video_property(
							PSVRVideoProperty_Gain, 
							m_videoProperties[PSVRVideoProperty_Gain] - getVideoPropertyStepSize(PSVRVideoProperty_Gain));
					}
					ImGui::SameLine();
				}
				if (canIncVideoProperty(PSVRVideoProperty_Gain))
				{
					if (ImGui::Button("+##Gain"))
					{
						request_tracker_set_video_property(
							PSVRVideoProperty_Gain, 
							m_videoProperties[PSVRVideoProperty_Gain] + getVideoPropertyStepSize(PSVRVideoProperty_Gain));
					}
					ImGui::SameLine();
				}
                ImGui::Text("Gain: %d", m_videoProperties[PSVRVideoProperty_Gain]);
				
                if (m_masterControllerView != nullptr)
                {
                    if (ImGui::Checkbox("Turn on all bulbs", &m_bTurnOnAllControllers))
                    {
                        request_turn_on_all_tracking_bulbs(m_bTurnOnAllControllers);
                    }
                }				
            }

            ImGui::End();
        }
        
        if (ImGui::IsMouseClicked(1) || m_bAutoCalibrate)
        {
            m_bAutoCalibrate = false;
            float x0 = ImGui::GetIO().DisplaySize.x / 2;
            float y0 = ImGui::GetIO().DisplaySize.y / 2 + m_AlignmentOffset;
            ImVec2 mousePos = (m_bShowAlignment) ? ImVec2(x0, y0) : ImGui::GetMousePos();
            ImVec2 dispSize = ImGui::GetIO().DisplaySize;
            int img_x = (static_cast<int>(mousePos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
            int img_y = (static_cast<int>(mousePos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);

            m_video_buffer_state->hsvBuffer->copyTo(*m_video_buffer_state->cpuScratchBuffer);
            cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->cpuScratchBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

            PSVR_HSVColorRange preset = getColorPreset();
            preset.hue_range.center = hsv_pixel[0];
            preset.saturation_range.center = hsv_pixel[1];
            preset.value_range.center = hsv_pixel[2];
            request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            if (m_masterControllerView != nullptr)
            {
                if (m_bAutoChangeColor) {
                    setState(eMenuState::blank1);
                    request_set_controller_tracking_color(m_masterControllerView, PSVRTrackingColorType_Magenta);
                    m_masterTrackingColorType = PSVRTrackingColorType_Magenta;
                    std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
                }
                else if ( m_bAutoChangeController && !m_bShowAlignment) {
                    setState(eMenuState::changeController);
                }
                else if (m_bAutoChangeTracker) {
                    setState(eMenuState::changeTracker);
                }
            }
            else if (m_hmdView != nullptr)
            {
                if (m_bAutoChangeTracker) {
                    setState(eMenuState::changeTracker);
                }
            }
        }

        // Keyboard shortcuts
        {
            // Hide setting windows: space bar
            if (ImGui::IsKeyReleased(32)) m_bShowWindows = !m_bShowWindows;
            // Hide alignment windows: x
            if (ImGui::IsKeyReleased(120)) 
            {
                if (!m_bShowAlignment) {
                    m_bShowAlignment = true;
                    m_bShowAlignmentColor = false;
                }
                else if (!m_bShowAlignmentColor) {
                    m_bShowAlignmentColor = true;
                }
                else {
                    m_bShowAlignment = false;
                    m_bShowAlignmentColor = false;
                }
            }
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
            if (m_bShowAlignment && ImGui::IsKeyPressed(122))
            {
                m_AlignmentOffset = 0;
            }
            // Change filter: F
            if (ImGui::IsKeyReleased(102)) {
                m_videoDisplayMode =
                    static_cast<eVideoDisplayMode>(
                    (m_videoDisplayMode + 1) % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
            }
            // Change tracker: T
            if (ImGui::IsKeyReleased(116)) {
                request_change_tracker(1);
            }
            if (m_masterControllerView != nullptr)
            {
                // Change controller: M
                if (ImGui::IsKeyReleased(109)) {
                    request_change_controller(1);
                }
                // Change color: C
                if (ImGui::IsKeyReleased(99)) {
                    PSVRTrackingColorType new_color =
                        static_cast<PSVRTrackingColorType>(
                        (m_masterTrackingColorType + 1) % PSVRTrackingColorType_MaxColorTypes);
                    request_set_controller_tracking_color(m_masterControllerView, new_color);
                    m_masterTrackingColorType = new_color;
                }
            }
        }

        // Color Control Panel
        if (m_bShowWindows)
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x - k_panel_width - 10, 10.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 300));
            ImGui::Begin("Color Filter", nullptr, window_flags);

            if (m_masterControllerView != nullptr)
            {
                if (ImGui::Button("<##Color"))
                {
                    PSVRTrackingColorType new_color =
                        static_cast<PSVRTrackingColorType>(
                            (m_masterTrackingColorType + PSVRTrackingColorType_MaxColorTypes - 1)
                            % PSVRTrackingColorType_MaxColorTypes);
                    request_set_controller_tracking_color(m_masterControllerView, new_color);
                    m_masterTrackingColorType= new_color;
                }
                ImGui::SameLine();
                if (ImGui::Button(">##Color"))
                {
                    PSVRTrackingColorType new_color =
                        static_cast<PSVRTrackingColorType>(
                            (m_masterTrackingColorType + 1) % PSVRTrackingColorType_MaxColorTypes);
                    request_set_controller_tracking_color(m_masterControllerView, new_color);
                    m_masterTrackingColorType= new_color;
                }
                ImGui::SameLine();
            }
            ImGui::Text("Tracking [C]olor: %s", k_tracking_color_names[m_masterTrackingColorType]);

            // -- Hue --
            if (ImGui::Button("-##HueCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.hue_range.center = wrap_range(preset.hue_range.center - 5.f, 0.f, 180.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##HueCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.hue_range.center = wrap_range(preset.hue_range.center + 5.f, 0.f, 180.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Hue Angle: %f", getColorPreset().hue_range.center);

            if (ImGui::Button("-##HueRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.hue_range.range = clampf(preset.hue_range.range - 5.f, 0.f, 90.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##HueRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.hue_range.range = clampf(preset.hue_range.range + 5.f, 0.f, 90.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Hue Range: %f", getColorPreset().hue_range.range);

            // -- Saturation --
            if (ImGui::Button("-##SaturationCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.saturation_range.center = clampf(preset.saturation_range.center - 5.f, 0.f, 255.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##SaturationCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.saturation_range.center = clampf(preset.saturation_range.center + 5.f, 0.f, 255.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Saturation Center: %f", getColorPreset().saturation_range.center);

            if (ImGui::Button("-##SaturationRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.saturation_range.range = clampf(preset.saturation_range.range - 5.f, 0.f, 125.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##SaturationRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.saturation_range.range = clampf(preset.saturation_range.range + 5.f, 0.f, 125.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Saturation Range: %f", getColorPreset().saturation_range.range);

            // -- Value --
            if (ImGui::Button("-##ValueCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.value_range.center = clampf(preset.value_range.center - 5.f, 0.f, 255.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##ValueCenter"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.value_range.center = clampf(preset.value_range.center + 5.f, 0.f, 255.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Value Center: %f", getColorPreset().value_range.center);

            if (ImGui::Button("-##ValueRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.value_range.range = clampf(preset.value_range.range - 5.f, 0.f, 125.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            if (ImGui::Button("+##ValueRange"))
            {
                PSVR_HSVColorRange preset = getColorPreset();
                preset.value_range.range = clampf(preset.value_range.range + 5.f, 0.f, 125.f);
                request_tracker_set_color_filter(m_masterTrackingColorType, preset);
            }
            ImGui::SameLine();
            ImGui::Text("Value Range: %f", getColorPreset().value_range.range);

            // -- Auto Calibration --
            ImGui::Text("Auto Change Settings:");
            if (m_masterControllerView != nullptr)
            {
                ImGui::Checkbox("Color", &m_bAutoChangeColor);
                ImGui::SameLine();
                ImGui::Checkbox("Controller", &m_bAutoChangeController);
                ImGui::SameLine();
            }
            ImGui::Checkbox("Tracker", &m_bAutoChangeTracker);

            // -- Change Controller --
            if (m_masterControllerView != nullptr)
            {
                if (ImGui::Button("<##Controller"))
                {
                    request_change_controller(-1);
                }
                ImGui::SameLine();
                if (ImGui::Button(">##Controller"))
                {
                    request_change_controller(1);
                }
                ImGui::SameLine();
                ImGui::Text("PS[M]ove Controller ID: %d", m_overrideControllerId);
            }

            // -- Change Tracker --
            if (ImGui::Button("<##Tracker"))
            {
                request_change_tracker(-1);
            }
            ImGui::SameLine();
            if (ImGui::Button(">##Tracker"))
            {
                request_change_tracker(1);
            }
            ImGui::SameLine();
            ImGui::Text("[T]racker ID: %d", tracker_index);

            if (m_masterControllerView != nullptr)
            {
                if (ImGui::Button("Test Tracking"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestControllerTracking(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
                ImGui::SameLine();
                if (ImGui::Button("One"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingControllerVideo(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
                ImGui::SameLine();
                if (ImGui::Button("ALL"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingVideoALL(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
            }
            else if (m_hmdView != nullptr)
            {
                if (ImGui::Button("Test Tracking"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTestHMDTracking(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
                ImGui::SameLine();
                if (ImGui::Button("One"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingHMDVideo(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
                ImGui::SameLine();
                if (ImGui::Button("ALL"))
                {
                    m_app->getAppStage<AppStage_TrackerSettings>()->gotoTrackingVideoALL(true);
                    request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
                }
            }
            
            ImGui::End();
        }
        
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

            unsigned char r, g, b;
            if (m_bShowAlignmentColor)
            {
                switch (m_masterTrackingColorType)
                {
                case PSVRTrackingColorType_Magenta:
                    r = 0xFF; g = 0x00; b = 0xFF;
                    break;
                case PSVRTrackingColorType_Cyan:
                    r = 0x00; g = 0xFF; b = 0xFF;
                    break;
                case PSVRTrackingColorType_Yellow:
                    r = 0xFF; g = 0xFF; b = 0x00;
                    break;
                case PSVRTrackingColorType_Red:
                    r = 0xFF; g = 0x00; b = 0x00;
                    break;
                case PSVRTrackingColorType_Green:
                    r = 0x00; g = 0xFF; b = 0x00;
                    break;
                case PSVRTrackingColorType_Blue:
                    r = 0x00; g = 0x00; b = 0xFF;
                    break;
                default:
                    r = 0x00; g = 0x00; b = 0x00;
                }
            }
            else {
                r = 0xFF; g = 0xFF; b = 0xFF;
            }
            ImU32 line_colour = ImColor(0xFF - r, 0xFF - g, 0xFF - b, 175);
            float line_thickness = 1.f;

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

    } break;

    case eMenuState::autoConfig:
    {
        PSVRTrackingColorType new_color =
            static_cast<PSVRTrackingColorType>(
            (m_masterTrackingColorType + 1) % PSVRTrackingColorType_MaxColorTypes);

        float x0 = ImGui::GetIO().DisplaySize.x / 2;
        float y0 = ImGui::GetIO().DisplaySize.y / 2 + m_AlignmentOffset;
        ImVec2 mousePos = (m_bShowAlignment) ? ImVec2(x0, y0) : ImGui::GetMousePos();
        ImVec2 dispSize = ImGui::GetIO().DisplaySize;
        int img_x = (static_cast<int>(mousePos.x) * m_video_buffer_state->hsvBuffer->cols) / static_cast<int>(dispSize.x);
        int img_y = (static_cast<int>(mousePos.y) * m_video_buffer_state->hsvBuffer->rows) / static_cast<int>(dispSize.y);

        m_video_buffer_state->hsvBuffer->copyTo(*m_video_buffer_state->cpuScratchBuffer);
        cv::Vec< unsigned char, 3 > hsv_pixel = m_video_buffer_state->cpuScratchBuffer->at<cv::Vec< unsigned char, 3 >>(cv::Point(img_x, img_y));

        request_set_controller_tracking_color(m_masterControllerView, new_color);

        if (new_color == PSVRTrackingColorType_Magenta) {
            if (m_bAutoChangeController && !m_bShowAlignment) setState(eMenuState::changeController);
            else if (m_bAutoChangeTracker) setState(eMenuState::changeTracker);
            else setState(eMenuState::manualConfig);
        }
        else setState(eMenuState::blank1);

        m_masterTrackingColorType = new_color;

        std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
    } break;

    case eMenuState::blank1:
        setState(eMenuState::blank2);
        std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
        break;
    case eMenuState::blank2:
        setState(eMenuState::autoConfig);
        std::this_thread::sleep_for(std::chrono::milliseconds(auto_calib_sleep));
        break;
    case eMenuState::changeController:
    {
        setState(eMenuState::manualConfig);
        request_change_controller(1);
    }
        break;
    case eMenuState::changeTracker:
        setState(eMenuState::manualConfig);
        request_change_tracker(1);
        break;
    case eMenuState::pendingTrackerStartStreamRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::waitingForStreamStartResponse:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        ImGui::Text("Waiting for device stream to start...");

        ImGui::End();
    } break;

    case eMenuState::failedTrackerStartStreamRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedControllerStartRequest:
    {
        ImGui::SetNextWindowPosCenter();
        ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
        ImGui::Begin(k_window_title, nullptr, window_flags);

        if (m_menuState == eMenuState::failedTrackerStartStreamRequest)
        {
            ImGui::Text("Failed to start tracker stream!");
        }
        else if (m_menuState == eMenuState::failedHmdStartRequest)
        {
            ImGui::Text("Failed to start controller stream!");
        }
        else
        {
            ImGui::Text("Failed to start controller stream!");
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

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_ColorCalibration::setState(
    AppStage_ColorCalibration::eMenuState newState)
{
    if (newState != m_menuState)
    {
        m_menuState = newState;
    }
}

void AppStage_ColorCalibration::request_start_controller_streams()
{
	m_areAllControllerStreamsActive= true;

    for (PSVRController *controllerView : m_controllerViews)
    {
		if (PSVR_StartControllerDataStream(
				controllerView->ControllerID,
				PSVRStreamFlags_defaultStreamOptions |
				PSVRStreamFlags_includePositionData) != PSVRResult_Success)
		{
			m_areAllControllerStreamsActive= false;
			break;
		}
    }

    // Start receiving data from the controller
    setState(m_areAllControllerStreamsActive 
		? AppStage_ColorCalibration::waitingForStreamStartResponse 
		: AppStage_ColorCalibration::failedControllerStartRequest);
}

void AppStage_ColorCalibration::request_set_controller_tracking_color(
    PSVRController *controllerView,
    PSVRTrackingColorType tracking_color)
{
    unsigned char r, g, b;

    switch (tracking_color)
    {
    case PSVRTrackingColorType_Magenta:
        r = 0xFF; g = 0x00; b = 0xFF;
        break;
    case PSVRTrackingColorType_Cyan:
        r = 0x00; g = 0xFF; b = 0xFF;
        break;
    case PSVRTrackingColorType_Yellow:
        r = 0xFF; g = 0xFF; b = 0x00;
        break;
    case PSVRTrackingColorType_Red:
        r = 0xFF; g = 0x00; b = 0x00;
        break;
    case PSVRTrackingColorType_Green:
        r = 0x00; g = 0xFF; b = 0x00;
        break;
    case PSVRTrackingColorType_Blue:
        r = 0x00; g = 0x00; b = 0xFF;
        break;
    default:
        assert(0 && "unreachable");
    }

    PSVR_SetControllerLEDOverrideColor(controllerView->ControllerID, r, g, b);
}

void AppStage_ColorCalibration::request_start_hmd_stream()
{
    // Start receiving data from the controller
    setState(AppStage_ColorCalibration::pendingHmdStartRequest);

	// Turning on position stream data will enable the tracking lights
    if (PSVR_StartHmdDataStream(
			m_hmdView->HmdID, 
			PSVRStreamFlags_defaultStreamOptions | 
			PSVRStreamFlags_includePositionData) == PSVRResult_Success)
    {
        m_isHmdStreamActive = true;
        setState(AppStage_ColorCalibration::waitingForStreamStartResponse);
    }
    else
    {
        setState(AppStage_ColorCalibration::failedHmdStartRequest);
    }
}

void AppStage_ColorCalibration::request_tracker_start_stream()
{
    if (m_menuState != AppStage_ColorCalibration::pendingTrackerStartStreamRequest)
    {
        setState(AppStage_ColorCalibration::pendingTrackerStartStreamRequest);

        // Tell the PSVR service that we want to start streaming data from the tracker
        if (PSVR_StartTrackerDataStream(m_trackerView->tracker_info.tracker_id) == PSVRResult_Success)
        {
            PSVRTracker *trackerView = m_trackerView;

            // Open the shared memory that the video stream is being written to
            if (PSVR_OpenTrackerVideoStream(trackerView->tracker_info.tracker_id) == PSVRResult_Success)
            {
                allocate_video_buffers();
            }

			if (m_hmdView != nullptr)
			{
				// Now that the tracker stream is started, start the HMD stream
				request_start_hmd_stream();
			}
			else
			{
				request_start_controller_streams();
			}
        }
        else
        {
            setState(AppStage_ColorCalibration::failedTrackerStartStreamRequest);
        }
    }
}

void AppStage_ColorCalibration::allocate_video_buffers()
{
    m_video_buffer_state = new VideoBufferState(m_trackerView);
}

void AppStage_ColorCalibration::release_video_buffers()
{
    delete m_video_buffer_state;
    m_video_buffer_state = nullptr;
}

bool AppStage_ColorCalibration::request_tracker_set_mode(const char *new_mode)
{
    return PSVR_SetTrackerMode(m_trackerView->tracker_info.tracker_id, new_mode) == PSVRResult_Success;
}

void AppStage_ColorCalibration::request_tracker_set_video_property(PSVRVideoPropertyType prop_type, int value)
{
	int actual_value;
	if (PSVR_SetTrackerVideoProperty(
			m_trackerView->tracker_info.tracker_id, 
			prop_type,
			value, 
			true, 
			&actual_value) == PSVRResult_Success)
	{
		m_videoProperties[prop_type] = actual_value;
	}
}

void AppStage_ColorCalibration::request_tracker_set_color_filter(
    const PSVRTrackingColorType color_type,
    const PSVR_HSVColorRange &color_filter)
{

    PSVR_HSVColorRange desired_color_filter= color_filter;
    PSVR_HSVColorRange result_color_filter;

	if (m_hmdView != nullptr)
	{
		if (PSVR_SetTrackerHMDColorFilter(
				m_trackerView->tracker_info.tracker_id, m_hmdView->HmdID, color_type,
				&desired_color_filter, &result_color_filter) == PSVRResult_Success)
		{
			m_colorPresetTable.color_presets[color_type]= result_color_filter;
		}
	}
	else
	{
		if (PSVR_SetTrackerControllerColorFilter(
				m_trackerView->tracker_info.tracker_id, m_masterControllerView->ControllerID, color_type,
				&desired_color_filter, &result_color_filter) == PSVRResult_Success)
		{
			m_colorPresetTable.color_presets[color_type]= result_color_filter;
		}
	}
}

void AppStage_ColorCalibration::request_tracker_get_settings()
{
    PSVRClientTrackerSettings settings;

	if (m_overrideHmdId != -1)
	{
		if (PSVR_GetHMDTrackerSettings(m_trackerView->tracker_info.tracker_id, m_overrideHmdId, &settings) == PSVRResult_Success)
		{
			memcpy(m_videoProperties, settings.video_properties, sizeof(m_videoProperties));
			m_colorPresetTable = settings.color_range_table;
		}
	}
	else
	{
		if (PSVR_GetControllerTrackerSettings(m_trackerView->tracker_info.tracker_id, m_overrideControllerId, &settings) == PSVRResult_Success)
		{
			memcpy(m_videoProperties, settings.video_properties, sizeof(m_videoProperties));
			m_colorPresetTable = settings.color_range_table;
		}
	}
}

void AppStage_ColorCalibration::release_devices()
{
    release_video_buffers();

    for (PSVRController *controllerView : m_controllerViews)
    {
        PSVR_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);

        if (m_areAllControllerStreamsActive)
        {
            PSVR_StopControllerDataStream(controllerView->ControllerID);
        }

        PSVR_FreeControllerListener(controllerView->ControllerID);
    }
    m_controllerViews.clear();

    m_masterControllerView = nullptr;
    m_areAllControllerStreamsActive= false;
    m_lastMasterControllerSeqNum= -1;


    if (m_hmdView != nullptr)
    {
        if (m_isHmdStreamActive)
        {
            PSVR_StopHmdDataStream(m_hmdView->HmdID);
        }

        PSVR_FreeHmdListener(m_hmdView->HmdID);
        m_hmdView = nullptr;
        m_isHmdStreamActive = false;
        m_lastHmdSeqNum = -1;
    }

    if (m_trackerView != nullptr)
    {
        PSVR_CloseTrackerVideoStream(m_trackerView->tracker_info.tracker_id);
        PSVR_StopTrackerDataStream(m_trackerView->tracker_info.tracker_id);

        PSVR_FreeTrackerListener(m_trackerView->tracker_info.tracker_id);
        m_trackerView = nullptr;
    }
}

void AppStage_ColorCalibration::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

int AppStage_ColorCalibration::getVideoPropertyStepSize(PSVRVideoPropertyType prop_type) const
{
	return m_trackerView->tracker_info.video_property_constraints[prop_type].stepping_delta;
}

bool AppStage_ColorCalibration::canIncVideoProperty(PSVRVideoPropertyType prop_type) const
{
	return m_videoProperties[prop_type] < m_trackerView->tracker_info.video_property_constraints[prop_type].max_value;
}

bool AppStage_ColorCalibration::canDecVideoProperty(PSVRVideoPropertyType prop_type) const
{
	return m_videoProperties[prop_type] > m_trackerView->tracker_info.video_property_constraints[prop_type].min_value;
}

void AppStage_ColorCalibration::request_turn_on_all_tracking_bulbs(bool bEnabled)
{
    assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());
    for (int list_index= 0; list_index < m_controllerViews.size(); ++list_index)
    {
        PSVRController *controllerView= m_controllerViews[list_index];

        if (controllerView == m_masterControllerView)
            continue;

        if (bEnabled)
        {
            request_set_controller_tracking_color(controllerView, m_controllerTrackingColorTypes[list_index]);
        }
        else
        {
            PSVR_SetControllerLEDOverrideColor(controllerView->ControllerID, 0, 0, 0);
        }
    }
}

void AppStage_ColorCalibration::request_change_controller(int step)
{
    assert(m_controllerViews.size() == m_controllerTrackingColorTypes.size());

    {
        PSVRController *controllerView = m_controllerViews[m_overrideControllerId];

        if (controllerView == m_masterControllerView) {
            PSVR_SetControllerLEDOverrideColor(m_masterControllerView->ControllerID, 0, 0, 0);
            if (m_overrideControllerId + step < static_cast<int>(m_controllerViews.size()) && m_overrideControllerId + step >= 0) {
                m_overrideControllerId = m_overrideControllerId + step;
                m_masterControllerView = m_controllerViews[m_overrideControllerId];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            }
            else if (step > 0) {
                m_overrideControllerId = 0;
                m_masterControllerView = m_controllerViews[0];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
                if (m_bAutoChangeTracker) setState(eMenuState::changeTracker);
            }
            else {
                m_overrideControllerId = static_cast<int>(m_controllerViews.size()) -1;
                m_masterControllerView = m_controllerViews[m_overrideControllerId];
                m_masterTrackingColorType = m_controllerTrackingColorTypes[m_overrideControllerId];
                request_set_controller_tracking_color(m_masterControllerView, m_masterTrackingColorType);
            }
        }
    }
    m_app->getAppStage<AppStage_TrackerSettings>()->setSelectedControllerIndex(m_overrideControllerId);
}

void AppStage_ColorCalibration::request_change_tracker(int step)
{
    m_app->getAppStage<AppStage_ColorCalibration>()->
        set_autoConfig(m_bAutoChangeColor, m_bAutoChangeController, m_bAutoChangeTracker);

    if (tracker_index + step < tracker_count && tracker_index + step >= 0)
    {
        if (m_bShowAlignment) m_bAutoCalibrate = true;
        m_app->getAppStage<AppStage_TrackerSettings>()->setSelectedTrackerIndex(tracker_index + step);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
    else if (step > 0)
    {
        m_app->getAppStage<AppStage_TrackerSettings>()->setSelectedTrackerIndex(0);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
    else
    {
        m_app->getAppStage<AppStage_TrackerSettings>()->setSelectedTrackerIndex(tracker_count -1);
        request_exit_to_app_stage(AppStage_ColorCalibration::APP_STAGE_NAME);
    }
}

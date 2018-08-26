//-- inludes -----
#include "AppStage_HMDGyroscopeTest.h"
#include "AppStage_HMDSettings.h"
#include "AppStage_MainMenu.h"
#include "App.h"
#include "Camera.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathGLM.h"
#include "MathEigen.h"
#include "MathUtility.h"
#include "MathTypeConversion.h"

#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"

#include <imgui.h>

#include <algorithm>

//-- statics ----
const char *AppStage_HMDGyroscopeTest::APP_STAGE_NAME = "HMDGyroscopeCalibration";

//-- constants -----
const double k_stabilize_wait_time_ms = 1000.f;
const int k_desired_noise_sample_count = 1000;
const float k_desired_drift_sampling_time = 30.0*1000.f; // milliseconds

//-- private methods -----
static void drawHMD(PSVRHeadMountedDisplay *hmdView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDGyroscopeTest::AppStage_HMDGyroscopeTest(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDGyroscopeTest::inactive)
    , m_bBypassCalibration(false)
    , m_hmdView(nullptr)
    , m_isHMDStreamActive(false)
    , m_lastHMDSeqNum(-1)
    , m_lastRawGyroscope()
{
}

void AppStage_HMDGyroscopeTest::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const PSVRClientHMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(1000.f); // zoom out to see the magnetometer data at scale

    m_menuState = eCalibrationMenuState::waitingForStreamStartResponse;

    // Initialize the HMD state
    assert(hmdInfo->hmd_id != -1);
    assert(m_hmdView == nullptr);
	PSVR_AllocateHmdListener(hmdInfo->hmd_id);
	m_hmdView = PSVR_GetHmd(hmdInfo->hmd_id);

    m_lastRawGyroscope = *k_PSVR_int_vector3_zero;
    m_lastHMDSeqNum = -1;

    m_lastCalibratedAccelerometer = *k_PSVR_float_vector3_zero;
    m_lastCalibratedGyroscope = *k_PSVR_float_vector3_zero;

    // Start streaming in controller data
    assert(!m_isHMDStreamActive);

    if (PSVR_StartHmdDataStream(
		    m_hmdView->HmdID, 
		    PSVRStreamFlags_includeCalibratedSensorData | 
            PSVRStreamFlags_includeRawSensorData) == PSVRResult_Success)
    {
        m_isHMDStreamActive = true;
        m_lastHMDSeqNum = -1;
        m_menuState = eCalibrationMenuState::waitingForStreamStartResponse;
    }
    else
    {
        m_menuState = AppStage_HMDGyroscopeTest::failedStreamStart;
    }

}

void AppStage_HMDGyroscopeTest::exit()
{
    assert(m_hmdView != nullptr);
    PSVR_FreeHmdListener(m_hmdView->HmdID);
    m_hmdView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_HMDGyroscopeTest::update()
{
    bool bControllerDataUpdatedThisFrame = false;
    bool bTimeDeltaValid = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float, std::milli> sampleTimeDeltaMilli(0);

    if (m_isHMDStreamActive && m_hmdView->OutputSequenceNum != m_lastHMDSeqNum)
    {
        switch(m_hmdView->HmdType)
        {
        case PSVRHmd_Morpheus:
            {
                const PSVRMorpheusRawSensorData &rawSensorData =					
                    m_hmdView->HmdState.MorpheusState.RawSensorData;
                const PSVRMorpheusCalibratedSensorData &calibratedSensorData =
                    m_hmdView->HmdState.MorpheusState.CalibratedSensorData;

                m_lastRawGyroscope = rawSensorData.Gyroscope;
                m_lastCalibratedGyroscope = calibratedSensorData.Gyroscope;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            }
            break;
        case PSVRHmd_Virtual:
            {
                m_lastRawGyroscope = {0, 0, 0};
                m_lastCalibratedGyroscope = {0, 0, 0};
                m_lastCalibratedAccelerometer = {0, 0, 0};
            }
            break;
        }

        m_lastHMDSeqNum = m_hmdView->OutputSequenceNum;
        
        if (m_bLastSampleTimeValid)
        {
            sampleTimeDeltaMilli = now - m_lastSampleTime;
        }

        m_lastSampleTime= now;
        m_bLastSampleTimeValid= true;
        
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                m_app->getOrbitCamera()->resetOrientation();
                m_menuState = AppStage_HMDGyroscopeTest::test;
            }
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::test:
        {
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDGyroscopeTest::render()
{
	const float modelScale = 9.f;
	glm::mat4 hmdTransform;

	switch (m_hmdView->HmdType)
	{
	case PSVRHmd_Morpheus:
	case PSVRHmd_Virtual:
		hmdTransform = glm::scale(glm::mat4(1.f), glm::vec3(modelScale, modelScale, modelScale));
		break;
	}

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
    case eCalibrationMenuState::failedStreamStart:
        {
        } break;
    case eCalibrationMenuState::test:
        {
            // Get the orientation of the controller in world space (OpenGL Coordinate System)
			PSVRQuatf orientation;
			if (PSVR_GetHmdOrientation(m_hmdView->HmdID, &orientation) == PSVRResult_Success)
			{
				glm::quat q= PSVR_quatf_to_glm_quat(orientation);
				glm::mat4 worldSpaceOrientation= glm::mat4_cast(q);
				glm::mat4 worldTransform = glm::scale(worldSpaceOrientation, glm::vec3(modelScale, modelScale, modelScale));

				drawHMD(m_hmdView, worldTransform);
				drawTransformedAxes(worldSpaceOrientation, 200.f);
				drawTransformedAxes(glm::mat4(1.f), 200.f);
			}
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDGyroscopeTest::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "HMD Gyroscope Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for hmd stream to start...");

            ImGui::End();
        } break;
    case eCalibrationMenuState::failedStreamStart:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Failed to start hmd stream!");

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

            if (ImGui::Button("Return to Main Menu"))
            {
                request_exit_to_app_stage(AppStage_MainMenu::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;
    case eCalibrationMenuState::test:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin("Test Gyroscope", nullptr, window_flags);

            if (m_bBypassCalibration)
            {
                ImGui::Text("Testing Calibration of HMD ID #%d", m_hmdView->HmdID);
            }
            else
            {
                ImGui::Text("Calibration of HMD ID #%d complete!", m_hmdView->HmdID);
            }

            if (ImGui::Button("Ok"))
            {
                request_exit_to_app_stage(AppStage_HMDSettings::APP_STAGE_NAME);
            }

            ImGui::SameLine();

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

//-- private methods -----
void AppStage_HMDGyroscopeTest::request_exit_to_app_stage(const char *app_stage_name)
{
    PSVR_StopHmdDataStream(m_hmdView->HmdID);

    m_isHMDStreamActive= false;
    m_app->setAppStage(app_stage_name);
}

//-- private methods -----
static void drawHMD(PSVRHeadMountedDisplay *hmdView, const glm::mat4 &transform)
{
    switch(hmdView->HmdType)
    {
    case PSVRHmd_Morpheus:
        drawMorpheusModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case PSVRHmd_Virtual:
        drawVirtualHMDModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    }
}

//-- inludes -----
#include "AppStage_HMDAccelerometerTest.h"
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
const char *AppStage_HMDAccelerometerTest::APP_STAGE_NAME = "HMDAcceleromterCalibration";

//-- constants -----
static const double k_stabilize_wait_time_ms = 1000.f;
static const int k_max_accelerometer_samples = 500;

static const float k_min_sample_distance = 1000.f;
static const float k_min_sample_distance_sq = k_min_sample_distance*k_min_sample_distance;

//-- private methods -----
static void drawHMD(PSVRHeadMountedDisplay *controllerView, const glm::mat4 &transform);

//-- public methods -----
AppStage_HMDAccelerometerTest::AppStage_HMDAccelerometerTest(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDAccelerometerTest::inactive)
    , m_hmdView(nullptr)
    , m_isHMDStreamActive(false)
    , m_lastHMDSeqNum(-1)
{
}

void AppStage_HMDAccelerometerTest::enter()
{
    const AppStage_HMDSettings *hmdSettings = m_app->getAppStage<AppStage_HMDSettings>();
    const PSVRClientHMDInfo *hmdInfo = hmdSettings->getSelectedHmdInfo();

    // Reset the menu state
    m_app->setCameraType(_cameraOrbit);
    m_app->getOrbitCamera()->resetOrientation();
    m_app->getOrbitCamera()->setCameraOrbitRadius(500.f); // zoom out to see the magnetometer data at scale

    // Initialize the controller state
    assert(hmdInfo->hmd_id != -1);
    assert(m_hmdView == nullptr);
	PSVR_AllocateHmdListener(hmdInfo->hmd_id);
	m_hmdView = PSVR_GetHmd(hmdInfo->hmd_id);

    m_lastCalibratedAccelerometer = *k_PSVR_float_vector3_zero;
    m_lastHMDSeqNum = -1;

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
        m_menuState = AppStage_HMDAccelerometerTest::failedStreamStart;
    }
}

void AppStage_HMDAccelerometerTest::exit()
{
    assert(m_hmdView != nullptr);
    PSVR_FreeHmdListener(m_hmdView->HmdID);
    m_hmdView = nullptr;
    m_menuState = eCalibrationMenuState::inactive;

    // Reset the orbit camera back to default orientation and scale
    m_app->getOrbitCamera()->reset();
}

void AppStage_HMDAccelerometerTest::update()
{
    bool bControllerDataUpdatedThisFrame = false;

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

				m_lastRawAccelerometer = rawSensorData.Accelerometer;
                m_lastCalibratedAccelerometer = calibratedSensorData.Accelerometer;
            } break;
        case PSVRHmd_Virtual:
            {
                m_lastRawAccelerometer = {0, 0, 0};
                m_lastCalibratedAccelerometer = {0, 0, 0};
            } break;
        default:
            assert(0 && "unreachable");
        }

        m_lastHMDSeqNum = m_hmdView->OutputSequenceNum;
        bControllerDataUpdatedThisFrame = true;
    }

    switch (m_menuState)
    {
    case eCalibrationMenuState::waitingForStreamStartResponse:
        {
            if (bControllerDataUpdatedThisFrame)
            {
                m_app->getOrbitCamera()->resetOrientation();
                m_menuState = AppStage_HMDAccelerometerTest::test;
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

void AppStage_HMDAccelerometerTest::render()
{
    const float modelScale = 9.f;
    glm::mat4 hmdTransform;

    switch(m_hmdView->HmdType)
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
            const float sensorScale = 100.f;
            glm::mat4 sensorTransform = glm::scale(glm::mat4(1.f), glm::vec3(sensorScale, sensorScale, sensorScale));

            drawHMD(m_hmdView, hmdTransform);
            drawTransformedAxes(hmdTransform, 500.f);

            // Draw the current filtered accelerometer direction
            {
                const float accel_g = PSVR_Vector3fLength(&m_lastCalibratedAccelerometer);
                glm::vec3 m_start = glm::vec3(0.f);
                glm::vec3 m_end = PSVR_vector3f_to_glm_vec3(m_lastCalibratedAccelerometer);

                drawArrow(sensorTransform, m_start, m_end, 0.1f, glm::vec3(1.f, 0.f, 0.f));
                drawTextAtWorldPosition(sensorTransform, m_end, "A(%.1fg)", accel_g);
            }
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDAccelerometerTest::renderUI()
{
    const float k_panel_width = 500;
    const char *k_window_title = "HMD Accelerometer Calibration";
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
            ImGui::Begin("Test Accelerometer", nullptr, window_flags);

            ImGui::Text("Testing Calibration of HMD ID #%d", m_hmdView->HmdID);

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
void AppStage_HMDAccelerometerTest::request_exit_to_app_stage(const char *app_stage_name)
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

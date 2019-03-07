//-- includes -----
#include "AppSubStage_CalibrateWithHMD.h"
#include "AppStage_ComputeTrackerPoses.h"
#include "AppStage_TrackerSettings.h"
#include "App.h"
#include "AssetManager.h"
#include "Camera.h"
#include "Logger.h"
#include "MathAlignment.h"
#include "MathTypeConversion.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"
#include "MathGLM.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <imgui.h>
#include <vector>

//-- constants -----
// Take 60 samples at each location
static const int k_hmd_calibration_sample_count = 60;
static const glm::vec3 k_psmove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const double k_stabilize_wait_time_ms = 1000.f;

//-- private definitions -----
struct TrackerRelativeHMDPoseStatistics
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	PSVRTracker *trackerView;

	// N samples for the current sampling location
	Eigen::Vector3f hmdPositionSamples[k_hmd_calibration_sample_count];
	Eigen::Quaternionf hmdOrientationSamples[k_hmd_calibration_sample_count];
	int sampleCount;

	// Average for each sampling location
	PSVRPosef avgHmdPose;

	PSVRPosef trackerPose;
	bool bValidTrackerPose;

	TrackerRelativeHMDPoseStatistics(PSVRTracker *_trackerView) : trackerView(_trackerView)
	{
		clearAll();
	}

	bool getIsComplete() const
	{
		return sampleCount >= k_hmd_calibration_sample_count;
	}

	void clearLastSampleBatch()
	{
		memset(hmdPositionSamples, 0, sizeof(Eigen::Vector3f)*k_hmd_calibration_sample_count);
		memset(hmdOrientationSamples, 0, sizeof(Eigen::Quaternionf)*k_hmd_calibration_sample_count);
		sampleCount = 0;
	}

	void clearAll()
	{
		clearLastSampleBatch();

		trackerPose = *k_PSVR_pose_identity;
		bValidTrackerPose = false;
	}

	void addHmdSample(const PSVRHeadMountedDisplay *hmd)
	{
		const int sampleTrackerID= trackerView->tracker_info.tracker_id;
        int streamTrackerID= -1;

		PSVRQuatf trackerRelativeOrientation;
		PSVRVector3f trackerRelativePosition;
		
		if (!getIsComplete() &&
			PSVR_GetHmdOrientationOnTracker(
				hmd->HmdID, &streamTrackerID, &trackerRelativeOrientation) == PSVRResult_Success &&
			PSVR_GetHmdPositionOnTracker(
				hmd->HmdID, &streamTrackerID, &trackerRelativePosition) == PSVRResult_Success &&
			streamTrackerID == sampleTrackerID)
		{
			hmdPositionSamples[sampleCount]= PSVR_vector3f_to_eigen_vector3(trackerRelativePosition);
			hmdOrientationSamples[sampleCount]= PSVR_quatf_to_eigen_quaternionf(trackerRelativeOrientation);
			++sampleCount;

			if (getIsComplete())
			{
				const float N = static_cast<float>(k_hmd_calibration_sample_count);

				Eigen::Vector3f avgPosition= 
					eigen_vector3f_compute_mean(hmdPositionSamples, k_hmd_calibration_sample_count);

				Eigen::Quaternionf avgOrientation;
				if (!eigen_quaternion_compute_normalized_weighted_average(
						hmdOrientationSamples,
						nullptr,
						k_hmd_calibration_sample_count,
						&avgOrientation))
				{
					avgOrientation= hmdOrientationSamples[0];
				}

				avgHmdPose.Orientation= eigen_quaternionf_to_PSVR_quatf(avgOrientation);
				avgHmdPose.Position= eigen_vector3f_to_PSVR_vector3f(avgPosition);
			}
		}
	}

	bool computeTrackerPose()
	{
		if (getIsComplete())
		{
			// HMD defines the origin,
			// so the inverse of the "tracker relative HMD pose" is the tracker pose
			trackerPose= PSVR_PosefInverse(&avgHmdPose);
			bValidTrackerPose= true;

			return true;
		}
	
		return false;
	}
};

//-- public methods -----
AppSubStage_CalibrateWithHMD::AppSubStage_CalibrateWithHMD(
    AppStage_ComputeTrackerPoses *parentStage)
    : m_parentStage(parentStage)
    , m_menuState(AppSubStage_CalibrateWithHMD::eMenuState::invalid)
    , m_bIsStable(false)
	, m_sampleLocationIndex(0)
    , m_bNeedMoreSamplesAtLocation(false)
{
}

AppSubStage_CalibrateWithHMD::~AppSubStage_CalibrateWithHMD()
{
}

void AppSubStage_CalibrateWithHMD::enter()
{
    for (AppStage_ComputeTrackerPoses::t_tracker_state_list_iterator iter = m_parentStage->m_trackerViews.begin();
        iter != m_parentStage->m_trackerViews.end(); 
        ++iter)
	{
		m_deviceTrackerPoseStats.push_back(
			new TrackerRelativeHMDPoseStatistics(iter->trackerView));
	}

    setState(AppSubStage_CalibrateWithHMD::eMenuState::initial);
}

void AppSubStage_CalibrateWithHMD::exit()
{
    setState(AppSubStage_CalibrateWithHMD::eMenuState::initial);

	for (TrackerRelativeHMDPoseStatistics *stats : m_deviceTrackerPoseStats)
	{
		delete stats;
	}
	m_deviceTrackerPoseStats.clear();
}

void AppSubStage_CalibrateWithHMD::update()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::initial:
        {
            if (m_parentStage->get_calibration_hmd_view() != nullptr)
            {
                // Go immediately to the initial place HMD stage
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD);
            }
            else
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD:
        {
            const PSVRHeadMountedDisplay *HmdView= m_parentStage->get_calibration_hmd_view();

			bool bIsStable= false;
			bool bCanBeStable= PSVR_GetIsHmdStable(HmdView->HmdID, &bIsStable) == PSVRResult_Success;

            if ((bCanBeStable && bIsStable) || m_bForceStable)
            {
                std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();

                if (m_bIsStable)
                {
                    std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                    if (stableDuration.count() >= k_stabilize_wait_time_ms)
                    {
                        setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD);
                    }
                }
                else
                {
                    m_bIsStable = true;
                    m_stableStartTime = now;
                }
            }
            else
            {
                if (m_bIsStable)
                {
                    m_bIsStable = false;
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD:
        {
            const PSVRHeadMountedDisplay *HmdView= m_parentStage->get_calibration_hmd_view();

			bool bIsStable= false;
			bool bCanBeStable= PSVR_GetIsHmdStable(HmdView->HmdID, &bIsStable) == PSVRResult_Success;

            // See if any tracker needs more samples
            if (m_bNeedMoreSamplesAtLocation)
            {
                m_bNeedMoreSamplesAtLocation= false;

                if (m_deviceTrackerPoseStats[m_currentPoseStatsIndex]->getIsComplete())
                {
					TrackerRelativeHMDPoseStatistics *nextTrackerStats= nullptr;

                    if (m_currentPoseStatsIndex < m_deviceTrackerPoseStats.size())
                    {
						nextTrackerStats= m_deviceTrackerPoseStats[++m_currentPoseStatsIndex];
                    }

					if (nextTrackerStats != nullptr)
					{
						PSVR_SetHmdDataStreamTrackerIndex(
							HmdView->HmdID, 
							nextTrackerStats->trackerView->tracker_info.tracker_id);
						m_bNeedMoreSamplesAtLocation= true;
					}
                }
                else
                {
                    m_bNeedMoreSamplesAtLocation = true;
                }
            }

            if (m_bNeedMoreSamplesAtLocation)
            {
                // Only record samples when the controller is stable
                if ((bCanBeStable && bIsStable) || m_bForceStable)
                {
					TrackerRelativeHMDPoseStatistics *stats= m_deviceTrackerPoseStats[m_currentPoseStatsIndex];

					bool bIsTracking= false;
					bool bCanBeTracked= PSVR_GetIsHmdTracking(HmdView->HmdID, &bIsTracking) == PSVRResult_Success;

                    if (bCanBeTracked && bIsTracking)
                    {
						stats->addHmdSample(HmdView);
                    }
                }
                else
                {
                    // Whoops! The controller got moved.
                    // Reset the sample count at this location for all trackers and wait for it 
                    setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD);
                }
            }
            else
            {
                // If we have completed sampling at this location, wait until the controller is picked up
                if (!bIsStable)
                {
                    setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses);
                }
            }

            // Poll the next video frame from the tracker rendering
            m_parentStage->update_tracker_video();
        }
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
        {
            bool bSuccess = true;

            // Compute and the pose transform for each tracker
            for (TrackerRelativeHMDPoseStatistics *trackerSampleData : m_deviceTrackerPoseStats)
            {
                bSuccess&= trackerSampleData->computeTrackerPose();
            }

            if (bSuccess)
            {
                // Update the poses on each local tracker view and notify the service of the new pose
                for (TrackerRelativeHMDPoseStatistics *trackerSampleData : m_deviceTrackerPoseStats)
                {
                    const PSVRPosef trackerPose = trackerSampleData->trackerPose;
                    PSVRTracker *trackerView = trackerSampleData->trackerView;

                    m_parentStage->request_set_tracker_pose(&trackerPose, trackerView);
                }
            }

            if (bSuccess)
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess);
            }
            else
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed);
            }
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::render()
{
    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD:
        {
            // Draw the video from the PoV of the current tracker
            m_parentStage->render_tracker_video();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::renderUI()
{
    const float k_panel_width = 450.f;
    const char *k_window_title = "Compute Tracker Poses";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;
    const std::chrono::time_point<std::chrono::high_resolution_clock> now = 
        std::chrono::high_resolution_clock::now();

    switch (m_menuState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::initial:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Stand the HMD upright facing all trackers");

            if (m_bIsStable)
            {
                std::chrono::duration<double, std::milli> stableDuration = now - m_stableStartTime;

                ImGui::Text("[stable for %d/%dms]", 
                    static_cast<int>(stableDuration.count()),
                    static_cast<int>(k_stabilize_wait_time_ms));
            }
            else
            {
                ImGui::Text("[Not stable and upright]");
            }

            ImGui::Separator();

            if (m_parentStage->get_tracker_count() > 1)
            {
                ImGui::Text("Tracker #%d", m_parentStage->get_render_tracker_index() + 1);

                if (ImGui::Button("Previous Tracker"))
                {
                    m_parentStage->go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    m_parentStage->go_next_tracker();
                }
            }

            if (ImGui::Button("Trust me, it's stable"))
            {
                m_bForceStable= true;
            }
            ImGui::SameLine();
            if (ImGui::Button("Restart Calibration"))
            {
                setState(AppSubStage_CalibrateWithHMD::eMenuState::initial);
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 200));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Recording HMD samples");

            bool bAnyTrackersSampling = false;
            for (int tracker_index = 0; tracker_index < m_parentStage->get_tracker_count(); ++tracker_index)
            {
                const int sampleCount = m_deviceTrackerPoseStats[tracker_index]->sampleCount;

                if (sampleCount < k_hmd_calibration_sample_count)
                {
                    ImGui::Text("Tracker %d: sample %d/%d", tracker_index + 1, sampleCount, k_hmd_calibration_sample_count);
                    bAnyTrackersSampling = true;
                }
                else
                {
                    ImGui::Text("Tracker %d: COMPLETE", tracker_index + 1);
                }
            }

            if (!bAnyTrackersSampling)
            {
                ImGui::Text("Location sampling complete. Please pick up the HMD.");
            }

            ImGui::Separator();

            if (m_parentStage->get_tracker_count() > 1)
            {
                ImGui::Text("Tracker #%d", m_parentStage->get_render_tracker_index() + 1);

                if (ImGui::Button("Previous Tracker"))
                {
                    m_parentStage->go_previous_tracker();
                }
                ImGui::SameLine();
                if (ImGui::Button("Next Tracker"))
                {
                    m_parentStage->go_next_tracker();
                }
            }

            if (ImGui::Button("Cancel"))
            {
                m_parentStage->setState(AppStage_ComputeTrackerPoses::eMenuState::verifyTrackers);
            }

            ImGui::End();
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::drawCalibrationDebug(PSVRTrackerID tracker_id)
{

}

//-- private methods -----
void AppSubStage_CalibrateWithHMD::setState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);
        m_menuState = newState;
    }
}

void AppSubStage_CalibrateWithHMD::onExitState(
    AppSubStage_CalibrateWithHMD::eMenuState oldState)
{
    switch (oldState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::invalid:
    case AppSubStage_CalibrateWithHMD::eMenuState::initial:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppSubStage_CalibrateWithHMD::onEnterState(
    AppSubStage_CalibrateWithHMD::eMenuState newState)
{
    switch (newState)
    {
    case AppSubStage_CalibrateWithHMD::eMenuState::initial:
        {
			for (TrackerRelativeHMDPoseStatistics *stats : m_deviceTrackerPoseStats)
            {
                stats->clearAll();
            }

            m_sampleLocationIndex = 0;
            m_bIsStable = false;
            m_bForceStable = false;
            m_currentPoseStatsIndex= 0;
        }
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepPlaceHMD:
        {
			for (TrackerRelativeHMDPoseStatistics *stats : m_deviceTrackerPoseStats)
            {
                stats->clearLastSampleBatch();
            }

            m_bIsStable = false;
            m_bForceStable= false;
            m_bNeedMoreSamplesAtLocation= true;
            m_currentPoseStatsIndex= 0;

            // Start off getting getting projection data from tracker 0
            PSVR_SetHmdDataStreamTrackerIndex(m_parentStage->get_calibration_hmd_view()->HmdID, 0);
        } break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepRecordHMD:
        break;
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrationStepComputeTrackerPoses:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepSuccess:
    case AppSubStage_CalibrateWithHMD::eMenuState::calibrateStepFailed:
        break;
    default:
        assert(0 && "unreachable");
    }
}
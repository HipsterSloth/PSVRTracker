//-- inludes -----
#include "AppStage_HMDTrackingTest.h"
#include "AppStage_MainMenu.h"
#include "AppStage_TrackerSettings.h"
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
const char *AppStage_HMDTrackingTest::APP_STAGE_NAME = "HMDTrackingTest";

//-- constants -----
static const glm::vec3 k_hmd_frustum_color = glm::vec3(1.f, 0.788f, 0.055f);
static const glm::vec3 k_PSVRove_frustum_color = glm::vec3(0.1f, 0.7f, 0.3f);
static const glm::vec3 k_PSVRove_frustum_color_no_track = glm::vec3(1.0f, 0.f, 0.f);

//-- private methods -----
static void drawHMD(const PSVRHeadMountedDisplay *hmdView, const glm::mat4 &transform, const PSVRTrackingColorType trackingColorType);

//-- public methods -----
AppStage_HMDTrackingTest::AppStage_HMDTrackingTest(App *app)
    : AppStage(app)
    , m_menuState(AppStage_HMDTrackingTest::inactive)
    , m_pendingTrackerStartCount(0)
    , m_overrideTrackerId(-1)
    , m_bShowAlignment(false)
    , m_AlignmentOffset(0.f)
    , m_overrideHmdId(-1)
{ 
}

void AppStage_HMDTrackingTest::enterStageAndTestTracking(class App *app, PSVRHmdID reqeusted_hmd_id)
{
    AppStage_HMDTrackingTest *appStage = app->getAppStage<AppStage_HMDTrackingTest>();
    appStage->m_overrideHmdId = reqeusted_hmd_id;

    app->setAppStage(AppStage_HMDTrackingTest::APP_STAGE_NAME);
}

void AppStage_HMDTrackingTest::enter()
{
    // Kick off this request chain 
    // hmd list request
    // -> hmd start request
    // -> tracker list request
    // -> tracker start request
    request_hmd_list();

    m_app->setCameraType(_cameraFixed);
}

void AppStage_HMDTrackingTest::exit()
{
    release_devices();

    setState(eMenuState::inactive);
}

void AppStage_HMDTrackingTest::update()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::testTracking:
        break;
    case eMenuState::showTrackerVideo:
        update_tracker_video();
        break;
    default:
        assert(0 && "unreachable");
    }
}

void compute_opencv_camera_intrinsic_matrix(PSVRTrackerID tracker_id,
	                                    PSVRVideoFrameSection section,
                                        cv::Matx33f &intrinsicOut,
                                        cv::Matx<float, 5, 1> &distortionOut)
{
	PSVRTrackerIntrinsics tracker_intrinsics;
	PSVR_GetTrackerIntrinsics(tracker_id, &tracker_intrinsics);

    PSVRMatrix3d *camera_matrix= nullptr;
    PSVRDistortionCoefficients *distortion_coefficients= nullptr;

    if (tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS)
    {
        if (section == PSVRVideoFrameSection_Left)
        {
            camera_matrix= &tracker_intrinsics.intrinsics.stereo.left_camera_matrix;
            distortion_coefficients = &tracker_intrinsics.intrinsics.stereo.left_distortion_coefficients;
        }
        else if (section == PSVRVideoFrameSection_Right)
        {
            camera_matrix= &tracker_intrinsics.intrinsics.stereo.right_camera_matrix;
            distortion_coefficients = &tracker_intrinsics.intrinsics.stereo.right_distortion_coefficients;
        }
    }
    else if (tracker_intrinsics.intrinsics_type == PSVR_MONO_TRACKER_INTRINSICS)
    {
        camera_matrix = &tracker_intrinsics.intrinsics.mono.camera_matrix;
        distortion_coefficients = &tracker_intrinsics.intrinsics.mono.distortion_coefficients;
    }

    if (camera_matrix != nullptr && distortion_coefficients != nullptr)
    {
        double *m= (double *)camera_matrix->m;
   
        intrinsicOut(0, 0)= (float)m[0]; intrinsicOut(0, 1)=  (float)m[1]; intrinsicOut(0, 2)= (float)m[2];
        intrinsicOut(1, 0)= (float)m[3]; intrinsicOut(1, 1)= (float)m[4]; intrinsicOut(1, 2)= (float)m[5]; 
        intrinsicOut(2, 0)= (float)m[6]; intrinsicOut(2, 1)= (float)m[7];  intrinsicOut(2, 2)= (float)m[8];

        distortionOut(0, 0)= (float)distortion_coefficients->k1;
        distortionOut(1, 0)= (float)distortion_coefficients->k2;
        distortionOut(2, 0)= (float)distortion_coefficients->p1;
        distortionOut(3, 0)= (float)distortion_coefficients->p2;
        distortionOut(4, 0)= (float)distortion_coefficients->k3;
    }
}

static void draw_image_point_camera_lines(
	PSVRTrackerID tracker_id,
	const PSVRTrackingProjection &projection)
{
	cv::Matx33f intrinsic_matrix;
	cv::Matx<float, 5, 1> dist_coeffs;
	compute_opencv_camera_intrinsic_matrix(tracker_id, PSVRVideoFrameSection_Primary, intrinsic_matrix, dist_coeffs);

	// Extract the focal lengths and principal point from the intrinsic matrix
	float fx= intrinsic_matrix(0, 0);
	float fy= intrinsic_matrix(1, 1);
	float cx= intrinsic_matrix(0, 2);
	float cy= intrinsic_matrix(1, 2);

	// Convert the PSVRVector2f points in the projection into a cv::Point2f list
	std::vector<cv::Point2f> cv_image_points;
	const int image_point_count= projection.projections[0].shape.pointcloud.point_count;
	for (int image_point_index = 0; image_point_index < image_point_count; ++image_point_index)
	{
		const PSVRVector2f &image_point= projection.projections[0].shape.pointcloud.points[image_point_index];

		cv_image_points.push_back({image_point.x, image_point.y});
	}

	// Compute a normalized ray for each projection point
	// See: http://answers.opencv.org/question/4862/how-can-i-do-back-projection/
	std::vector<glm::vec3> out_image_point_rays;
	std::for_each(cv_image_points.begin(), cv_image_points.end(),
		[&out_image_point_rays, fx, fy, cx, cy](const cv::Point2f& image_point) {
			// NOTE!!!
			// Y-Axis is flipped since OpenCV has +Y pointing down
			// and our tracking model assumes that +Y is pointing up
			const float end_z= 100.f;
			const glm::vec3 ray_end((image_point.x - cx)*end_z/fx, (cy - image_point.y)*end_z/fy, end_z);

			out_image_point_rays.push_back(glm::vec3(0.f, 0.f, 0.f));
			out_image_point_rays.push_back(ray_end);
		});

	drawLineList3d(
		glm::mat4(1), glm::vec3(1.f, 0.f, 0.f), 
		(float *)out_image_point_rays.data(), 
		(int)out_image_point_rays.size());
}

void AppStage_HMDTrackingTest::render()
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::testTracking:
        {
            // Draw the chaperone origin axes
            drawTransformedAxes(glm::mat4(1.0f), 100.f);

            // Draw the frustum for each tracking camera.
            // The frustums are defined in PSVRove tracking space.
            // We need to transform them into chaperone space to display them along side the HMD.
            {
                const int tracker_id= m_trackerView->tracker_info.tracker_id;
                const PSVRPosef trackerPose = m_trackerView->tracker_info.tracker_pose;
                const glm::mat4 trackerMat4 = PSVR_posef_to_glm_mat4(trackerPose);

                PSVRFrustum frustum;
                PSVR_GetTrackerFrustum(tracker_id, &frustum);

                // use color depending on tracking status
                glm::vec3 color= does_tracker_see_any_hmd(m_trackerView) ? k_PSVRove_frustum_color : k_PSVRove_frustum_color_no_track;

                drawTextAtWorldPosition(glm::mat4(1.f), PSVR_vector3f_to_glm_vec3(trackerPose.Position), "#%d", tracker_id);
                drawTransformedFrustum(glm::mat4(1.f), &frustum, color);

                drawTransformedAxes(trackerMat4, 20.f);
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

				PSVRRawTrackerData rawTrackerData;
				if (PSVR_GetHmdRawTrackerData(hmdView->HmdID, &rawTrackerData) == PSVRResult_Success)
				{
					drawTrackingShape(&rawTrackerData.WorldRelativeShape, glm::vec3(1.f, 1.f, 0.f));
					draw_image_point_camera_lines(rawTrackerData.TrackerID, rawTrackerData.TrackingProjection);
				}
            }
        } break;
    case eMenuState::showTrackerVideo:
        {
            render_tracker_video();
        } break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDTrackingTest::renderUI()
{
    const float k_panel_width = 300.f;
    const char *k_window_title = "Test HMD Tracking Pose";
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

    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        {
            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 80));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Pending device initialization...");

            if (ImGui::Button("Return to Tracker Settings"))
            {
                request_exit_to_app_stage(AppStage_TrackerSettings::APP_STAGE_NAME);
            }

            ImGui::End();
        } break;

    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 180));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            switch (m_menuState)
            {
            case eMenuState::failedHmdListRequest:
                ImGui::Text("Failed HMD list retrieval!");
                break;
            case eMenuState::failedHmdStartRequest:
                ImGui::Text("Failed HMD stream start!");
                break;
            case eMenuState::failedTrackerListRequest:
                ImGui::Text("Failed tracker list retrieval!");
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

    case eMenuState::testTracking:
        {
            ImGui::SetNextWindowPos(ImVec2(20.f, 20.f));
            ImGui::SetNextWindowSize(ImVec2(250.f, 260.f));
            ImGui::Begin("Test Tracking Pose", nullptr, window_flags);

            // display per tracker UI
            {
                ImGui::PushItemWidth(125.f);
                if (does_tracker_see_any_hmd(m_trackerView))
                {
                    ImGui::Text("Tracker #%d: OK", m_trackerView->tracker_info.tracker_id);
                }
                else 
                {
                    ImGui::Text("Tracker #%d: FAIL", m_trackerView->tracker_info.tracker_id);
                }
                ImGui::PopItemWidth();

                ImGui::SameLine();

                ImGui::PushItemWidth(100.f);
                ImGui::PushID(m_trackerView->tracker_info.tracker_id);
                if (ImGui::Button("Tracker Video") || m_trackerView->tracker_info.tracker_id == m_overrideTrackerId)
                {
                    m_overrideTrackerId = -1;
                    setState(eMenuState::showTrackerVideo);
                }
                ImGui::PopID();
                ImGui::PopItemWidth();
            }

            ImGui::Separator();

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

    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDTrackingTest::setState(eMenuState newState)
{
    if (newState != m_menuState)
    {
        onExitState(m_menuState);
        onEnterState(newState);

        m_menuState = newState;
    }
}

void AppStage_HMDTrackingTest::onExitState(eMenuState newState)
{
    switch (m_menuState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingHmdListRequest:
    case eMenuState::pendingHmdStartRequest:
    case eMenuState::pendingTrackerListRequest:
    case eMenuState::pendingTrackerStartRequest:
        break;
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
        break;
    case eMenuState::testTracking:
        m_app->setCameraType(_cameraFixed);
        break;
    case eMenuState::showTrackerVideo:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDTrackingTest::onEnterState(eMenuState newState)
{
    switch (newState)
    {
    case eMenuState::inactive:
        break;
    case eMenuState::pendingHmdListRequest:
        break;
    case eMenuState::pendingHmdStartRequest:
        m_hmdViews.clear();
        m_pendingHmdStartCount = 0;
        break;
    case eMenuState::pendingTrackerListRequest:
        break;
    case eMenuState::pendingTrackerStartRequest:
        m_trackerView= nullptr;
        m_textureAsset[0]= nullptr;
        m_textureAsset[1]= nullptr;
        m_pendingTrackerStartCount = 0;
        break;
    case eMenuState::failedHmdListRequest:
    case eMenuState::failedHmdStartRequest:
    case eMenuState::failedTrackerListRequest:
    case eMenuState::failedTrackerStartRequest:
    case eMenuState::testTracking:
        {
            m_app->setCameraType(_cameraOrbit);
        }
        break;
    case eMenuState::showTrackerVideo:
        break;
    default:
        assert(0 && "unreachable");
    }
}

void AppStage_HMDTrackingTest::update_tracker_video()
{
    const int tracker_id= m_trackerView->tracker_info.tracker_id;

    // Render the latest from the currently active tracker
    const unsigned char *buffer= nullptr;
    if (m_bIsStereoTracker)
    {
        if (PSVR_GetTrackerVideoFrameBuffer(tracker_id, PSVRVideoFrameSection_Left, &buffer) == PSVRResult_Success)
        {
            m_textureAsset[0]->copyBufferIntoTexture(buffer);
        }

        if (PSVR_GetTrackerVideoFrameBuffer(tracker_id, PSVRVideoFrameSection_Right, &buffer) == PSVRResult_Success)
        {
            m_textureAsset[1]->copyBufferIntoTexture(buffer);
        }
    }
    else
    {
        if (PSVR_GetTrackerVideoFrameBuffer(tracker_id, PSVRVideoFrameSection_Primary, &buffer) == PSVRResult_Success)
        {
            m_textureAsset[0]->copyBufferIntoTexture(buffer);
        }
    }
}

void AppStage_HMDTrackingTest::render_tracker_video()
{
    if (m_bIsStereoTracker)
    {
        if (m_textureAsset[0] != nullptr && m_textureAsset[1] != nullptr)
        {
            drawFullscreenStereoTexture(m_textureAsset[0]->texture_id, m_textureAsset[1]->texture_id);
        }
    }
    else
    {
        if (m_textureAsset[0] != nullptr)
        {
            drawFullscreenTexture(m_textureAsset[0]->texture_id);
        }
    }
}

PSVRTracker *AppStage_HMDTrackingTest::get_render_tracker_view() const
{
    return m_trackerView;
}

PSVRHeadMountedDisplay *AppStage_HMDTrackingTest::get_calibration_hmd_view() const
{
    return (m_hmdViews.size() > 0) ? m_hmdViews.begin()->second.hmdView : nullptr;
}

void AppStage_HMDTrackingTest::release_devices()
{
    //###HipsterSloth $REVIEW Do we care about canceling in-flight requests?
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
    m_pendingHmdStartCount = 0;

    if (m_trackerView != nullptr)
    {
        if (m_textureAsset[1] != nullptr)
        {
            delete m_textureAsset[1];
            m_textureAsset[1]= nullptr;
        }

        if (m_textureAsset[0] != nullptr)
        {
            delete m_textureAsset[0];
            m_textureAsset[0]= nullptr;
        }

        if (m_trackerView != nullptr)
        {
            const int tracker_id= m_trackerView->tracker_info.tracker_id;

            PSVR_CloseTrackerVideoStream(tracker_id);
            PSVR_StopTrackerDataStream(tracker_id);
            PSVR_FreeTrackerListener(tracker_id);
        }
    }
    m_trackerView= nullptr;
    m_pendingTrackerStartCount= 0;
}

void AppStage_HMDTrackingTest::request_exit_to_app_stage(const char *app_stage_name)
{
    release_devices();

    m_app->setAppStage(app_stage_name);
}

void AppStage_HMDTrackingTest::request_hmd_list()
{
    if (m_menuState != AppStage_HMDTrackingTest::pendingHmdListRequest)
    {
        m_menuState = AppStage_HMDTrackingTest::pendingHmdListRequest;
        
        // Request a list of controllers back from the server
        PSVRHmdList hmd_list;
        if (PSVR_GetHmdList(&hmd_list) == PSVRResult_Success)
        {
            handle_hmd_list_response(hmd_list);
        }
        else
        {
            setState(AppStage_HMDTrackingTest::failedHmdListRequest);
        }
    }
}

void AppStage_HMDTrackingTest::handle_hmd_list_response(
    const PSVRHmdList &hmd_list)
{
    if (m_overrideHmdId == -1)
    {
        bool bStartedAnyHMDs = false;

        // Start all head mounted displays
        for (int list_index = 0; list_index < hmd_list.count; ++list_index)
        {
            if (hmd_list.hmds[list_index].hmd_type == PSVRHmd_Morpheus ||
                hmd_list.hmds[list_index].hmd_type == PSVRHmd_Virtual)
            {
                const PSVRHmdID trackedHmdId = hmd_list.hmds[list_index].hmd_id;
                const PSVRTrackingColorType trackingColorType= hmd_list.hmds[list_index].tracking_color_type;

                request_start_hmd_stream(trackedHmdId, list_index, trackingColorType);
                bStartedAnyHMDs = true;
            }
        }

        if (!bStartedAnyHMDs)
        {
            // Move on to the tracker list if there are no HMDs
            request_tracker_list();
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
            if (hmd_list.hmds[list_index].hmd_id == m_overrideHmdId)
            {
                trackingColorType = hmd_list.hmds[list_index].tracking_color_type;
                trackedHmdId = hmd_list.hmds[list_index].hmd_id;
                trackedHmdListIndex = list_index;
                break;
            }
        }

        if (trackedHmdId != -1)
        {
            request_start_hmd_stream(trackedHmdId, trackedHmdListIndex, trackingColorType);
        }
        else
        {
            setState(AppStage_HMDTrackingTest::failedHmdListRequest);
        }
    }
}

void AppStage_HMDTrackingTest::request_start_hmd_stream(
    PSVRHmdID HmdID,
    int listIndex,
    PSVRTrackingColorType trackingColorType)
{
    HMDState hmdState;

    setState(eMenuState::pendingHmdStartRequest);

    // Allocate a new HMD view
    PSVR_AllocateHmdListener(HmdID);
    hmdState.listIndex = listIndex;
    hmdState.hmdView = PSVR_GetHmd(HmdID);
    hmdState.trackingColorType = trackingColorType;

    // Add the hmd to the list of HMDs we're monitoring
    assert(m_hmdViews.find(HmdID) == m_hmdViews.end());
    m_hmdViews.insert(t_id_hmd_state_pair(HmdID, hmdState));

    // Increment the number of requests we're waiting to get back
    ++m_pendingHmdStartCount;

    unsigned int flags =
        PSVRStreamFlags_includePositionData |
        PSVRStreamFlags_includeRawTrackerData;

    // Start off getting getting projection data from tracker 0
    PSVR_SetHmdDataStreamTrackerIndex(hmdState.hmdView->HmdID, static_cast<PSVRTrackerID>(0));

    // Start receiving data from the controller
    if (PSVR_StartHmdDataStream(hmdState.hmdView->HmdID, flags) == PSVRResult_Success)
    {
        handle_start_hmd_response();
    }
    else
    {
        setState(AppStage_HMDTrackingTest::failedHmdStartRequest);
    }
}

void AppStage_HMDTrackingTest::handle_start_hmd_response()
{
    // See if this was the last controller we were waiting to get a response from
    --m_pendingHmdStartCount;
    if (m_pendingHmdStartCount <= 0)
    {
        // Move on to the trackers
        request_tracker_list();
    }
}

void AppStage_HMDTrackingTest::request_tracker_list()
{
    if (m_menuState != eMenuState::pendingTrackerListRequest)
    {
        setState(eMenuState::pendingTrackerListRequest);

        // Tell the PSVRove service that we we want a list of trackers connected to this machine
        PSVRTrackerList tracker_list;
        if (PSVR_GetTrackerList(&tracker_list) == PSVRResult_Success &&
            tracker_list.count > 0)
        {
            handle_tracker_list_response(tracker_list);
        }
        else
        {
            setState(eMenuState::failedTrackerListRequest);
        }
    }
}

void AppStage_HMDTrackingTest::handle_tracker_list_response(
    const PSVRTrackerList &tracker_list)
{
    const PSVRClientTrackerInfo *TrackerInfo= nullptr;
    if (m_overrideTrackerId != -1)
    {
        for (int list_index = 0; list_index < tracker_list.count; ++list_index)
        {
            if (tracker_list.trackers[list_index].tracker_id == m_overrideTrackerId)
            {
                TrackerInfo= &tracker_list.trackers[list_index];
                break;
            }
        }
    }
    else
    {
        TrackerInfo= &tracker_list.trackers[0];
    }

    request_tracker_start_stream(TrackerInfo);
}

void AppStage_HMDTrackingTest::request_tracker_start_stream(
    const PSVRClientTrackerInfo *TrackerInfo)
{
    setState(eMenuState::pendingTrackerStartRequest);

    // Allocate a new tracker view
    const PSVRTrackerID tracker_id= TrackerInfo->tracker_id;
    PSVR_AllocateTrackerListener(tracker_id, TrackerInfo);
    m_trackerView = PSVR_GetTracker(tracker_id);
    m_textureAsset[0] = nullptr;
    m_textureAsset[1] = nullptr;
    m_bIsStereoTracker= false;

    // Increment the number of requests we're waiting to get back
    ++m_pendingTrackerStartCount;

    // Request data to start streaming to the tracker
    if (PSVR_StartTrackerDataStream(TrackerInfo->tracker_id) == PSVRResult_Success)
    {
        handle_tracker_start_stream_response(TrackerInfo);
    }
    else
    {
        setState(eMenuState::failedTrackerStartRequest);
    }
}

void AppStage_HMDTrackingTest::handle_tracker_start_stream_response(
    const PSVRClientTrackerInfo *TrackerInfo)
{
    // Get the tracker ID this request was for
    const PSVRTrackerID tracker_id= TrackerInfo->tracker_id;

    // Open the shared memory that the video stream is being written to
    if (PSVR_OpenTrackerVideoStream(tracker_id) == PSVRResult_Success)
    {
        PSVRVector2f screenSize;
        PSVR_GetTrackerScreenSize(tracker_id, &screenSize);
        const unsigned int frameWidth = static_cast<unsigned int>(screenSize.x);
        const unsigned int frameHeight = static_cast<unsigned int>(screenSize.y);

        // Create a texture to render the video frame to
        m_textureAsset[0] = new TextureAsset();
        m_textureAsset[0]->init(
            frameWidth,
            frameHeight,
            GL_RGB, // texture format
            GL_BGR, // buffer format
            nullptr);

        if (TrackerInfo->tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS)
        {
            m_textureAsset[1] = new TextureAsset();
            m_textureAsset[1]->init(
                frameWidth,
                frameHeight,
                GL_RGB, // texture format
                GL_BGR, // buffer format
                nullptr);
            m_bIsStereoTracker= true;
        }
    }

    // See if this was the last tracker we were waiting to get a response from
    --m_pendingTrackerStartCount;
    if (m_pendingTrackerStartCount <= 0)
    {
        handle_all_devices_ready();
    }
}

void AppStage_HMDTrackingTest::handle_all_devices_ready()
{
    setState(eMenuState::verifyTrackers);
}

bool AppStage_HMDTrackingTest::does_tracker_see_any_hmd(const PSVRTracker *trackerView)
{
    bool bTrackerSeesAnyHmd = false;
    for (t_hmd_state_map_iterator hmd_iter = m_hmdViews.begin(); hmd_iter != m_hmdViews.end(); ++hmd_iter)
    {
        const PSVRHeadMountedDisplay *hmdView = hmd_iter->second.hmdView;
        const int tracker_id= trackerView->tracker_info.tracker_id;

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
        drawMorpheusModel(transform, glm::vec3(1.f, 1.f, 1.f));
        break;
    case PSVRHmd_Virtual:
        drawVirtualHMDModel(transform, bulb_color);
        break;
    }
}

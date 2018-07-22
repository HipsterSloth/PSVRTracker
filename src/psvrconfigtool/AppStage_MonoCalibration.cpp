// Derived From example 11-1 of "Learning OpenCV: Computer Vision with the OpenCV Library" by Gary Bradski

//-- includes -----
#include "AppStage_MonoCalibration.h"
#include "AppStage_TrackerSettings.h"
#include "AppStage_MainMenu.h"
#include "AssetManager.h"
#include "App.h"
#include "Camera.h"
#include "Logger.h"
#include "MathTypeConversion.h"
#include "MathUtility.h"
#include "Renderer.h"
#include "UIConstants.h"

#include "SDL_keycode.h"
#include "SDL_opengl.h"

#include <imgui.h>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <array>
#include <atomic>
#include <thread>
#include <vector>

#ifdef _MSC_VER
#pragma warning (disable: 4996) // 'This function or variable may be unsafe': snprintf
#define snprintf _snprintf
#endif

//-- statics ----
const char *AppStage_MonoCalibration::APP_STAGE_NAME = "MonoCalibration";

//-- constants -----
enum eVideoDisplayMode
{
    mode_bgr,
    mode_undistored,
    mode_grayscale,

    MAX_VIDEO_DISPLAY_MODES
};

static const char *k_video_display_mode_names[] = {
    "BGR",
    "Undistorted",
    "Grayscale"
};

enum eCalibrationPatternType
{
    mode_chessboard,
    mode_circlegrid,

    MAX_CALIBRATION_PATTERN_TYPES
};

static const char *k_calibration_pattern_names[] = {
    "Chessboard",
    "Circle Grid",
};

#define CHESSBOARD_PATTERN_W 9 // Internal corners
#define CHESSBOARD_PATTERN_H 7
#define CHESSBOARD_CORNER_COUNT (CHESSBOARD_PATTERN_W*CHESSBOARD_PATTERN_H)
#define DEFAULT_SQUARE_LEN_MM 22

#define CIRCLEGRID_PATTERN_W 4
#define CIRCLEGRID_PATTERN_H 11
#define CIRCLE_COUNT (CIRCLEGRID_PATTERN_W*CIRCLEGRID_PATTERN_H)
#define DEFAULT_CIRCLE_DIAMETER_MM 1.5
#define DEFAULT_CIRCLE_SPACING_MM  2.0

#define BOARD_MOVED_PIXEL_DIST 5
#define BOARD_MOVED_ERROR_SUM BOARD_MOVED_PIXEL_DIST*CHESSBOARD_CORNER_COUNT

#define BOARD_NEW_LOCATION_PIXEL_DIST 100 
#define BOARD_NEW_LOCATION_ERROR_SUM BOARD_NEW_LOCATION_PIXEL_DIST*CHESSBOARD_CORNER_COUNT

#define STRAIGHT_LINE_TOLERANCE 10 // error tolerance in pixels

#define DESIRED_CAPTURE_BOARD_COUNT 12

//-- typedefs -----
namespace cv
{
    typedef Matx<double, 5, 1> Matx51d;
}

//-- private definitions -----
class OpenCVMonoSectionState
{
public:
    OpenCVMonoSectionState(const PSVRClientTrackerInfo &_trackerInfo)
        : trackerInfo(_trackerInfo)
        , frameWidth(static_cast<int>(_trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_width))
        , frameHeight(static_cast<int>(_trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_height))
        // Video frame buffers
        , bgrSourceBuffer(nullptr)
        , gsSourceBuffer(nullptr)
        , gsUndistortBuffer(nullptr)
        , bgrUndistortBuffer(nullptr)
        , bgrGsUndistortBuffer(nullptr)
        // Chess board computed state
        , capturedPatternCount(0)
        , lastValidImagePoints(0)
        , currentImagePoints(0)
        , bCurrentImagePointsValid(false)
        , quadList(0)
        , imagePointsList(0)
        // Calibration state
        , intrinsic_matrix()
        , distortion_coeffs()
        // Distortion preview
        , distortionMapX(nullptr)
        , distortionMapY(nullptr)
    {
        // Video Frame data
        bgrSourceBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsSourceBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsUndistortBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        bgrUndistortBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        bgrGsUndistortBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);

        // Distortion state
        distortionMapX = new cv::Mat(cv::Size(frameWidth, frameHeight), CV_32FC1);
        distortionMapY = new cv::Mat(cv::Size(frameWidth, frameHeight), CV_32FC1);

        resetCalibrationState();
    }

    virtual ~OpenCVMonoSectionState()
    {
        // Video Frame data
        delete bgrSourceBuffer;
        delete gsSourceBuffer;
        delete gsUndistortBuffer;
        delete bgrGsUndistortBuffer;
        delete bgrUndistortBuffer;

        // Distortion state
        delete distortionMapX;
        delete distortionMapY;
    }

    void resetCaptureState()
    {
        capturedPatternCount= 0;
        bCurrentImagePointsValid= false;
        currentImagePoints.clear();
        lastValidImagePoints.clear();
        quadList.clear();
        imagePointsList.clear();
    }

    void resetCalibrationState()
    {
        PSVRMatrix3d PSVR_camera_matrix= 
			trackerInfo.tracker_intrinsics.intrinsics.mono.camera_matrix;
        PSVRDistortionCoefficients PSVR_distortion_coeffs= 
			trackerInfo.tracker_intrinsics.intrinsics.mono.distortion_coefficients;

        intrinsic_matrix= PSVR_matrix3x3_to_cv_mat33d(PSVR_camera_matrix);
        
        // Fill in the distortion coefficients
        distortion_coeffs(0, 0)= PSVR_distortion_coeffs.k1; 
        distortion_coeffs(1, 0)= PSVR_distortion_coeffs.k2;
        distortion_coeffs(2, 0)= PSVR_distortion_coeffs.p1;
        distortion_coeffs(3, 0)= PSVR_distortion_coeffs.p2;
        distortion_coeffs(4, 0)= PSVR_distortion_coeffs.k3;

        // Generate the distortion map that corresponds to the tracker's camera settings
        rebuildDistortionMap();
    }

    void applyVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        // Copy and Flip image about the x-axis
        videoBufferMat.copyTo(*bgrSourceBuffer);

        // Convert the video buffer to a grayscale image
        cv::cvtColor(*bgrSourceBuffer, *gsSourceBuffer, cv::COLOR_BGR2GRAY);

        // Apply the distortion map
        cv::remap(
            *bgrSourceBuffer, *bgrUndistortBuffer, 
            *distortionMapX, *distortionMapY, 
            cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::remap(
            *gsSourceBuffer, *gsUndistortBuffer, 
            *distortionMapX, *distortionMapY, 
            cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        // Convert the video buffer to a grayscale image
        cv::cvtColor(*gsUndistortBuffer, *bgrGsUndistortBuffer, cv::COLOR_GRAY2BGR);
    }

    bool findAndAppendNewChessBoard(bool appWantsAppend)
    {        
        bool bAppendNewChessBoard= false;

        if (capturedPatternCount < DESIRED_CAPTURE_BOARD_COUNT)
        {
            // Clear out the previous images points
            currentImagePoints.clear();

            // Find chessboard corners:
            if (cv::findChessboardCorners(
                    *gsSourceBuffer, 
                    cv::Size(CHESSBOARD_PATTERN_W, CHESSBOARD_PATTERN_H), 
                    currentImagePoints, // output corners
                    cv::CALIB_CB_ADAPTIVE_THRESH 
                    + cv::CALIB_CB_FILTER_QUADS 
                    // + cv::CALIB_CB_NORMALIZE_IMAGE is suuuper slow
                    + cv::CALIB_CB_FAST_CHECK))
            {
                // Get subpixel accuracy on those corners
                cv::cornerSubPix(
                    *gsSourceBuffer, 
                    currentImagePoints, // corners to refine
                    cv::Size(11, 11), // winSize- Half of the side length of the search window
                    cv::Size(-1, -1), // zeroZone- (-1,-1) means no dead zone in search
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

                // Append the new chessboard corner pixels into the image_points matrix
                // Append the corresponding 3d chessboard corners into the object_points matrix
                if (currentImagePoints.size() == CHESSBOARD_CORNER_COUNT) 
                {
                    bCurrentImagePointsValid= false;

                    if (lastValidImagePoints.size() > 0)
                    {
                        float error_sum= 0.f;

                        for (int corner_index= 0; corner_index < CHESSBOARD_CORNER_COUNT; ++corner_index)
                        {
                            float squared_error= static_cast<float>(cv::norm(currentImagePoints[corner_index] - lastValidImagePoints[corner_index]));

                            error_sum+= squared_error;
                        }

                        bCurrentImagePointsValid= error_sum >= BOARD_NEW_LOCATION_ERROR_SUM;
                    }
                    else
                    {
                        // We don't have previous capture.
                        bCurrentImagePointsValid= true;
                    }


                    // If it's a valid new location, append it to the board list
                    if (bCurrentImagePointsValid && appWantsAppend)
                    {
                        // Keep track of the corners of all of the chessboards we sample
                        quadList.push_back(currentImagePoints[0]);
                        quadList.push_back(currentImagePoints[CHESSBOARD_PATTERN_W - 1]);
                        quadList.push_back(currentImagePoints[CHESSBOARD_CORNER_COUNT-1]);
                        quadList.push_back(currentImagePoints[CHESSBOARD_CORNER_COUNT-CHESSBOARD_PATTERN_W]);                        

                        // Append the new images points and object points
                        imagePointsList.push_back(currentImagePoints);

                        // Remember the last valid captured points
                        lastValidImagePoints= currentImagePoints;

                        // Keep track of how many boards have been captured so far
                        capturedPatternCount++;

                        bAppendNewChessBoard= true;
                    }
                }
            }
        }

        return bAppendNewChessBoard;
    }

    bool findAndAppendNewCircleGrid(bool appWantsAppend)
    {        
        bool bAppendNewChessBoard= false;

        if (capturedPatternCount < DESIRED_CAPTURE_BOARD_COUNT)
        {
            // Clear out the previous images points
            currentImagePoints.clear();

            // Find circle grid centers:
            if (cv::findCirclesGrid(
                    *gsSourceBuffer, 
                    cv::Size(CIRCLEGRID_PATTERN_W, CIRCLEGRID_PATTERN_H), 
                    currentImagePoints, // output centers
                    cv::CALIB_CB_ASYMMETRIC_GRID))
            {
                // Append the new chessboard corner pixels into the image_points matrix
                // Append the corresponding 3d chessboard corners into the object_points matrix
                if (currentImagePoints.size() == CIRCLE_COUNT) 
                {
                    bCurrentImagePointsValid= false;

                    if (lastValidImagePoints.size() > 0)
                    {
                        float error_sum= 0.f;

                        for (int corner_index= 0; corner_index < CIRCLE_COUNT; ++corner_index)
                        {
                            float squared_error= static_cast<float>(cv::norm(currentImagePoints[corner_index] - lastValidImagePoints[corner_index]));

                            error_sum+= squared_error;
                        }

                        bCurrentImagePointsValid= error_sum >= BOARD_NEW_LOCATION_ERROR_SUM;
                    }
                    else
                    {
                        // We don't have previous capture.
                        bCurrentImagePointsValid= true;
                    }


                    // If it's a valid new location, append it to the board list
                    if (bCurrentImagePointsValid && appWantsAppend)
                    {
                        // Keep track of the corners of all of the circle grids we sample
                        quadList.push_back(currentImagePoints[0]);
                        quadList.push_back(currentImagePoints[CIRCLEGRID_PATTERN_W - 1]);
                        quadList.push_back(currentImagePoints[CIRCLE_COUNT-1]);
                        quadList.push_back(currentImagePoints[CIRCLE_COUNT-CIRCLEGRID_PATTERN_W]);                        

                        // Append the new images points and object points
                        imagePointsList.push_back(currentImagePoints);

                        // Remember the last valid captured points
                        lastValidImagePoints= currentImagePoints;

                        // Keep track of how many boards have been captured so far
                        capturedPatternCount++;

                        bAppendNewChessBoard= true;
                    }
                }
            }
        }

        return bAppendNewChessBoard;
    }

    void rebuildDistortionMap()
    {
        cv::initUndistortRectifyMap(
            intrinsic_matrix, distortion_coeffs, 
            cv::noArray(), // unneeded rectification transformation computed by stereoRectify()
                               // newCameraMatrix - can be computed by getOptimalNewCameraMatrix(), but
            intrinsic_matrix, // "In case of a monocular camera, newCameraMatrix is usually equal to cameraMatrix"
            cv::Size(frameWidth, frameHeight),
            CV_32FC1, // Distortion map type
            *distortionMapX, *distortionMapY);
    }

    const PSVRClientTrackerInfo &trackerInfo;
    int frameWidth;
    int frameHeight;

    // Video frame buffers
    cv::Mat *bgrSourceBuffer;
    cv::Mat *gsSourceBuffer;
    cv::Mat *gsUndistortBuffer;
    cv::Mat *bgrGsUndistortBuffer;
    cv::Mat *bgrUndistortBuffer;

    // Chess board computed state
    int capturedPatternCount;
    std::vector<cv::Point2f> lastValidImagePoints;
    std::vector<cv::Point2f> currentImagePoints;
    bool bCurrentImagePointsValid;
    std::vector<cv::Point2f> quadList;
    std::vector<std::vector<cv::Point2f>> imagePointsList;

    // Calibration state
    cv::Matx33d intrinsic_matrix;
    cv::Matx33d rectification_rotation;
    cv::Matx34d rectification_projection;
    cv::Matx51d distortion_coeffs;

    // Distortion preview
    cv::Mat *distortionMapX;
    cv::Mat *distortionMapY;
};

class OpenCVMonoState
{
public:
    OpenCVMonoState(const PSVRClientTrackerInfo &_trackerInfo)
        : m_videoDisplayMode(eVideoDisplayMode::mode_bgr)
		, calibrationPatternType(eCalibrationPatternType::mode_circlegrid)
        , trackerInfo(_trackerInfo)
        , frameWidth(_trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_width)
        , frameHeight(_trackerInfo.tracker_intrinsics.intrinsics.mono.pixel_height)
		, squareLengthMM(DEFAULT_SQUARE_LEN_MM)
		, circleSpacingMM(DEFAULT_CIRCLE_SPACING_MM)
		, circleDiameterMM(DEFAULT_CIRCLE_DIAMETER_MM)
        , reprojectionError(0.0)
        , valid_new_calibration(false)
        , async_compute_task(nullptr)
        , async_task_completed({false})
    {
        memset(&async_task_result, 0, sizeof(PSVRMonoTrackerIntrinsics));

        m_opencv_section_state= nullptr;
        m_video_texture= nullptr;
    }

    virtual ~OpenCVMonoState()
    {
        if (m_opencv_section_state != nullptr)
        {
            delete m_opencv_section_state;
            m_opencv_section_state= nullptr;
        }

        disposeVideoTextures();
    }

    void allocateVideoTextures(bool bypassCalibrationFlag)
    {
        const PSVRMonoTrackerIntrinsics &trackerIntrinsics= trackerInfo.tracker_intrinsics.intrinsics.mono;

        const int width= static_cast<int>(trackerIntrinsics.pixel_width);
        const int height= static_cast<int>(trackerIntrinsics.pixel_height);

        // Create a texture to render the video frame to
        m_video_texture = new TextureAsset();
        m_video_texture->init(
            width, 
            height,
            GL_RGB, // texture format
            GL_BGR, // buffer format
            nullptr);

        // Allocate opencv buffer state
        m_opencv_section_state = new OpenCVMonoSectionState(trackerInfo);

        // Copy the current calibration state saved for this tracker
        resetCalibrationState();

        // Say we have a valid calibration if we want to bypass the calibration stage
        valid_new_calibration= bypassCalibrationFlag;
    }

    void disposeVideoTextures()
    {
        // Free the texture we were rendering to
        if (m_video_texture != nullptr)
        {
            delete m_video_texture;
            m_video_texture = nullptr;
        }
    }

    void applyMonoVideoFrame(const unsigned char *video_frame_buffer)
    {
        // Compute the undistorted 
        m_opencv_section_state->applyVideoFrame(video_frame_buffer);

		// Update the video frame display texture
		switch (m_videoDisplayMode)
		{
		case mode_bgr:
			m_video_texture->copyBufferIntoTexture(m_opencv_section_state->bgrSourceBuffer->data);
			break;
		case mode_undistored:
			m_video_texture->copyBufferIntoTexture(m_opencv_section_state->bgrUndistortBuffer->data);
			break;
        case mode_grayscale:
			m_video_texture->copyBufferIntoTexture(m_opencv_section_state->bgrGsUndistortBuffer->data);
			break;
		default:
			assert(0 && "unreachable");
			break;
		}
    }

    void findNewCalibrationPattern()
    {
        ImGuiIO io_state = ImGui::GetIO();
        const bool bWantsAppend= io_state.KeysDown[32];

		switch (calibrationPatternType)
		{
		case eCalibrationPatternType::mode_chessboard:
			m_opencv_section_state->findAndAppendNewChessBoard(bWantsAppend);
			break;
		case eCalibrationPatternType::mode_circlegrid:
			m_opencv_section_state->findAndAppendNewCircleGrid(bWantsAppend);
			break;
		}
    }

    bool hasSampledAllCalibrationPatterns() const
    {
        return
            m_opencv_section_state->capturedPatternCount >= DESIRED_CAPTURE_BOARD_COUNT;
    }

    bool areCurrentImagePointsValid() const
    {
        return 
            m_opencv_section_state->bCurrentImagePointsValid;
    }

    float computeCalibrationProgress() const 
    {
        const float samplePercentage= 
            static_cast<float>(m_opencv_section_state->capturedPatternCount) / static_cast<float>(DESIRED_CAPTURE_BOARD_COUNT);

        return samplePercentage;
    }

    void resetCalibrationState()
    {
        m_opencv_section_state->resetCalibrationState();

        reprojectionError= 0.f;
        valid_new_calibration= false;
    }

    void resetCaptureAndCalibration()
    {
        resetCalibrationState();

        m_opencv_section_state->resetCaptureState();
        m_videoDisplayMode= mode_bgr;
    }

    inline void calcChessBoardCornerPositions(const float square_length_mm, std::vector<cv::Point3f>& corners)
    {
        corners.clear();
        
        for( int i = 0; i < CHESSBOARD_PATTERN_H; ++i )
        {
            for( int j = 0; j < CHESSBOARD_PATTERN_W; ++j )
            {
                corners.push_back(cv::Point3f(float(j)*square_length_mm, float(i)*square_length_mm, 0.f));
            }
        }
    }

    inline void calcCircleGridCenterPositions(
		const float grid_spacing_mm, 
		const float diameter_mm, 
		std::vector<cv::Point3f>& centers)
    {
		const int col_count= 2*CIRCLEGRID_PATTERN_W;
		const int row_count= CIRCLEGRID_PATTERN_H;
		const float radius_mm= diameter_mm / 2.f;
        
		centers.clear();        
        for( int row = 0; row < row_count; ++row )
        {
            for( int col = 0; col < col_count; ++col )
            {
				const bool bRowIsEven= (row % 2) == 0;
				const bool bColIsEven= (col % 2) == 0;

				if ((bRowIsEven && !bColIsEven) || (!bRowIsEven && bColIsEven))
				{
					centers.push_back(
						cv::Point3f(
							float(col)*grid_spacing_mm + radius_mm, 
							float(row)*grid_spacing_mm + radius_mm, 
							0.f));
				}
            }
        }
		assert(centers.size() == CIRCLE_COUNT);
    }

    inline PSVRDistortionCoefficients cv_vec5_to_PSVR_distortion(const cv::Matx51d &cv_distortion_coeffs)
    {
        PSVRDistortionCoefficients distortion_coeffs;
        distortion_coeffs.k1= cv_distortion_coeffs(0, 0);
        distortion_coeffs.k2= cv_distortion_coeffs(1, 0);
        distortion_coeffs.p1= cv_distortion_coeffs(2, 0);
        distortion_coeffs.p2= cv_distortion_coeffs(3, 0);
        distortion_coeffs.k3= cv_distortion_coeffs(4, 0);

        return distortion_coeffs;
    }

    void computeCameraCalibration()
    {
        assert(async_compute_task == nullptr);

        async_task_completed= false;
        async_compute_task = new std::thread([this] { 
            this->computeCameraCalibrationTask(); 
        });
    }

    bool getCameraCalibration(PSVRMonoTrackerIntrinsics &out_mono_intrinsics)
    {
        bool bFetchSuccess= false;

        if (async_task_completed.load())
        {
            async_compute_task->join();

            delete async_compute_task;
            async_compute_task= nullptr;

            out_mono_intrinsics= async_task_result;
            valid_new_calibration= true;
            bFetchSuccess= true;
        }

        return bFetchSuccess;
    }

    void computeCameraCalibrationTask()
    {
        // Copy over the pre-existing tracker intrinsics
        async_task_result= trackerInfo.tracker_intrinsics.intrinsics.mono;

        // Only need to calculate objectPointsList once,
        // then resize for each set of image points.
        std::vector<std::vector<cv::Point3f> > objectPointsList(1);

		switch (calibrationPatternType)
		{
		case mode_chessboard:
	        calcChessBoardCornerPositions(squareLengthMM, objectPointsList[0]);
			break;
		case mode_circlegrid:
			calcCircleGridCenterPositions(circleSpacingMM, circleDiameterMM, objectPointsList[0]);
			break;
		default:
			break;
		}
        objectPointsList.resize(m_opencv_section_state->imagePointsList.size(), objectPointsList[0]);
            
        // Compute the camera intrinsic matrix and distortion parameters
        reprojectionError= 
            cv::calibrateCamera(
                objectPointsList, 
				m_opencv_section_state->imagePointsList,
                cv::Size((int)frameWidth, (int)frameHeight), 
                m_opencv_section_state->intrinsic_matrix, 
				m_opencv_section_state->distortion_coeffs, // Output we care about
                cv::noArray(), cv::noArray(), // best fit board poses as rvec/tvec pairs
                cv::CALIB_FIX_ASPECT_RATIO,
                cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON));

        // Recompute the distortion mapping (for debug display only)
        m_opencv_section_state->rebuildDistortionMap();

        // Pack the calibration results into the output mono intrinsics structure
        async_task_result.camera_matrix= 
            cv_mat33d_to_PSVR_matrix3x3(m_opencv_section_state->intrinsic_matrix);
        async_task_result.distortion_coefficients= 
            cv_vec5_to_PSVR_distortion(m_opencv_section_state->distortion_coeffs);

        // Signal the main thread that the task is complete
        async_task_completed= true;
    }

    void renderSelectedVideoBuffers()
    {
        switch (m_videoDisplayMode)
        {
        case mode_bgr:
        case mode_undistored:
        case mode_grayscale:
            {
                assert(m_video_texture != nullptr);
                unsigned int texture_id = m_video_texture->texture_id;

                if (texture_id != 0)
                {
                    drawFullscreenTexture(texture_id);
                }
            } break;
        }
    }

    void renderChessboard()
    {
        // Draw the most recently capture chessboard
        if (m_opencv_section_state->currentImagePoints.size() > 0)
        {
			drawOpenCVChessBoard(
				frameWidth, frameHeight, 
				reinterpret_cast<float *>(m_opencv_section_state->currentImagePoints.data()), // cv::point2f is just two floats 
				static_cast<int>(m_opencv_section_state->currentImagePoints.size()),
				true);
        }

        // Draw the outlines of all of the chess boards 
        if (m_opencv_section_state->quadList.size() > 0)
        {
            drawQuadList2d(
                frameWidth, frameHeight, 
                glm::vec3(1.f, 1.f, 0.f), 
                    reinterpret_cast<float *>(m_opencv_section_state->quadList.data()), // cv::point2f is just two floats 
                    static_cast<int>(m_opencv_section_state->quadList.size()));
        }
    }

    // Menu state
    eVideoDisplayMode m_videoDisplayMode;
	eCalibrationPatternType calibrationPatternType;
    const PSVRClientTrackerInfo &trackerInfo;
    float frameWidth;
    float frameHeight;
	float squareLengthMM;
	float circleSpacingMM;
	float circleDiameterMM;

    // Calibration State
    double reprojectionError;
    bool valid_new_calibration;

    // Async task state
    std::thread *async_compute_task;
    std::atomic_bool async_task_completed;
    PSVRMonoTrackerIntrinsics async_task_result;

    // Video buffer state
    OpenCVMonoSectionState *m_opencv_section_state;
    TextureAsset *m_video_texture;
};

//-- public methods -----
AppStage_MonoCalibration::AppStage_MonoCalibration(App *app)
    : AppStage(app)
    , m_menuState(AppStage_MonoCalibration::inactive)
    , m_trackerExposure(0)
    , m_trackerGain(0)
    , m_bStreamIsActive(false)
    , m_tracker_view(nullptr)
{ 
    m_opencv_mono_state= nullptr;
}

void AppStage_MonoCalibration::enter()
{
    const AppStage_TrackerSettings *trackerSettings =
        m_app->getAppStage<AppStage_TrackerSettings>();
    const PSVRClientTrackerInfo *trackerInfo = trackerSettings->getSelectedTrackerInfo();
    assert(trackerInfo->tracker_id != -1);

    m_app->setCameraType(_cameraFixed);

    assert(m_tracker_view == nullptr);
	PSVR_AllocateTrackerListener(trackerInfo->tracker_id, trackerInfo);
	m_tracker_view = PSVR_GetTracker(trackerInfo->tracker_id);

    m_opencv_mono_state = new OpenCVMonoState(*trackerInfo);
	m_opencv_mono_state->calibrationPatternType= eCalibrationPatternType::mode_circlegrid;
	m_opencv_mono_state->squareLengthMM = DEFAULT_SQUARE_LEN_MM;
	m_opencv_mono_state->circleSpacingMM = DEFAULT_CIRCLE_SPACING_MM;
	m_opencv_mono_state->circleDiameterMM = DEFAULT_CIRCLE_DIAMETER_MM;

	assert(!m_bStreamIsActive);
	request_tracker_start_stream();
}

void AppStage_MonoCalibration::exit()
{
    m_menuState = AppStage_MonoCalibration::inactive;

    if (m_opencv_mono_state != nullptr)
    {
        delete m_opencv_mono_state;
        m_opencv_mono_state= nullptr;
    }

    // Revert unsaved modifications to the tracker settings
    request_tracker_reload_settings();

    PSVR_FreeTrackerListener(m_tracker_view->tracker_info.tracker_id);
    m_tracker_view = nullptr;
    m_bStreamIsActive= false;
}

void AppStage_MonoCalibration::update()
{
    if (m_menuState == AppStage_MonoCalibration::capture ||
        m_menuState == AppStage_MonoCalibration::testCalibration)
    {
        // Try and read the next video frame from shared memory
        const unsigned char *video_frame_buffer= nullptr;
		if (PSVR_GetTrackerVideoFrameBuffer(m_tracker_view->tracker_info.tracker_id, PSVRVideoFrameSection_Primary, &video_frame_buffer) == PSVRResult_Success)
		{
			// Update the video frame buffers
            m_opencv_mono_state->applyMonoVideoFrame(video_frame_buffer);
		}

        if (m_menuState == AppStage_MonoCalibration::capture)
        {
            // Update the chess board capture state
            m_opencv_mono_state->findNewCalibrationPattern();

            if (m_opencv_mono_state->hasSampledAllCalibrationPatterns())
            {
                // Kick off the async task (very expensive)
                m_opencv_mono_state->computeCameraCalibration();
                m_menuState= AppStage_MonoCalibration::processingCalibration;
            }
        }
    }
    else if (m_menuState == AppStage_MonoCalibration::processingCalibration)
    {
        PSVRMonoTrackerIntrinsics new_mono_intrinsics;

        if (m_opencv_mono_state->getCameraCalibration(new_mono_intrinsics))
        {
            // Update the camera intrinsics for this camera
            request_tracker_set_intrinsic(new_mono_intrinsics);

            m_opencv_mono_state->m_videoDisplayMode= mode_undistored;
            m_menuState= AppStage_MonoCalibration::testCalibration;
        }
    }
}

void AppStage_MonoCalibration::render()
{
    if (m_menuState == AppStage_MonoCalibration::capture)
    {
        m_opencv_mono_state->renderSelectedVideoBuffers();
        m_opencv_mono_state->renderChessboard();
    }
    else if (m_menuState == AppStage_MonoCalibration::testCalibration)
    {
        m_opencv_mono_state->renderSelectedVideoBuffers();
    }
}

void AppStage_MonoCalibration::renderCameraSettingsUI()
{
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    ImGui::SetNextWindowPos(ImVec2(10.f, ImGui::GetIO().DisplaySize.y - 100 - 10.f));
    ImGui::SetNextWindowSize(ImVec2(275, 100));
    ImGui::Begin("Video Controls", nullptr, window_flags);

    if (ImGui::Button("<##Filter"))
    {
        m_opencv_mono_state->m_videoDisplayMode =
            static_cast<eVideoDisplayMode>(
            (m_opencv_mono_state->m_videoDisplayMode + eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES - 1)
            % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
    }
    ImGui::SameLine();
    if (ImGui::Button(">##Filter"))
    {
        m_opencv_mono_state->m_videoDisplayMode =
            static_cast<eVideoDisplayMode>(
            (m_opencv_mono_state->m_videoDisplayMode + 1) % eVideoDisplayMode::MAX_VIDEO_DISPLAY_MODES);
    }
    ImGui::SameLine();
    ImGui::Text("Video Filter Mode: %s", k_video_display_mode_names[m_opencv_mono_state->m_videoDisplayMode]);

	const int exposure_step= m_tracker_view->tracker_info.video_property_constraints[PSVRVideoProperty_Exposure].stepping_delta;
    if (ImGui::Button("-##Exposure"))
    {
        request_tracker_set_temp_exposure(m_trackerExposure - exposure_step);
    }
    ImGui::SameLine();
    if (ImGui::Button("+##Exposure"))
    {
        request_tracker_set_temp_exposure(m_trackerExposure + exposure_step);
    }
    ImGui::SameLine();
    ImGui::Text("Exposure: %d", m_trackerExposure);

	const int gain_step= m_tracker_view->tracker_info.video_property_constraints[PSVRVideoProperty_Gain].stepping_delta;
    if (ImGui::Button("-##Gain"))
    {
        request_tracker_set_temp_gain(m_trackerGain - gain_step);
    }
    ImGui::SameLine();
    if (ImGui::Button("+##Gain"))
    {
        request_tracker_set_temp_gain(m_trackerGain + gain_step);
    }
    ImGui::SameLine();
    ImGui::Text("Gain: %d", m_trackerGain);

    ImGui::End();
}

void AppStage_MonoCalibration::renderUI()
{
    const float k_panel_width = 200.f;
    const char *k_window_title = "Distortion Calibration";
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_ShowBorders |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoCollapse;

    switch (m_menuState)
    {
	case eMenuState::showWarning:
		{
			const float k_wide_panel_width = 350.f;
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_wide_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_wide_panel_width, 130));

			ImGui::Begin("WARNING", nullptr, window_flags);

			ImGui::TextWrapped(
				"The tracker you want to calibrate already has pre-computed distortion and focal lengths." \
				"If you proceed you will be overriding these defaults.");

			ImGui::Spacing();

			if (ImGui::Button("Continue"))
			{
				m_menuState = eMenuState::enterBoardSettings;
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				request_exit();
			}

			ImGui::End();
		} break;
	case eMenuState::enterBoardSettings:
		{
			const float k_wide_panel_width = 350.f;
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_wide_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_wide_panel_width, 100));

			ImGui::Begin("Enter Calibration Settings", nullptr, window_flags);

			ImGui::PushItemWidth(100.f);		
			switch (m_opencv_mono_state->calibrationPatternType)
			{
			case eCalibrationPatternType::mode_chessboard:
				{
					if (ImGui::InputFloat("Square Length (mm)", &m_opencv_mono_state->squareLengthMM, 0.5f, 1.f, 1))
					{
						if (m_opencv_mono_state->squareLengthMM < 1.f)
						{
							m_opencv_mono_state->squareLengthMM = 1.f;
						}

						if (m_opencv_mono_state->squareLengthMM > 100.f)
						{
							m_opencv_mono_state->squareLengthMM = 100.f;
						}
					}
				} break;
			case eCalibrationPatternType::mode_circlegrid:
				{
					if (ImGui::InputFloat("Circle Spacing (mm)", &m_opencv_mono_state->circleSpacingMM, 0.1f, 1.f, 1))
					{
						if (m_opencv_mono_state->circleSpacingMM < 1.f)
						{
							m_opencv_mono_state->circleSpacingMM = 1.f;
						}

						if (m_opencv_mono_state->circleSpacingMM > 100.f)
						{
							m_opencv_mono_state->circleSpacingMM = 100.f;
						}
					}

					if (ImGui::InputFloat("Circle Diameter (mm)", &m_opencv_mono_state->circleDiameterMM, 0.1f, 1.f, 1))
					{
						if (m_opencv_mono_state->circleDiameterMM < 1.f)
						{
							m_opencv_mono_state->circleDiameterMM = 1.f;
						}

						if (m_opencv_mono_state->circleDiameterMM > m_opencv_mono_state->circleSpacingMM)
						{
							m_opencv_mono_state->circleDiameterMM = m_opencv_mono_state->circleSpacingMM;
						}
					}
				} break;
			}
			ImGui::PopItemWidth();

			ImGui::Spacing();

			if (ImGui::Button("Ok"))
			{
				// Crank up the exposure and gain so that we can see the chessboard
				// These overrides will get rolled back once tracker gets closed
				request_tracker_set_temp_exposure(128);
				request_tracker_set_temp_gain(128);

				m_menuState = eMenuState::capture;
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel"))
			{
				request_exit();
			}

			ImGui::End();
		} break;
    case eMenuState::capture:
        {
            assert (m_opencv_mono_state != nullptr);

            renderCameraSettingsUI();

            {
                ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 20.f));
                ImGui::SetNextWindowSize(ImVec2(k_panel_width, 110));
                ImGui::Begin(k_window_title, nullptr, window_flags);

                ImGui::ProgressBar(m_opencv_mono_state->computeCalibrationProgress(), ImVec2(k_panel_width - 20, 20));

                if (ImGui::Button("Restart"))
                {
                    m_opencv_mono_state->resetCaptureAndCalibration();
                }
                ImGui::SameLine();
                if (ImGui::Button("Cancel"))
                {
                    request_exit();
                }
                else if (m_opencv_mono_state->areCurrentImagePointsValid())
                {
                    ImGui::Text("Press spacebar to capture");
                }

                ImGui::End();
            }
        } break;

    case eMenuState::processingCalibration:
        {
			const float k_wide_panel_width = 250.f;
			ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_wide_panel_width / 2.f, 20.f));
			ImGui::SetNextWindowSize(ImVec2(k_wide_panel_width, 100));

			ImGui::Begin("PROCESSING", nullptr, window_flags);

			ImGui::TextWrapped(
                "Computing distortion calibration.\n" \
				"This may take a few seconds...");

			ImGui::End();
        } break;

    case eMenuState::testCalibration:
        {
            renderCameraSettingsUI();

            ImGui::SetNextWindowPos(ImVec2(ImGui::GetIO().DisplaySize.x / 2.f - k_panel_width / 2.f, 10.f));
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 110));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (!m_bypassCalibrationFlag)
            {
                ImGui::Text("Calibration complete!");
                ImGui::Text("Avg Error: %f", m_opencv_mono_state->reprojectionError);
            }

            if (!m_bypassCalibrationFlag)
            {
                if (ImGui::Button("Redo Calibration"))
                {
                    m_opencv_mono_state->resetCaptureAndCalibration();

                    m_menuState= eMenuState::capture;
                }
                ImGui::SameLine();
            }
            if (ImGui::Button("Ok"))
            {
                request_exit();
            }

            ImGui::End();
        } break;

    case eMenuState::pendingTrackerStartStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for tracker stream to start...");

            ImGui::End();
        } break;

    case eMenuState::failedTrackerStartStreamRequest:
    case eMenuState::failedTrackerOpenStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 130));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            if (m_menuState == eMenuState::failedTrackerStartStreamRequest)
                ImGui::Text("Failed to start tracker stream!");
            else
                ImGui::Text("Failed to open tracker stream!");

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

    case eMenuState::pendingTrackerStopStreamRequest:
        {
            ImGui::SetNextWindowPosCenter();
            ImGui::SetNextWindowSize(ImVec2(k_panel_width, 50));
            ImGui::Begin(k_window_title, nullptr, window_flags);

            ImGui::Text("Waiting for tracker stream to stop...");

            ImGui::End();
        } break;

    case eMenuState::failedTrackerStopStreamRequest:
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

void AppStage_MonoCalibration::request_tracker_start_stream()
{
    if (m_menuState != AppStage_MonoCalibration::pendingTrackerStartStreamRequest)
    {
        m_menuState = AppStage_MonoCalibration::pendingTrackerStartStreamRequest;

        // Tell the PSVR service that we want to start streaming data from the tracker
        if (PSVR_StartTrackerDataStream(m_tracker_view->tracker_info.tracker_id) == PSVRResult_Success)
        {
            handle_tracker_start_stream_response();
        }
        else
        {
            m_menuState = AppStage_MonoCalibration::failedTrackerStartStreamRequest;
        }
    }
}

void AppStage_MonoCalibration::handle_tracker_start_stream_response()
{
    m_bStreamIsActive = true;

    // Open the shared memory that the video stream is being written to
    if (PSVR_OpenTrackerVideoStream(m_tracker_view->tracker_info.tracker_id) == PSVRResult_Success)
    {
        m_opencv_mono_state->allocateVideoTextures(m_bypassCalibrationFlag);

        if (m_bypassCalibrationFlag)
        {
			// Crank up the exposure and gain so that we can see the chessboard
			// These overrides will get rolled back once tracker gets closed
			request_tracker_set_temp_exposure(128);
			request_tracker_set_temp_gain(128);

            m_menuState = AppStage_MonoCalibration::testCalibration;
        }
		else if (m_tracker_view->tracker_info.tracker_type == PSVRTrackerType::PSVRTracker_PS3Eye)
		{
    		// Warn the user if they are about to change the distortion calibration settings for the PS3Eye
			m_menuState = AppStage_MonoCalibration::showWarning;
		}
		else
		{
			// Start capturing chess boards
			m_menuState = AppStage_MonoCalibration::enterBoardSettings;
		}
    }
    else
    {
        m_menuState = AppStage_MonoCalibration::failedTrackerOpenStreamRequest;
    }
}

void AppStage_MonoCalibration::request_tracker_stop_stream()
{
    if (m_bStreamIsActive && m_menuState != AppStage_MonoCalibration::pendingTrackerStopStreamRequest)
    {
        m_menuState = AppStage_MonoCalibration::pendingTrackerStopStreamRequest;

        // Tell the PSVR service that we want to stop streaming data from the tracker
        if (PSVR_StopTrackerDataStream(m_tracker_view->tracker_info.tracker_id) == PSVRResult_Success)
        {
            handle_tracker_stop_stream_response();
        }
        else
        {
            m_menuState = AppStage_MonoCalibration::failedTrackerStopStreamRequest;
        }
    }
}

void AppStage_MonoCalibration::handle_tracker_stop_stream_response()
{
    m_menuState = AppStage_MonoCalibration::inactive;

    // In either case consider the stream as now inactive
    m_bStreamIsActive = false;

    // Close the shared memory buffer
	PSVR_CloseTrackerVideoStream(m_tracker_view->tracker_info.tracker_id);

    // Free memory allocated for the video frame textures
    m_opencv_mono_state->disposeVideoTextures();

    // After closing the stream, we should go back to the tracker settings
    m_app->setAppStage(AppStage_TrackerSettings::APP_STAGE_NAME);
}

void AppStage_MonoCalibration::request_tracker_set_temp_gain(int gain)
{
    // Tell the PSVR service that we want to change gain, but not save the change
    PSVR_SetTrackerVideoProperty(m_tracker_view->tracker_info.tracker_id, PSVRVideoProperty_Gain, gain, false, &m_trackerGain);
}

void AppStage_MonoCalibration::request_tracker_set_temp_exposure(int exposure)
{
    // Tell the PSVR service that we want to change exposure, but not save the change.
    PSVR_SetTrackerVideoProperty(m_tracker_view->tracker_info.tracker_id, PSVRVideoProperty_Exposure, exposure, false, &m_trackerExposure);
}

void AppStage_MonoCalibration::request_tracker_set_intrinsic(
    const PSVRMonoTrackerIntrinsics &new_intrinsics)
{
    // Update the intrinsic state on the tracker info
    // so that this becomes the new reset point.
    m_tracker_view->tracker_info.tracker_intrinsics.intrinsics.mono= new_intrinsics;

    PSVR_SetTrackerIntrinsics(m_tracker_view->tracker_info.tracker_id, &m_tracker_view->tracker_info.tracker_intrinsics);
}

void AppStage_MonoCalibration::request_tracker_reload_settings()
{
    PSVR_ReloadTrackerSettings(m_tracker_view->tracker_info.tracker_id);
}

void AppStage_MonoCalibration::request_exit()
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

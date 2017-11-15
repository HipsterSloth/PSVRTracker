//-- includes -----
#include "DeviceEnumerator.h"
#include "DeviceManager.h"
#include "ServerTrackerView.h"
#include "ServerHMDView.h"
#include "MathUtility.h"
#include "MathEigen.h"
#include "MathGLM.h"
#include "MathAlignment.h"
#include "PS3EyeTracker.h"
#include "Utility.h"
#include "Logger.h"
#include "MathTypeConversion.h"
#include "ServiceRequestHandler.h"
#include "TrackerManager.h"
#include "PoseFilterInterface.h"
#include "VirtualStereoTracker.h"

#include <memory>

#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <algorithm>

#define USE_OPEN_CV_ELLIPSE_FIT

//-- constants ----
static const int k_min_roi_size= 32;

//-- typedefs ----
typedef std::vector<cv::Point> t_opencv_int_contour;
typedef std::vector<t_opencv_int_contour> t_opencv_int_contour_list;

typedef std::vector<cv::Point2f> t_opencv_float_contour;
typedef std::vector<t_opencv_float_contour> t_opencv_float_contour_list;

//-- template utility methods
template<typename t_opencv_contour_type>
cv::Point2f computeSafeCenterOfMassForContour(const t_opencv_contour_type &contour);

//-- private methods -----
struct OpenCVPlane2D
{
    cv::Point2f origin;
    cv::Point3f coefficients; // coefficients a, b, c in 2d plane equation: a*x + b*y + c = 0

    OpenCVPlane2D() 
        : origin(0.f, 0.f)
        , coefficients(0.f, 0.f, 0.f)
    {
    }

    OpenCVPlane2D(
        const cv::Point2f &o,
        const cv::Point3f &c) 
        : origin(o)
        , coefficients(c)
    {
    }

    static OpenCVPlane2D createFromPoints(const cv::Point2f &a, const cv::Point2f &b, const cv::Point2f &inside)
    {
        const cv::Point2f tangent= b - a;
        const float tangent_length= static_cast<float>(cv::norm(tangent));
        cv::Point2f normal=
            !is_nearly_zero(tangent_length)
            ? cv::Point2f(-tangent.y/tangent_length, tangent.x/tangent_length) 
            : cv::Point2f(0.f, 0.f);

        // Make sure the "inside point" is on the positive side of the plane
        if ((inside - a).dot(normal) < 0)
        {
            normal= -normal;
        }

        const cv::Point2f center= (a + b) / 2.f;

        return OpenCVPlane2D::createFromPointAndNormal(center, normal);
    }

    static OpenCVPlane2D createFromPointAndNormal(const cv::Point2f &p, const cv::Point2f &n)
    {
        const float n_length= static_cast<float>(cv::norm(n));
        const cv::Point2f normal= !is_nearly_zero(n_length) ? (n / n_length) : cv::Point2f(0.f, 0.f);

        cv::Point3f coefficients;
        coefficients.x= normal.x;        // coefficient a
        coefficients.y= normal.y;        // coefficient b
        coefficients.z= -normal.dot(p);  // coefficient c

        return OpenCVPlane2D(p, coefficients); 
    }

    inline cv::Point2f getOrigin() const
    {
        return origin;
    }

    inline cv::Point2f getNormal() const
    {
        return cv::Point2f(coefficients.x, coefficients.y);
    }

    inline bool isValidPlane() const
    {
        return !is_nearly_zero(coefficients.x) || !is_nearly_zero(coefficients.y);
    }

    float signedDistance(const cv::Point2f &point) const
    {
        return point.x*coefficients.x + point.y*coefficients.y + coefficients.z;
    }

    float unsignedDistance(const cv::Point2f &point) const
    {
        return fabsf(signedDistance(point));
    }

    cv::Point2f computeTangent() const
    {
        return cv::Point2f(-coefficients.y, coefficients.x);
    }
};

class OpenCVBGRToHSVMapper
{
public:
    typedef cv::Point3_<uint8_t> ColorTuple;

    static OpenCVBGRToHSVMapper *allocate()
    {
        if (m_refCount == 0)
        {
            assert(m_instance == nullptr);
            m_instance = new OpenCVBGRToHSVMapper();
        }
        assert(m_instance != nullptr);

        ++m_refCount;
        return m_instance;
    }

    static void dispose(OpenCVBGRToHSVMapper *instance)
    {
        assert(m_instance != nullptr);
        assert(m_instance == instance);
        assert(m_refCount > 0);

        --m_refCount;
        if (m_refCount <= 0)
        {
            delete m_instance;
            m_instance = nullptr;
        }
    }

    void cvtColor(const cv::Mat &bgrBuffer, cv::Mat &hsvBuffer)
    {
        hsvBuffer.forEach<ColorTuple>([&bgrBuffer, this](ColorTuple &hsvColor, const int position[]) -> void {
            const ColorTuple &bgrColor = bgrBuffer.at<ColorTuple>(position[0], position[1]);
            const int b = bgrColor.x;
            const int g = bgrColor.y;
            const int r = bgrColor.z;
            const int LUTIndex = OpenCVBGRToHSVMapper::getLUTIndex(r, g, b);

            hsvColor = bgr2hsv->at<ColorTuple>(LUTIndex, 0);
        });
    }

private:
    static OpenCVBGRToHSVMapper *m_instance;
    static int m_refCount;

    OpenCVBGRToHSVMapper()
    {
        bgr2hsv = new cv::Mat(256*256*256, 1, CV_8UC3);

        int LUTIndex = 0;
        for (int r = 0; r < 256; ++r)
        {
            for (int g = 0; g < 256; ++g)
            {
                for (int b = 0; b < 256; ++b)
                {
                    bgr2hsv->at<ColorTuple>(LUTIndex, 0) = ColorTuple(b, g, r);
                    ++LUTIndex;
                }
            }
        }

        cv::cvtColor(*bgr2hsv, *bgr2hsv, cv::COLOR_BGR2HSV);
    }

    ~OpenCVBGRToHSVMapper()
    {
        delete bgr2hsv;
    }

    static int getLUTIndex(int r, int g, int b)
    {
        return (256 * 256)*r + 256*g + b;
    }

    cv::Mat *bgr2hsv;
};
OpenCVBGRToHSVMapper *OpenCVBGRToHSVMapper::m_instance = nullptr;
int OpenCVBGRToHSVMapper::m_refCount= 0;

class OpenCVBufferState
{
public:
    OpenCVBufferState(ITrackerInterface *device, PSVRVideoFrameSection _section)
        : section(_section)
        , bgrBuffer(nullptr)
        , bgrShmemBuffer(nullptr)
        , hsvBuffer(nullptr)
        , gsLowerBuffer(nullptr)
        , gsUpperBuffer(nullptr)
        , maskedBuffer(nullptr)
    {
        device->getVideoFrameDimensions(&frameWidth, &frameHeight, nullptr);

        bgrBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        bgrShmemBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        hsvBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        gsLowerBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        gsUpperBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC1);
        maskedBuffer = new cv::Mat(frameHeight, frameWidth, CV_8UC3);
        
        const TrackerManagerConfig &cfg= DeviceManager::getInstance()->m_tracker_manager->getConfig();
        if (cfg.use_bgr_to_hsv_lookup_table)
        {
            bgr2hsv = OpenCVBGRToHSVMapper::allocate();
        }
        else
        {
            bgr2hsv = nullptr;
        }
        
        //Apply default ROI (full frame).
        applyROI(cv::Rect2i(cv::Point(0,0), cv::Size(frameWidth, frameHeight)));
    }

    virtual ~OpenCVBufferState()
    {
        if (maskedBuffer != nullptr)
        {
            delete maskedBuffer;
        }
        
        if (gsLowerBuffer != nullptr)
        {
            delete gsLowerBuffer;
        }
        
        if (gsUpperBuffer != nullptr)
        {
            delete gsUpperBuffer;
        }
        
        if (hsvBuffer != nullptr)
        {
            delete hsvBuffer;
        }
        
        if (bgrShmemBuffer != nullptr)
        {
            delete bgrShmemBuffer;
        }
        
        if (bgrBuffer != nullptr)
        {
            delete bgrBuffer;
        }
        
        if (bgr2hsv != nullptr)
        {
            OpenCVBGRToHSVMapper::dispose(bgr2hsv);
        }
    }

    void writeVideoFrame(const unsigned char *video_buffer)
    {
        const cv::Mat videoBufferMat(frameHeight, frameWidth, CV_8UC3, const_cast<unsigned char *>(video_buffer));

        videoBufferMat.copyTo(*bgrBuffer);
        videoBufferMat.copyTo(*bgrShmemBuffer);
    }
    
    void updateHsvBuffer()
    {
        // Convert the video buffer to the HSV color space
        if (bgr2hsv != nullptr)
        {
            bgr2hsv->cvtColor(bgrROI, hsvROI);
        }
        else
        {
            cv::cvtColor(bgrROI, hsvROI, cv::COLOR_BGR2HSV);
        }
    }
    
    void applyROI(cv::Rect2i ROI)
    {
        // Make sure the ROI box is always clamped in bounds of the frame buffer
        int x0= std::min(std::max(ROI.tl().x, 0), frameWidth-1);
        int y0= std::min(std::max(ROI.tl().y, 0), frameHeight-1);
        int x1= std::min(std::max(ROI.br().x, 0), frameWidth-1);
        int y1= std::min(std::max(ROI.br().y, 0), frameHeight-1);
        int clamped_width = std::max(x1-x0, 0);
        int clamped_height = std::max(y1-y0, 0);

        // If the clamped ROI ends up being zero-width or zero-height, 
        // just make it full screen
        if (clamped_width > 0 && clamped_height > 0)
        {
            ROI.x= x0;
            ROI.y= y0;
            ROI.width = clamped_width;
            ROI.height = clamped_height;
        }
        else
        {
            ROI.x= 0;
            ROI.y= 0;
            ROI.width = frameWidth;
            ROI.height = frameHeight;
        }
       
        //Create the ROI matrices.
        //It's not a full copy, so this isn't too slow.
        //adjustROI is probably slightly faster but I ran into trouble with it.
        bgrROI = cv::Mat(*bgrBuffer, ROI);
        hsvROI = cv::Mat(*hsvBuffer, ROI);
        gsLowerROI = cv::Mat(*gsLowerBuffer, ROI);
        gsUpperROI = cv::Mat(*gsUpperBuffer, ROI);
        
        updateHsvBuffer();
        
        //Draw ROI.
        cv::rectangle(*bgrShmemBuffer, ROI, cv::Scalar(255, 0, 0));
    }

    // Return points in raw image space:
    // i.e. [0, 0] at lower left  to [frameWidth-1, frameHeight-1] at lower right
    bool computeBiggestNContours(
        const PSVR_HSVColorRange &hsvColorRange,
        t_opencv_int_contour_list &out_biggest_N_contours,
        std::vector<double> &out_contour_areas,
        const int max_contour_count,
        const int min_points_in_contour = 6)
    {
        out_biggest_N_contours.clear();
        out_contour_areas.clear();
        
        // Clamp the HSV image, taking into account wrapping the hue angle
        {
            const float hue_min = hsvColorRange.hue_range.center - hsvColorRange.hue_range.range;
            const float hue_max = hsvColorRange.hue_range.center + hsvColorRange.hue_range.range;
            const float saturation_min = clampf(hsvColorRange.saturation_range.center - hsvColorRange.saturation_range.range, 0, 255);
            const float saturation_max = clampf(hsvColorRange.saturation_range.center + hsvColorRange.saturation_range.range, 0, 255);
            const float value_min = clampf(hsvColorRange.value_range.center - hsvColorRange.value_range.range, 0, 255);
            const float value_max = clampf(hsvColorRange.value_range.center + hsvColorRange.value_range.range, 0, 255);

            if (hue_min < 0)
            {
                cv::inRange(
                    hsvROI,
                    cv::Scalar(0, saturation_min, value_min),
                    cv::Scalar(clampf(hue_max, 0, 180), saturation_max, value_max),
                    gsLowerROI);
                cv::inRange(
                    hsvROI,
                    cv::Scalar(clampf(180 + hue_min, 0, 180), saturation_min, value_min),
                    cv::Scalar(180, saturation_max, value_max),
                    gsUpperROI);
                cv::bitwise_or(gsLowerROI, gsUpperROI, gsLowerROI);
            }
            else if (hue_max > 180)
            {
                cv::inRange(
                    hsvROI,
                    cv::Scalar(0, saturation_min, value_min),
                    cv::Scalar(clampf(hue_max - 180, 0, 180), saturation_max, value_max),
                    gsLowerROI);
                cv::inRange(
                    hsvROI,
                    cv::Scalar(clampf(hue_min, 0, 180), saturation_min, value_min),
                    cv::Scalar(180, saturation_max, value_max),
                    gsUpperROI);
                cv::bitwise_or(gsLowerROI, gsUpperROI, gsLowerROI);
            }
            else
            {
                cv::inRange(
                    hsvROI,
                    cv::Scalar(hue_min, saturation_min, value_min),
                    cv::Scalar(hue_max, saturation_max, value_max),
                    gsLowerROI);
            }
        }
        
        //TODO: Why no blurring of the gsLowerBuffer?

        // Find the largest convex blob in the filtered grayscale buffer
        {
            struct ContourInfo
            {
                int contour_index;
                double contour_area;
            };
            std::vector<ContourInfo> sorted_contour_list;

            // Find all counters in the image buffer
            cv::Size size; cv::Point ofs;
            gsLowerROI.locateROI(size, ofs);
            t_opencv_int_contour_list contours;
            cv::findContours(gsLowerROI,
                             contours,
                             CV_RETR_EXTERNAL,
                             CV_CHAIN_APPROX_SIMPLE,  //CV_CHAIN_APPROX_NONE?
                             ofs);

            // Compute the area of each contour
            int contour_index = 0;
            for (auto it = contours.begin(); it != contours.end(); ++it) 
            {
                const double contour_area = cv::contourArea(*it);
                const ContourInfo contour_info = { contour_index, contour_area };

                sorted_contour_list.push_back(contour_info);
                ++contour_index;
            }
            
            // Sort the list of contours by area, largest to smallest
            if (sorted_contour_list.size() > 1)
            {
                std::sort(
                    sorted_contour_list.begin(), sorted_contour_list.end(), 
                    [](const ContourInfo &a, const ContourInfo &b) {
                        return b.contour_area < a.contour_area;
                });
            }

            // Copy up to N valid contours
            for (auto it = sorted_contour_list.begin(); 
                it != sorted_contour_list.end() && static_cast<int>(out_biggest_N_contours.size()) < max_contour_count; 
                ++it)
            {
                const ContourInfo &contour_info = *it;
                t_opencv_int_contour &contour = contours[contour_info.contour_index];

                if (contour.size() > min_points_in_contour)
                {
                    // Remove any points in contour on edge of camera/ROI
                    // TODO: Contours touching image border will be clipped,
                    // so this might not be necessary.
                    t_opencv_int_contour::iterator it = contour.begin();
                    while (it != contour.end()) 
                    {
                        if (it->x == 0 || it->x == (frameWidth - 1) || it->y == 0 || it->y == (frameHeight - 1))
                        {
                            it = contour.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }

                    // Add cleaned up contour to the output list
                    out_biggest_N_contours.push_back(contour);
                    // Add its area to the output list too.
                    out_contour_areas.push_back(contour_info.contour_area);
                }
            }
        }

        return (out_biggest_N_contours.size() > 0);
    }
    
    void
    draw_contour(const t_opencv_int_contour &contour)
    {
        // Draws the contour directly onto the shared mem buffer.
        // This is useful for debugging
        std::vector<t_opencv_int_contour> contours = {contour};
        const cv::Point2f massCenter = computeSafeCenterOfMassForContour<t_opencv_int_contour>(contour);
        cv::drawContours(*bgrShmemBuffer, contours, 0, cv::Scalar(255, 255, 255));
        cv::rectangle(*bgrShmemBuffer, cv::boundingRect(contour), cv::Scalar(255, 255, 255));
        cv::drawMarker(*bgrShmemBuffer, massCenter, cv::Scalar(255, 255, 255), 0,
            (cv::boundingRect(contour).height < cv::boundingRect(contour).width) ?
            cv::boundingRect(contour).height : cv::boundingRect(contour).width);
    }
    
    void
    draw_pose_projection(const PSVRTrackingProjection &pose_projection)
    {
        // Draw the projection of the pose onto the shared mem buffer.
        switch (pose_projection.shape_type)
        {
        case PSVRShape_Ellipse:
            {
                // For the sphere, its ellipse projection parameters should already
                // be calculated, so we can use those parameters to draw an ellipse.

                //Create cv::ellipse from pose_estimate
                cv::Point ell_center(
                    static_cast<int>(pose_projection.projections[section].shape.ellipse.center.x),
                    static_cast<int>(pose_projection.projections[section].shape.ellipse.center.y));
                cv::Size ell_size(
                    static_cast<int>(pose_projection.projections[section].shape.ellipse.half_x_extent),
                    static_cast<int>(pose_projection.projections[section].shape.ellipse.half_y_extent));

                //Draw ellipse on bgrShmemBuffer
                cv::ellipse(*bgrShmemBuffer,
                    ell_center,
                    ell_size,
                    pose_projection.projections[section].shape.ellipse.angle,
                    0, 360, cv::Scalar(0, 0, 255));
                cv::drawMarker(*bgrShmemBuffer, ell_center, cv::Scalar(0, 0, 255), 0,
                    (ell_size.height < ell_size.width) ? ell_size.height * 2 : ell_size.width * 2);
            } break;
        case PSVRShape_LightBar:
            {
                int prev_point_index;

                prev_point_index = QUAD_POINT_COUNT - 1;
                for (int point_index = 0; point_index < QUAD_POINT_COUNT; ++point_index)
                {
                    cv::Point pt1(
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.quad[prev_point_index].x),
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.quad[prev_point_index].y));
                    cv::Point pt2(
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.quad[point_index].x),
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.quad[point_index].y));
                    cv::line(*bgrShmemBuffer, pt1, pt2, cv::Scalar(0, 0, 255));

                    prev_point_index = point_index;
                }

                prev_point_index = TRIANGLE_POINT_COUNT - 1;
                for (int point_index = 0; point_index < TRIANGLE_POINT_COUNT; ++point_index)
                {
                    cv::Point pt1(
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.triangle[prev_point_index].x),
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.triangle[prev_point_index].y));
                    cv::Point pt2(
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.triangle[point_index].x),
                        static_cast<int>(pose_projection.projections[section].shape.lightbar.triangle[point_index].y));
                    cv::line(*bgrShmemBuffer, pt1, pt2, cv::Scalar(0, 0, 255));

                    prev_point_index = point_index;
                }
                
            } break;
        case PSVRShape_PointCloud:
            {
                for (int point_index = 0; point_index < pose_projection.projections[section].shape.pointcloud.point_count; ++point_index)
                {
                    cv::Point pt(
                        static_cast<int>(pose_projection.projections[section].shape.pointcloud.points[point_index].x),
                        static_cast<int>(pose_projection.projections[section].shape.pointcloud.points[point_index].y));
                    cv::drawMarker(*bgrShmemBuffer, pt, cv::Scalar(0, 0, 255));
                }
            } break;
        default:
            assert(false && "unreachable");
            break;
        }		
    }

    PSVRVideoFrameSection section;

    int frameWidth;
    int frameHeight;

    cv::Mat *bgrBuffer; // source video frame
    cv::Mat *bgrShmemBuffer; //Frame onto which we draw debug lines, and transmit via shared mem.
    cv::Mat bgrROI;
    cv::Mat *hsvBuffer; // source frame converted to HSV color space
    cv::Mat hsvROI;
    cv::Mat *gsLowerBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat gsLowerROI;
    cv::Mat *gsUpperBuffer; // HSV image clamped by HSV range into grayscale mask
    cv::Mat gsUpperROI;
    cv::Mat *maskedBuffer; // bgr image ANDed together with grayscale mask
    OpenCVBGRToHSVMapper *bgr2hsv; // Used to convert an rgb image to an hsv image
};

// -- Utility Methods -----
static glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device);
static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device);
static void computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device,
                                                      cv::Matx34f &extrinsicOut);
cv::Mat cvDistCoeffs = cv::Mat(4, 1, cv::DataType<float>::type, 0.f);
static void computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device,
                                               PSVRVideoFrameSection section,
                                               cv::Matx33f &intrinsicOut,
                                               cv::Matx<float, 5, 1> &distortionOut);
static bool computeOpenCVCameraRectification(const ITrackerInterface *tracker_device,
                                               PSVRVideoFrameSection section,
                                               cv::Matx33d &rotationOut,
                                               cv::Matx34d &projectionOut);
static bool computeTrackerRelativePointCloudContourPoseInSection(
    const ITrackerInterface *tracker_device,
    const PSVRTrackingShape *tracking_shape,
	const PSVRVideoFrameSection section,
    const t_opencv_float_contour_list &opencv_contours,
    const PSVRPosef *tracker_relative_pose_guess,
    HMDOpticalPoseEstimation *out_pose_estimate);
static cv::Rect2i computeTrackerROIForPoseProjection(
    const bool disabled_roi,
    const ServerTrackerView *tracker,
    const PSVRVideoFrameSection section,
    const IPoseFilter* pose_filter,
    const PSVRTrackingProjection *prior_tracking_projection,
    const PSVRTrackingShape *tracking_shape);
static bool computeBestFitTriangleForContour(
    const t_opencv_float_contour &opencv_contour,
    cv::Point2f &out_triangle_top,
    cv::Point2f &out_triangle_bottom_left,
    cv::Point2f &out_triangle_bottom_right);
static bool computeBestFitQuadForContour(
    const t_opencv_float_contour &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    cv::Point2f &top_right,
    cv::Point2f &top_left,
    cv::Point2f &bottom_left,
    cv::Point2f &bottom_right);
static void commonDeviceOrientationToOpenCVRodrigues(
    const PSVRQuatf &orientation,
    cv::Mat &rvec);
static void openCVRodriguesToAngleAxis(
    const cv::Mat &rvec,
    float &axis_x, float &axis_y, float &axis_z, float &radians);
static void angleAxisVectorToEulerAngles(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    float &yaw, float &pitch, float &roll);
static void angleAxisVectorToCommonDeviceOrientation(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    PSVRQuatf &orientation);

//-- public implementation -----
ServerTrackerView::ServerTrackerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_shared_memory_accesor(nullptr)
    , m_shared_memory_video_stream_count(0)
    , m_device(nullptr)
{
    Utility::format_string(m_shared_memory_name, sizeof(m_shared_memory_name), "tracker_view_%d", device_id);
    for (int i = 0; i < MAX_PROJECTION_COUNT; ++i)
    {
        m_opencv_buffer_state[i]= nullptr;
    }
}

ServerTrackerView::~ServerTrackerView()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
    }

    for (int i = 0; i < MAX_PROJECTION_COUNT; ++i)
    {
        if (m_opencv_buffer_state[i] != nullptr)
        {
            delete m_opencv_buffer_state[i];
        }
    }

    if (m_device != nullptr)
    {
        delete m_device;
    }
}

CommonDeviceState::eDeviceType
ServerTrackerView::getTrackerDeviceType() const
{
    return m_device->getDeviceType();
}

ITrackerInterface::eDriverType 
ServerTrackerView::getTrackerDriverType() const
{
    return m_device->getDriverType();
}

bool 
ServerTrackerView::getIsStereoCamera() const
{
    return m_device->getIsStereoCamera();
}

std::string
ServerTrackerView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

std::string 
ServerTrackerView::getSharedMemoryStreamName() const
{
    return std::string(m_shared_memory_name);
}

const SharedVideoFrameBuffer *
ServerTrackerView::getSharedVideoFrameBuffer() const
{
    return m_shared_memory_accesor;
}

bool ServerTrackerView::open(const class DeviceEnumerator *enumerator)
{
    bool bSuccess = ServerDeviceView::open(enumerator);

    if (bSuccess)
    {
        // Allocate the shared 
        reallocate_shared_memory();

        // Allocate the open cv buffers used for tracking filtering
        reallocate_opencv_buffer_state();
    }

    return bSuccess;
}

void ServerTrackerView::close()
{
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }

    for (int i = 0; i < MAX_PROJECTION_COUNT; ++i)
    {
        if (m_opencv_buffer_state[i] != nullptr)
        {
            delete m_opencv_buffer_state[i];
            m_opencv_buffer_state[i]= nullptr;
        }
    }

    ServerDeviceView::close();
}

void ServerTrackerView::startSharedMemoryVideoStream()
{
    ++m_shared_memory_video_stream_count;
}

void ServerTrackerView::stopSharedMemoryVideoStream()
{
    assert(m_shared_memory_video_stream_count > 0);
    --m_shared_memory_video_stream_count;
}

bool ServerTrackerView::poll()
{
    bool bSuccess = ServerDeviceView::poll();

    if (bSuccess && m_device != nullptr)
    {
        if (m_device->getIsStereoCamera())
        {
            const unsigned char *left_buffer = m_device->getVideoFrameBuffer(PSVRVideoFrameSection_Left);
            const unsigned char *right_buffer = m_device->getVideoFrameBuffer(PSVRVideoFrameSection_Right);

            if (left_buffer != nullptr && right_buffer != nullptr)
            {
                // Cache the left raw video frame
                if (m_opencv_buffer_state[PSVRVideoFrameSection_Left] != nullptr)
                {
                    m_opencv_buffer_state[PSVRVideoFrameSection_Left]->writeVideoFrame(left_buffer);
                }

                // Cache the right raw video frame
                if (m_opencv_buffer_state[PSVRVideoFrameSection_Right] != nullptr)
                {
                    m_opencv_buffer_state[PSVRVideoFrameSection_Right]->writeVideoFrame(right_buffer);
                }
            }
        }
        else
        {
            const unsigned char *buffer = m_device->getVideoFrameBuffer(PSVRVideoFrameSection_Primary);

            if (buffer != nullptr)
            {
                // Cache the raw video frame
                if (m_opencv_buffer_state[PSVRVideoFrameSection_Primary] != nullptr)
                {
                    m_opencv_buffer_state[PSVRVideoFrameSection_Primary]->writeVideoFrame(buffer);
                }
            }
        }
    }

    return bSuccess;
}

void ServerTrackerView::reallocate_shared_memory()
{
    int width, height, stride;

    // close buffer
    if (m_shared_memory_accesor != nullptr)
    {
        delete m_shared_memory_accesor;
        m_shared_memory_accesor = nullptr;
    }

    // Query the video frame first so that we know how big to make the buffer
    if (m_device->getVideoFrameDimensions(&width, &height, &stride))
    {
        int section_count= m_device->getIsStereoCamera() ? 2 : 1;

        assert(m_shared_memory_accesor == nullptr);
        m_shared_memory_accesor = new SharedVideoFrameBuffer();

        if (!m_shared_memory_accesor->initialize(m_shared_memory_name, width, height, stride, section_count))
        {
            delete m_shared_memory_accesor;
            m_shared_memory_accesor = nullptr;

            PSVR_LOG_ERROR("ServerTrackerView::reallocate_shared_memory()") << "Failed to allocated shared memory: " << m_shared_memory_name;
        }
    }
}

void ServerTrackerView::reallocate_opencv_buffer_state()
{
    // Delete any existing opencv buffers
    for (int i = 0; i < MAX_PROJECTION_COUNT; ++i)
    {
        if (m_opencv_buffer_state[i] != nullptr)
        {
            delete m_opencv_buffer_state[i];
            m_opencv_buffer_state[i]= nullptr;
        }
    }

    // Allocate the OpenCV scratch buffers used for finding tracking blobs
    if (m_device->getIsStereoCamera())
    {
        m_opencv_buffer_state[PSVRVideoFrameSection_Left] = 
            new OpenCVBufferState(m_device, PSVRVideoFrameSection_Left);
        m_opencv_buffer_state[PSVRVideoFrameSection_Right] = 
            new OpenCVBufferState(m_device, PSVRVideoFrameSection_Right);
    }
    else
    {
        m_opencv_buffer_state[PSVRVideoFrameSection_Primary] =
            new OpenCVBufferState(m_device, PSVRVideoFrameSection_Primary);
    }
}

bool ServerTrackerView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    m_device= ServerTrackerView::allocate_tracker_interface(enumerator);

    return m_device != nullptr;
}

ITrackerInterface *ServerTrackerView::allocate_tracker_interface(const class DeviceEnumerator *enumerator)
{
    ITrackerInterface *tracker_interface= nullptr;

    switch (enumerator->get_device_type())
    {
    case CommonDeviceState::PS3EYE:
        {
            tracker_interface = new PS3EyeTracker();
        } break;
    case CommonDeviceState::VirtualStereoCamera:
        {
            tracker_interface = new VirtualStereoTracker();
        } break;
    default:
        break;
    }

    return tracker_interface;
}

void ServerTrackerView::free_device_interface()
{
    if (m_device != nullptr)
    {
        delete m_device;  // Deleting abstract object should be OK because
        // this (ServerDeviceView) is abstract as well.
        // All non-abstract children will have non-abstract types
        // for m_device.
        m_device = nullptr;
    }
}

void ServerTrackerView::publish_device_data_frame()
{
    // Copy the video frame to shared memory (if requested)
    if (m_shared_memory_accesor != nullptr && m_shared_memory_video_stream_count > 0)
    {
        if (m_device->getIsStereoCamera())
        {
            m_shared_memory_accesor->writeVideoFrame(
                PSVRVideoFrameSection_Left, 
                m_opencv_buffer_state[PSVRVideoFrameSection_Left]->bgrShmemBuffer->data);
            m_shared_memory_accesor->writeVideoFrame(
                PSVRVideoFrameSection_Right, 
                m_opencv_buffer_state[PSVRVideoFrameSection_Right]->bgrShmemBuffer->data);
        }
        else
        {
            m_shared_memory_accesor->writeVideoFrame(
                PSVRVideoFrameSection_Primary,
                m_opencv_buffer_state[PSVRVideoFrameSection_Primary]->bgrShmemBuffer->data);
        }
    }
    
    // Tell the server request handler we want to send out tracker updates.
    // This will call generate_tracker_data_frame_for_stream for each listening connection.
    ServiceRequestHandler::get_instance()->publish_tracker_data_frame(
        this, &ServerTrackerView::generate_tracker_data_frame_for_stream);
}

void ServerTrackerView::generate_tracker_data_frame_for_stream(
    const ServerTrackerView *tracker_view,
    const struct TrackerStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    TrackerDataPacket *tracker_data_frame = &data_frame.device.tracker_data_packet;

    tracker_data_frame->tracker_id= tracker_view->getDeviceID();
    tracker_data_frame->sequence_num= tracker_view->m_sequence_number;
    tracker_data_frame->is_connected= tracker_view->getIsOpen();

    switch (tracker_view->getTrackerDeviceType())
    {
    case CommonDeviceState::PS3EYE:
        {
            //TODO: PS3EYE tracker location
        } break;
    case CommonDeviceState::VirtualStereoCamera:
        {
            //TODO: PS3EYE tracker location
        } break;
    default:
        assert(0 && "Unhandled Tracker type");
    }

    data_frame.device_category= DeviceCategory_TRACKER;
}

void ServerTrackerView::loadSettings()
{
    m_device->loadSettings();
}

void ServerTrackerView::saveSettings()
{
    m_device->saveSettings();
}

double ServerTrackerView::getFrameWidth() const
{
    return m_device->getFrameWidth();
}

void ServerTrackerView::setFrameWidth(double value, bool bUpdateConfig)
{
    if (value == m_device->getFrameWidth()) return;

    // change frame width
    m_device->setFrameWidth(value, bUpdateConfig);

    // Resize the shared memory and opencv buffers
    reallocate_shared_memory();
    reallocate_opencv_buffer_state();
}

double ServerTrackerView::getFrameHeight() const
{
    return m_device->getFrameHeight();
}

void ServerTrackerView::setFrameHeight(double value, bool bUpdateConfig)
{
    if (value == m_device->getFrameHeight()) return;

    // change frame height
    m_device->setFrameHeight(value, bUpdateConfig);

    // Resize the shared memory and opencv buffers
    reallocate_shared_memory();
    reallocate_opencv_buffer_state();

}

double ServerTrackerView::getFrameRate() const
{
    return m_device->getFrameRate();
}

void ServerTrackerView::setFrameRate(double value, bool bUpdateConfig)
{
    m_device->setFrameRate(value, bUpdateConfig);
}

double ServerTrackerView::getExposure() const
{
    return m_device->getExposure();
}

void ServerTrackerView::setExposure(double value, bool bUpdateConfig)
{
    m_device->setExposure(value, bUpdateConfig);
}

double ServerTrackerView::getGain() const
{
    return m_device->getGain();
}

void ServerTrackerView::setGain(double value, bool bUpdateConfig)
{
    m_device->setGain(value, bUpdateConfig);
}

void ServerTrackerView::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    m_device->getCameraIntrinsics(out_tracker_intrinsics);
}

void ServerTrackerView::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    m_device->setCameraIntrinsics(tracker_intrinsics);
}

PSVRPosef ServerTrackerView::getTrackerPose() const
{
    return m_device->getTrackerPose();
}

void ServerTrackerView::setTrackerPose(
    const PSVRPosef *pose)
{
    m_device->setTrackerPose(pose);
}

void ServerTrackerView::getPixelDimensions(float &outWidth, float &outHeight) const
{
    int pixelWidth, pixelHeight;

    m_device->getVideoFrameDimensions(&pixelWidth, &pixelHeight, nullptr);

    outWidth = static_cast<float>(pixelWidth);
    outHeight = static_cast<float>(pixelHeight);
}

void ServerTrackerView::getFOV(float &outHFOV, float &outVFOV) const
{
    m_device->getFOV(outHFOV, outVFOV);
}

void ServerTrackerView::getZRange(float &outZNear, float &outZFar) const
{
    m_device->getZRange(outZNear, outZFar);
}

void ServerTrackerView::gatherTrackingColorPresets(
    const class ServerHMDView *hmd,
    PSVRClientTrackerSettings* settings) const
{
    std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

    return m_device->gatherTrackingColorPresets(hmd_id, settings);
}

void ServerTrackerView::setHMDTrackingColorPreset(
    const class ServerHMDView *hmd,
    PSVRTrackingColorType color,
    const PSVR_HSVColorRange *preset)
{
    std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

    return m_device->setTrackingColorPreset(hmd_id, color, preset);
}

void ServerTrackerView::getHMDTrackingColorPreset(
    const class ServerHMDView *hmd,
    PSVRTrackingColorType color,
    PSVR_HSVColorRange *out_preset) const
{
    std::string hmd_id = (hmd != nullptr) ? hmd->getConfigIdentifier() : "";

    return m_device->getTrackingColorPreset(hmd_id, color, out_preset);
}

bool ServerTrackerView::computeProjectionForHMD(
    const class ServerHMDView* tracked_hmd,
    const PSVRTrackingShape *tracking_shape,
    struct HMDOpticalPoseEstimation *out_pose_estimate)
{
    bool bSuccess= false;

    if (m_device->getIsStereoCamera())
    {
        bool bLeftSuccess=
            computeProjectionForHmdInSection(
                tracked_hmd,
                tracking_shape,
                PSVRVideoFrameSection_Left,
                out_pose_estimate);
        bool bRightSuccess= true;
            computeProjectionForHmdInSection(
                tracked_hmd,
                tracking_shape,
                PSVRVideoFrameSection_Right,
                out_pose_estimate);

        if (bLeftSuccess && bRightSuccess)
        {
            out_pose_estimate->projection.projection_count= STEREO_PROJECTION_COUNT;

            //TODO: Compute pose estimate using stereo triangulation
	        //4) Find corresponding points on 2d curves using fundamental matrix
	        //5) Triangulate a single 3d curve
	        //6) Compute normals on curve
	        //7) Use SICP::point_to_plane to align model points to triangulated curve points
		       // a. Use previous frames best fit model as a starting point
	        //8) Use RigidMotionEstimator::point_to_point(X, U) to find the transform that best aligned the model points with the curve points
            bSuccess= true;
        }
    }
    else
    {
        out_pose_estimate->projection.projection_count= MONO_PROJECTION_COUNT;

        bSuccess=
            computeProjectionForHmdInSection(
                tracked_hmd,
                tracking_shape,
                PSVRVideoFrameSection_Primary,
                out_pose_estimate);
    }

    return bSuccess;
}

bool
ServerTrackerView::computeProjectionForHmdInSection(
    const ServerHMDView* tracked_hmd,
    const PSVRTrackingShape *tracking_shape,
    const PSVRVideoFrameSection section,
    HMDOpticalPoseEstimation *out_pose_estimate)
{
    bool bSuccess = true;

    // Get the HSV filter used to find the tracking blob
    PSVR_HSVColorRange hsvColorRange;
    if (bSuccess)
    {
        PSVRTrackingColorType tracked_color_id = tracked_hmd->getTrackingColorID();

        if (tracked_color_id != PSVRTrackingColorType_INVALID)
        {
            getHMDTrackingColorPreset(tracked_hmd, tracked_color_id, &hsvColorRange);
        }
        else
        {
            bSuccess = false;
        }
    }
    
    // Compute a region of interest in the tracker buffer around where we expect to find the tracking shape
    const TrackerManagerConfig &trackerMgrConfig= DeviceManager::getInstance()->m_tracker_manager->getConfig();
    const bool bRoiDisabled = tracked_hmd->getIsROIDisabled() || trackerMgrConfig.disable_roi;

    const HMDOpticalPoseEstimation *priorPoseEst= 
        tracked_hmd->getTrackerPoseEstimate(this->getDeviceID());
    const bool bIsTracking = priorPoseEst->bCurrentlyTracking;

    cv::Rect2i ROI = computeTrackerROIForPoseProjection(
        bRoiDisabled,
        this,
        section,
        bIsTracking ? tracked_hmd->getPoseFilter() : nullptr,
        bIsTracking ? &priorPoseEst->projection : nullptr,
        tracking_shape);
    m_opencv_buffer_state[section]->applyROI(ROI);

    // Find the N best contours associated with the HMD
    t_opencv_int_contour_list biggest_contours;
    std::vector<double> contour_areas;
    if (bSuccess)
    {
        bSuccess = 
            m_opencv_buffer_state[section]->computeBiggestNContours(
                hsvColorRange, biggest_contours, contour_areas, MAX_PROJECTION_COUNT);
    }

    // Compute the tracker relative 3d position of the controller from the contour
    if (bSuccess)
    {
        cv::Matx33f camera_matrix;
        cv::Matx<float, 5, 1> distortions;
        cv::Matx33d rectification_rotation;
        cv::Matx34d rectification_projection; 
        computeOpenCVCameraIntrinsicMatrix(m_device, section, camera_matrix, distortions);
        bool valid_rectification= computeOpenCVCameraRectification(m_device, section, rectification_rotation, rectification_projection);

        switch (tracking_shape->shape_type)
        {
        // For the sphere projection we can go ahead and compute the full pose estimation now
        case PSVRTrackingShape_Sphere:
            {
                // Compute the convex hull of the contour
                t_opencv_int_contour convex_contour;
                cv::convexHull(biggest_contours[0], convex_contour);
                m_opencv_buffer_state[section]->draw_contour(convex_contour);

                // Convert integer to float
                t_opencv_float_contour convex_contour_f;
                cv::Mat(convex_contour).convertTo(convex_contour_f, cv::Mat(convex_contour_f).type());

                // Undistort points
                t_opencv_float_contour undistorted_contour;  //destination for undistorted contour
                if (valid_rectification)
                {
                    cv::undistortPoints(convex_contour_f, undistorted_contour,
                                        camera_matrix,
                                        distortions,
                                        rectification_rotation,
                                        rectification_projection);
                }
                else 
                {
                    cv::undistortPoints(convex_contour_f, undistorted_contour,
                                        camera_matrix,
                                        distortions,
                                        cv::noArray(),
                                        camera_matrix);
                }
                // Note: if we omit the last two arguments, then
                // undistort_contour points are in 'normalized' space.
                // i.e., they are relative to their F_PX,F_PY
                
                // Compute the sphere center AND the projected ellipse
                Eigen::Vector3f sphere_center;
                EigenFitEllipse ellipse_projection;

                std::vector<Eigen::Vector2f> eigen_contour;
                std::for_each(undistorted_contour.begin(),
                              undistorted_contour.end(),
                              [&eigen_contour](const cv::Point2f& p) {
                                  eigen_contour.push_back(Eigen::Vector2f(p.x, p.y));
                              });
                eigen_alignment_fit_focal_cone_to_sphere(eigen_contour.data(),
                                                         static_cast<int>(eigen_contour.size()),
                                                         tracking_shape->shape.sphere.radius,
                                                         1, //I was expecting this to be -1. Is it +1 because we're using -F_PY?
                                                         &sphere_center,
                                                         &ellipse_projection);
                
                if (ellipse_projection.area > k_real_epsilon)
                {
                    //Save the optically-estimate 3D pose.
                    out_pose_estimate->position_cm = eigen_vector3f_to_PSVR_vector3f(sphere_center);
                    out_pose_estimate->bCurrentlyTracking = true;
                    // Not possible to get an orientation off of a sphere
                    out_pose_estimate->orientation= *k_PSVR_quaternion_identity;
                    out_pose_estimate->bOrientationValid = false;

                    // Save off the projection of the sphere (an ellipse)
                    out_pose_estimate->projection.projections[section].shape.ellipse.angle = ellipse_projection.angle;
                    out_pose_estimate->projection.projections[section].screen_area= ellipse_projection.area;
                    //The ellipse projection is still in normalized space.
                    //i.e., it is a 2-dimensional ellipse floating somewhere.
                    //We must reproject it onto the camera.
                    //TODO: Use opencv's project points instead of manual way below
                    //because it will account for distortion, at least for the center point.
                    out_pose_estimate->projection.shape_type = PSVRShape_Ellipse;
                    out_pose_estimate->projection.projections[section].shape.ellipse.center= {
                        ellipse_projection.center.x()*camera_matrix.val[0] + camera_matrix.val[2],
                        ellipse_projection.center.y()*camera_matrix.val[4] + camera_matrix.val[5]};
                    out_pose_estimate->projection.projections[section].shape.ellipse.half_x_extent = ellipse_projection.extents.x()*camera_matrix.val[0];
                    out_pose_estimate->projection.projections[section].shape.ellipse.half_y_extent = ellipse_projection.extents.y()*camera_matrix.val[0];
                    out_pose_estimate->projection.projections[section].screen_area=
                        k_real_pi*out_pose_estimate->projection.projections[section].shape.ellipse.half_x_extent
                        *out_pose_estimate->projection.projections[section].shape.ellipse.half_y_extent;
                
                    //Draw results onto m_opencv_buffer_state
                    m_opencv_buffer_state[section]->draw_pose_projection(out_pose_estimate->projection);

                    bSuccess = true;
                }
            } break;
        case PSVRTrackingShape_PointCloud:
            {
                const HMDOpticalPoseEstimation *prior_post_est= tracked_hmd->getTrackerPoseEstimate(getDeviceID());
                PSVRPosef tracker_pose_guess= {prior_post_est->position_cm, prior_post_est->orientation};

                // Undistort the source contours
                t_opencv_float_contour_list undistorted_contours;
                for (auto it = biggest_contours.begin(); it != biggest_contours.end(); ++it)
                {
                    // Draw the source contour
                    m_opencv_buffer_state[section]->draw_contour(*it);

                    // Convert integer contour to float
                    t_opencv_float_contour biggest_contour_f;
                    cv::Mat(*it).convertTo(biggest_contour_f, cv::Mat(biggest_contour_f).type());

                    // Compute an undistorted version of the contour
                    t_opencv_float_contour undistorted_contour;
                    if (valid_rectification)
                    {
                        cv::undistortPoints(biggest_contour_f, undistorted_contour,
                                            camera_matrix,
                                            distortions,
                                            rectification_rotation,
                                            rectification_projection);
                    }
                    else 
                    {
                        cv::undistortPoints(biggest_contour_f, undistorted_contour,
                                            camera_matrix,
                                            distortions,
                                            cv::noArray(),
                                            camera_matrix);
                    }

                    undistorted_contours.push_back(undistorted_contour);
                }

                bSuccess =
                    computeTrackerRelativePointCloudContourPoseInSection(
                        m_device,
                        tracking_shape,
                        section,
                        undistorted_contours,
                        prior_post_est->bCurrentlyTracking ? &tracker_pose_guess : nullptr,
                        out_pose_estimate);

                //Draw results onto m_opencv_buffer_state
                m_opencv_buffer_state[section]->draw_pose_projection(out_pose_estimate->projection);
            } break;
        default:
            assert(0 && "Unreachable");
            break;
        }
    }

    return bSuccess;
}

PSVRVector3f
ServerTrackerView::computeWorldPosition(
    const PSVRVector3f *tracker_relative_position) const
{
    const glm::vec4 rel_pos(tracker_relative_position->x, tracker_relative_position->y, tracker_relative_position->z, 1.f);
    const glm::mat4 cameraTransform= computeGLMCameraTransformMatrix(m_device);
    const glm::vec4 world_pos = cameraTransform * rel_pos;
    
    PSVRVector3f result= glm_vec3_to_PSVR_vector3f(world_pos);

    return result;
}

PSVRQuatf
ServerTrackerView::computeWorldOrientation(
    const PSVRQuatf *tracker_relative_orientation) const
{
    // Compute a rotations that rotates from +X to global "forward"
    const TrackerManagerConfig &cfg = DeviceManager::getInstance()->m_tracker_manager->getConfig();
    const float global_forward_yaw_radians = cfg.global_forward_degrees*k_degrees_to_radians;
    const glm::quat global_forward_quat= glm::quat(glm::vec3(0.f, global_forward_yaw_radians, 0.f));
    
    const glm::quat rel_orientation(
        tracker_relative_orientation->w,
        tracker_relative_orientation->x,
        tracker_relative_orientation->y,
        tracker_relative_orientation->z);    
    const glm::quat camera_quat= computeGLMCameraTransformQuaternion(m_device);
    const glm::quat world_quat = global_forward_quat * camera_quat * rel_orientation;
    
    PSVRQuatf result;
    result.w= world_quat.w;
    result.x= world_quat.x;
    result.y= world_quat.y;
    result.z= world_quat.z;

    return result;
}

PSVRVector3f 
ServerTrackerView::computeTrackerPosition(
    const PSVRVector3f *world_relative_position) const
{
    const glm::vec4 world_pos(world_relative_position->x, world_relative_position->y, world_relative_position->z, 1.f);
    const glm::mat4 invCameraTransform= glm::inverse(computeGLMCameraTransformMatrix(m_device));
    const glm::vec4 rel_pos = invCameraTransform * world_pos;    
    const PSVRVector3f result= glm_vec3_to_PSVR_vector3f(rel_pos);

    return result;
}

PSVRQuatf 
ServerTrackerView::computeTrackerOrientation(
    const PSVRQuatf *world_relative_orientation) const
{
    const glm::quat world_orientation(
        world_relative_orientation->w,
        world_relative_orientation->x,
        world_relative_orientation->y,
        world_relative_orientation->z);    
    const glm::quat camera_inv_quat= glm::conjugate(computeGLMCameraTransformQuaternion(m_device));
    // combined_rotation = second_rotation * first_rotation;
    const glm::quat rel_quat = camera_inv_quat * world_orientation;
    
    PSVRQuatf result;
    result.w= rel_quat.w;
    result.x= rel_quat.x;
    result.y= rel_quat.y;
    result.z= rel_quat.z;

    return result;
}

std::vector<PSVRVector2f>
ServerTrackerView::projectTrackerRelativePositions(
	const PSVRVideoFrameSection section,
	const std::vector<PSVRVector3f> &objectPositions) const
{
    cv::Matx33f camera_matrix;
    cv::Matx<float, 5, 1> distortions;
    computeOpenCVCameraIntrinsicMatrix(m_device, section, camera_matrix, distortions);
    
    // Use the identity transform for tracker relative positions
    cv::Mat rvec(3, 1, cv::DataType<double>::type, double(0));
    cv::Mat tvec(3, 1, cv::DataType<double>::type, double(0));
    
    std::vector<cv::Point3f> cvObjectPoints;
    size_t i;
    for (i=0; i<objectPositions.size(); ++i) {
        cvObjectPoints.push_back(cv::Point3f(objectPositions[i].x,
                                             objectPositions[i].y,
                                             objectPositions[i].z));
    }
    
    // Projected point
    std::vector<cv::Point2f> projectedPoints;
    cv::projectPoints(cvObjectPoints,
                      rvec,
                      tvec,
                      camera_matrix,
                      distortions,
                      projectedPoints);
    
    std::vector<PSVRVector2f> screenLocations;
    for (i=0; i<projectedPoints.size(); ++i) {
        PSVRVector2f thisloc= {projectedPoints[i].x, projectedPoints[i].y};
        screenLocations.push_back(thisloc);
    }
    
    return screenLocations;
}

PSVRVector2f
ServerTrackerView::projectTrackerRelativePosition(
    const PSVRVideoFrameSection section,
	const PSVRVector3f *trackerRelativePosition) const
{
    std::vector<PSVRVector3f> trp_vec {*trackerRelativePosition};
    PSVRVector2f screenLocation = projectTrackerRelativePositions(section, trp_vec)[0];

    return screenLocation;
}


// -- Tracker Utility Methods -----
static glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device)
{

    const PSVRPosef pose = tracker_device->getTrackerPose();
    const PSVRQuatf &quat = pose.Orientation;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);

    return glm_quat;
}

static glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device)
{

    const PSVRPosef pose = tracker_device->getTrackerPose();
    const PSVRQuatf &quat = pose.Orientation;
    const PSVRVector3f &pos = pose.Position;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);
    const glm::vec3 glm_pos(pos.x, pos.y, pos.z);
    const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);

    return glm_camera_xform;
}

static void computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device,
                                               cv::Matx34f &out)
{
    // Extrinsic matrix is the inverse of the camera pose matrix
    const glm::mat4 glm_camera_xform = computeGLMCameraTransformMatrix(tracker_device);
    const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);

    out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
    out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
    out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];
}

static void computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device,
                                               PSVRVideoFrameSection section,
                                               cv::Matx33f &intrinsicOut,
                                               cv::Matx<float, 5, 1> &distortionOut)
{
    PSVRTrackerIntrinsics tracker_intrinsics;
    tracker_device->getCameraIntrinsics(tracker_intrinsics);

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

static bool computeOpenCVCameraRectification(const ITrackerInterface *tracker_device,
                                               PSVRVideoFrameSection section,
                                               cv::Matx33d &rotationOut,
                                               cv::Matx34d &projectionOut)
{
    PSVRTrackerIntrinsics tracker_intrinsics;
    tracker_device->getCameraIntrinsics(tracker_intrinsics);

    PSVRMatrix3d *rectification_rotation= nullptr;
    PSVRMatrix34d *rectification_projection= nullptr;

    if (tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS)
    {
        if (section == PSVRVideoFrameSection_Left)
        {
            rectification_rotation= &tracker_intrinsics.intrinsics.stereo.left_rectification_rotation;
            rectification_projection= &tracker_intrinsics.intrinsics.stereo.left_rectification_projection;
        }
        else if (section == PSVRVideoFrameSection_Right)
        {
            rectification_rotation= &tracker_intrinsics.intrinsics.stereo.right_rectification_rotation;
            rectification_projection= &tracker_intrinsics.intrinsics.stereo.right_rectification_projection;
        }
    }

    if (rectification_rotation != nullptr && rectification_projection != nullptr)
    {
        double *r= (double *)rectification_rotation->m;
        double *p= (double *)rectification_projection->m;
   
        rotationOut(0, 0)= (float)r[0]; rotationOut(0, 1)= (float)r[1]; rotationOut(0, 2)= (float)r[2];
        rotationOut(1, 0)= (float)r[3]; rotationOut(1, 1)= (float)r[4]; rotationOut(1, 2)= (float)r[5]; 
        rotationOut(2, 0)= (float)r[6]; rotationOut(2, 1)= (float)r[7]; rotationOut(2, 2)= (float)r[8];

        projectionOut(0, 0)= (float)p[0]; projectionOut(0, 1)= (float)p[1]; projectionOut(0, 2)= (float)p[2]; projectionOut(0, 3)= (float)p[3];
        projectionOut(1, 0)= (float)p[4]; projectionOut(1, 1)= (float)p[5]; projectionOut(1, 2)= (float)p[6]; projectionOut(1, 3)= (float)p[7]; 
        projectionOut(2, 0)= (float)p[8]; projectionOut(2, 1)= (float)p[9]; projectionOut(2, 2)= (float)p[10]; projectionOut(2, 3)= (float)p[11];

        return true;
    }
    else
    {
        return false;
    }
}

static bool computeTrackerRelativeLightBarProjection(
    const PSVRTrackingShape *tracking_shape,
    const PSVRVideoFrameSection section,
    const t_opencv_float_contour &opencv_contour,
    PSVRTrackingProjection *out_projection)
{
    assert(tracking_shape->shape_type == PSVRTrackingShape_LightBar);

    bool bValidTrackerProjection= true;
    float projectionArea= 0.f;
    std::vector<cv::Point2f> cvImagePoints;
    {
        cv::Point2f tri_top, tri_bottom_left, tri_bottom_right;
        cv::Point2f quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right;

        // Create a best fit triangle around the contour
        bValidTrackerProjection= computeBestFitTriangleForContour(
            opencv_contour, 
            tri_top, tri_bottom_left, tri_bottom_right);

        // Also create a best fit quad around the contour
        // Use the best fit triangle to define the orientation
        if (bValidTrackerProjection)
        {
            // Use the triangle to define an up and a right direction
            const cv::Point2f up_hint= tri_top - 0.5f*(tri_bottom_left + tri_bottom_right);
            const cv::Point2f right_hint= tri_bottom_right - tri_bottom_left;

            bValidTrackerProjection= computeBestFitQuadForContour(
                opencv_contour, 
                up_hint, right_hint, 
                quad_top_right, quad_top_left, quad_bottom_left, quad_bottom_right);
        }

        if (bValidTrackerProjection)
        {
            // In practice the best fit triangle top is a bit noisy.
            // Since it should be at the midpoint of the top of the quad we use that instead.
            tri_top= 0.5f*(quad_top_right + quad_top_left);

            // Put the image points in corresponding order with cvObjectPoints
            cvImagePoints.push_back(tri_bottom_right);
            cvImagePoints.push_back(tri_bottom_left);
            cvImagePoints.push_back(tri_top);
            cvImagePoints.push_back(quad_top_right);
            cvImagePoints.push_back(quad_top_left);
            cvImagePoints.push_back(quad_bottom_left);
            cvImagePoints.push_back(quad_bottom_right);

            // The projection area is the size of the best fit quad
            projectionArea= 
                static_cast<float>(
                    cv::norm(quad_bottom_right-quad_bottom_left)
                    *cv::norm(quad_bottom_left-quad_top_left));                   
        }
    }

    // Return the projection of the tracking shape
    if (bValidTrackerProjection)
    {
        out_projection->shape_type = PSVRShape_LightBar;

        for (int vertex_index = 0; vertex_index < 3; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index];

            out_projection->projections[section].shape.lightbar.triangle[vertex_index] = { cvPoint.x, cvPoint.y };
        }

        for (int vertex_index = 0; vertex_index < 4; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index + 3];

            out_projection->projections[section].shape.lightbar.quad[vertex_index] = { cvPoint.x, cvPoint.y };
        }

        out_projection->projections[section].screen_area= projectionArea;
    }

    return bValidTrackerProjection;
}

static bool computeTrackerRelativePointCloudContourPose(
    const ITrackerInterface *tracker_device,
    const PSVRTrackingShape *tracking_shape,
    const PSVRVideoFrameSection section,
    const t_opencv_float_contour_list &opencv_contours,
    const PSVRPosef *tracker_relative_pose_guess,
    HMDOpticalPoseEstimation *out_pose_estimate)
{
    assert(tracking_shape->shape_type == PSVRTrackingShape_PointCloud);

    bool bValidTrackerPose = true;
    float projectionArea = 0.f;

    // Compute centers of mass for the contours
    t_opencv_float_contour cvImagePoints;
    for (auto it = opencv_contours.begin(); it != opencv_contours.end(); ++it)
    {
        cv::Point2f massCenter= computeSafeCenterOfMassForContour<t_opencv_float_contour>(*it);

        cvImagePoints.push_back(massCenter);
    }

    if (cvImagePoints.size() >= 3)
    {
        //###HipsterSloth $TODO Solve the pose using SoftPOSIT
        out_pose_estimate->position_cm= *k_PSVR_float_vector3_zero;
        out_pose_estimate->orientation= *k_PSVR_quaternion_identity;
        out_pose_estimate->bOrientationValid = false;
        bValidTrackerPose = true;
    }

    // Return the projection of the tracking shape
    if (bValidTrackerPose)
    {
        PSVRTrackingProjection *out_projection = &out_pose_estimate->projection;
        const int imagePointCount = static_cast<int>(cvImagePoints.size());

        out_projection->shape_type = PSVRShape_PointCloud;

        for (int vertex_index = 0; vertex_index < imagePointCount; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index];

            out_projection->projections[section].shape.pointcloud.points[vertex_index] = {cvPoint.x, cvPoint.y};
        }

        out_projection->projections[section].shape.pointcloud.point_count = imagePointCount;
        out_projection->projections[section].screen_area = projectionArea;
    }

    return bValidTrackerPose;
}

static bool computeTrackerRelativePointCloudContourPoseInSection(
    const ITrackerInterface *tracker_device,
    const PSVRTrackingShape *tracking_shape,
    const PSVRVideoFrameSection section,
    const t_opencv_float_contour_list &opencv_contours,
    const PSVRPosef *tracker_relative_pose_guess,
    HMDOpticalPoseEstimation *out_pose_estimate)
{
    assert(tracking_shape->shape_type == PSVRTrackingShape_PointCloud);

    bool bValidTrackerPose = true;
    float projectionArea = 0.f;

    // Compute centers of mass for the contours
    t_opencv_float_contour cvImagePoints;
    for (auto it = opencv_contours.begin(); it != opencv_contours.end(); ++it)
    {
        cv::Point2f massCenter= computeSafeCenterOfMassForContour<t_opencv_float_contour>(*it);

        cvImagePoints.push_back(massCenter);
    }

    if (cvImagePoints.size() >= 3)
    {
        //###HipsterSloth $TODO Solve the pose using SoftPOSIT
        out_pose_estimate->position_cm= *k_PSVR_float_vector3_zero;
        out_pose_estimate->orientation= *k_PSVR_quaternion_identity;
        out_pose_estimate->bOrientationValid = false;
        bValidTrackerPose = true;
    }

    // Return the projection of the tracking shape
    if (bValidTrackerPose)
    {
        PSVRTrackingProjection *out_projection = &out_pose_estimate->projection;
        const int imagePointCount = static_cast<int>(cvImagePoints.size());

        out_projection->shape_type = PSVRShape_PointCloud;

        for (int vertex_index = 0; vertex_index < imagePointCount; ++vertex_index)
        {
            const cv::Point2f &cvPoint = cvImagePoints[vertex_index];

            out_projection->projections[section].shape.pointcloud.points[vertex_index] = {cvPoint.x, cvPoint.y};
        }

        out_projection->projections[section].shape.pointcloud.point_count = imagePointCount;
        out_projection->projections[section].screen_area = projectionArea;
    }

    return bValidTrackerPose;
}

static cv::Rect2i computeTrackerROIForPoseProjection(
    const bool roi_disabled,
    const ServerTrackerView *tracker,
    const PSVRVideoFrameSection section,
    const IPoseFilter* pose_filter,
    const PSVRTrackingProjection *prior_tracking_projection,
    const PSVRTrackingShape *tracking_shape)
{
    // Get expected ROI
    // Default to full screen.
    float screenWidth, screenHeight;
    tracker->getPixelDimensions(screenWidth, screenHeight);
    cv::Rect2i ROI(0, 0, static_cast<int>(screenWidth), static_cast<int>(screenHeight));

    //Calculate a more refined ROI.
    //Based on the physical limits of the object's bounding box
    //projected onto the image.
    if (!roi_disabled && pose_filter != nullptr && prior_tracking_projection != nullptr)
    {
        // Get the (predicted) position in world space.
        Eigen::Vector3f position_cm = pose_filter->getPositionCm(0.f); 
        PSVRVector3f world_position_cm= eigen_vector3f_to_PSVR_vector3f(position_cm);

        // Get the (predicted) position in tracker-local space.
        PSVRVector3f tracker_position_cm = tracker->computeTrackerPosition(&world_position_cm);

        // Project the state computed position +/- object extents onto the image.
        PSVRVector3f tl, br;
        switch (tracking_shape->shape_type)
        {
        case PSVRTrackingShape_Sphere:
            {
                // Simply: center - radius, center + radius.
                tl= {tracker_position_cm.x - tracking_shape->shape.sphere.radius,
                    tracker_position_cm.y + tracking_shape->shape.sphere.radius,
                    tracker_position_cm.z};
                br= {tracker_position_cm.x + tracking_shape->shape.sphere.radius,
                    tracker_position_cm.y - tracking_shape->shape.sphere.radius,
                    tracker_position_cm.z};
            } break;

        case PSVRTrackingShape_LightBar:
            {
                // Compute the bounding radius of the lightbar tracking shape
                const auto &shape_tl = tracking_shape->shape.lightbar.quad[QUAD_VERTEX_UPPER_LEFT];
                const auto &shape_br = tracking_shape->shape.lightbar.quad[QUAD_VERTEX_LOWER_RIGHT];
                const PSVRVector3f half_vec = { (shape_tl.x - shape_br.x)*0.5f, (shape_tl.y - shape_br.y)*0.5f, (shape_tl.z - shape_br.z)*0.5f };
                const auto shape_radius = fmaxf(sqrtf(half_vec.x*half_vec.x + half_vec.y*half_vec.y + half_vec.z*half_vec.z), 1.f);

                // Simply: center - shape_radius, center + shape_radius.
                tl= {tracker_position_cm.x - shape_radius,
                    tracker_position_cm.y + shape_radius,
                    tracker_position_cm.z};
                br= {tracker_position_cm.x + shape_radius,
                    tracker_position_cm.y - shape_radius,
                    tracker_position_cm.z};
            } break;

        case PSVRTrackingShape_PointCloud:
            {
                // Compute the bounding radius of the point cloud
                PSVRVector3f shape_tl = tracking_shape->shape.pointcloud.points[0];
                PSVRVector3f shape_br = tracking_shape->shape.pointcloud.points[0];
                for (int point_index = 1; point_index < tracking_shape->shape.pointcloud.point_count; ++point_index)
                {
                    const PSVRVector3f &point = tracking_shape->shape.pointcloud.points[point_index];
                    shape_tl= {fmaxf(shape_tl.x, point.x), fmaxf(shape_tl.y, point.y), fmaxf(shape_tl.z, point.z)};
                    shape_br= {fminf(shape_br.x, point.x), fminf(shape_br.y, point.y), fminf(shape_br.z, point.z)};
                }
                const PSVRVector3f half_vec = { (shape_tl.x - shape_br.x)*0.5f, (shape_tl.y - shape_br.y)*0.5f, (shape_tl.z - shape_br.z)*0.5f };
                const auto shape_radius = fmaxf(sqrtf(half_vec.x*half_vec.x + half_vec.y*half_vec.y + half_vec.z*half_vec.z), 1.f);

                // Simply: center - shape_radius, center + shape_radius.
                tl= {tracker_position_cm.x - shape_radius,
                    tracker_position_cm.y + shape_radius,
                    tracker_position_cm.z};
                br= {tracker_position_cm.x + shape_radius,
                    tracker_position_cm.y - shape_radius,
                    tracker_position_cm.z};
            } break;

        default:
            {
                assert(false && "unreachable");
            } break;
        }

        // Extract the pixel projection center from the previous frame's projection.
        PSVRVector2f projection_pixel_center= *k_PSVR_float_vector2_zero;

        switch (prior_tracking_projection->shape_type)
        {
        case PSVRShape_Ellipse:
            {
                // Use the center of the ellipsoid projection for the ROI area
                projection_pixel_center = prior_tracking_projection->projections[section].shape.ellipse.center;
            } break;

        case PSVRShape_LightBar:
            {
                // Use the center of the quad projection for the ROI area
                const auto proj_tl = prior_tracking_projection->projections[section].shape.lightbar.quad[QUAD_VERTEX_UPPER_LEFT];
                const auto proj_br = prior_tracking_projection->projections[section].shape.lightbar.quad[QUAD_VERTEX_LOWER_RIGHT];

                projection_pixel_center= {0.5f * (proj_tl.x + proj_br.x), 0.5f * (proj_tl.y + proj_br.y)};
            } break;

        case PSVRShape_PointCloud:
            {
                // Compute the centroid of the projection pixels
                for (int point_index = 0; point_index < prior_tracking_projection->projections[section].shape.pointcloud.point_count; ++point_index)
                {
                    const auto &pixel = prior_tracking_projection->projections[section].shape.pointcloud.points[point_index];

                    projection_pixel_center.x += pixel.x;
                    projection_pixel_center.y += pixel.y;
                }
                const float N = static_cast<float>(prior_tracking_projection->projections[section].shape.pointcloud.point_count);
                projection_pixel_center.x /= N;
                projection_pixel_center.y /= N;
            } break;

        default:
            {
                assert(false && "unreachable");
            } break;
        }

        // The center of the ROI is the pixel projection center from last frame
        // The size of the ROI computed by projecting the bounding box 
        {
            std::vector<PSVRVector3f> trps{ tl, br };
            std::vector<PSVRVector2f> screen_locs = tracker->projectTrackerRelativePositions(section, trps);

            const int proj_min_x = static_cast<int>(std::min(screen_locs[0].x, screen_locs[1].x));
            const int proj_max_x = static_cast<int>(std::max(screen_locs[0].x, screen_locs[1].x));
            const int proj_min_y = static_cast<int>(std::min(screen_locs[0].y, screen_locs[1].y));
            const int proj_max_y = static_cast<int>(std::max(screen_locs[0].y, screen_locs[1].y));

            const int proj_width = proj_max_x - proj_min_x;
            const int proj_height = proj_max_y - proj_min_y;

            const cv::Point2i roi_center(static_cast<int>(projection_pixel_center.x), static_cast<int>(projection_pixel_center.y));

            const int safe_proj_width = std::max(proj_width, k_min_roi_size);
            const int safe_proj_height = std::max(proj_height, k_min_roi_size);

            const cv::Point2i roi_top_left = roi_center + cv::Point2i(-safe_proj_width, -safe_proj_height);
            const cv::Size roi_size(2*safe_proj_width, 2*safe_proj_height);

            ROI = cv::Rect2i(roi_top_left, roi_size);
        }
    }

    return ROI;
}

static bool computeBestFitTriangleForContour(
    const t_opencv_float_contour &opencv_contour,
    cv::Point2f &out_triangle_top,
    cv::Point2f &out_triangle_bottom_left,
    cv::Point2f &out_triangle_bottom_right)
{
    // Compute the tightest possible bounding triangle for the given contour
    t_opencv_float_contour cv_min_triangle;

    try
    {
        cv::minEnclosingTriangle(opencv_contour, cv_min_triangle);
    }
    catch( cv::Exception& e )
    {
        PSVR_LOG_INFO("computeBestFitTriangleForContour") << e.what();
        return false;
    }

    if (cv_min_triangle.size() != 3)
    {
        return false;
    }

    cv::Point2f best_fit_origin_01 = (cv_min_triangle[0] + cv_min_triangle[1]) / 2.f;
    cv::Point2f best_fit_origin_12 = (cv_min_triangle[1] + cv_min_triangle[2]) / 2.f;
    cv::Point2f best_fit_origin_20 = (cv_min_triangle[2] + cv_min_triangle[0]) / 2.f;

    t_opencv_float_contour cv_midpoint_triangle;
    cv_midpoint_triangle.push_back(best_fit_origin_01);
    cv_midpoint_triangle.push_back(best_fit_origin_12);
    cv_midpoint_triangle.push_back(best_fit_origin_20);

    // Find the corner closest to the center of mass.
    // This is the bottom of the triangle.
    int topCornerIndex = -1;
    {
        const cv::Point2f massCenter = computeSafeCenterOfMassForContour<t_opencv_float_contour>(opencv_contour);

        double bestDistance = k_real_max;
        for (int cornerIndex = 0; cornerIndex < 3; ++cornerIndex)
        {
            const double testDistance = cv::norm(cv_midpoint_triangle[cornerIndex] - massCenter);

            if (testDistance < bestDistance)
            {
                topCornerIndex = cornerIndex;
                bestDistance = testDistance;
            }
        }
    }

    // Assign the left and right corner indices
    int leftCornerIndex = -1;
    int rightCornerIndex = -1;
    switch (topCornerIndex)
    {
    case 0:
        leftCornerIndex = 1;
        rightCornerIndex = 2;
        break;
    case 1:
        leftCornerIndex = 0;
        rightCornerIndex = 2;
        break;
    case 2:
        leftCornerIndex = 0;
        rightCornerIndex = 1;
        break;
    default:
        assert(0 && "unreachable");
    }

    // Make sure the left and right corners are actually 
    // on the left and right of the triangle
    out_triangle_top = cv_midpoint_triangle[topCornerIndex];
    out_triangle_bottom_left = cv_midpoint_triangle[leftCornerIndex];
    out_triangle_bottom_right = cv_midpoint_triangle[rightCornerIndex];

    const cv::Point2f topToLeft = out_triangle_bottom_left - out_triangle_top;
    const cv::Point2f topToRight = out_triangle_bottom_right - out_triangle_top;

    // Cross product should be positive if sides are correct
    // If not, then swap them.
    if (topToRight.cross(topToLeft) < 0)
    {
        std::swap(out_triangle_bottom_left, out_triangle_bottom_right);
    }

    return true;
}

static bool computeBestFitQuadForContour(
    const t_opencv_float_contour &opencv_contour,
    const cv::Point2f &up_hint, 
    const cv::Point2f &right_hint,
    cv::Point2f &top_right,
    cv::Point2f &top_left,
    cv::Point2f &bottom_left,
    cv::Point2f &bottom_right)
{
    // Compute the tightest possible bounding triangle for the given contour
    cv::RotatedRect cv_min_box= cv::minAreaRect(opencv_contour);

    if (cv_min_box.size.width <= k_real_epsilon || cv_min_box.size.height <= k_real_epsilon)
    {
        return false;
    }

    float half_width, half_height;
    float radians;
    if (cv_min_box.size.width > cv_min_box.size.height)
    {
        half_width= cv_min_box.size.width / 2.f;
        half_height= cv_min_box.size.height / 2.f;
        radians= cv_min_box.angle*k_degrees_to_radians;
    }
    else
    {
        half_width= cv_min_box.size.height / 2.f;
        half_height= cv_min_box.size.width / 2.f;
        radians= (cv_min_box.angle + 90.f)*k_degrees_to_radians;
    }

    cv::Point2f quad_half_right, quad_half_up;
    {
        const float cos_angle= cosf(radians);
        const float sin_angle= sinf(radians);

        quad_half_right.x= half_width*cos_angle;
        quad_half_right.y= half_width*sin_angle;

        quad_half_up.x= -half_height*sin_angle;
        quad_half_up.y= half_height*cos_angle;
    }

    if (quad_half_up.dot(up_hint) < 0)
    {
        // up axis is flipped
        // flip the box vertically
        quad_half_up= -quad_half_up;
    }

    if (quad_half_right.dot(right_hint) < 0)
    {
        // right axis is flipped
        // flip the box horizontally
        quad_half_right= -quad_half_right;
    }

    top_right= cv_min_box.center + quad_half_up + quad_half_right;
    top_left= cv_min_box.center + quad_half_up - quad_half_right;
    bottom_right= cv_min_box.center - quad_half_up + quad_half_right;
    bottom_left= cv_min_box.center - quad_half_up - quad_half_right;

    return true;
}

template<typename t_opencv_contour_type>
cv::Point2f computeSafeCenterOfMassForContour(const t_opencv_contour_type &contour)
{
    cv::Moments mu(cv::moments(contour));
    cv::Point2f massCenter;
        
    // mu.m00 is zero for contours of zero area.
    // Fallback to standard centroid in this case.

    if (!is_double_nearly_zero(mu.m00))
    {
        massCenter= cv::Point2f(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));
    }
    else
    {
        massCenter.x = 0.f;
        massCenter.y = 0.f;

        for (const cv::Point &int_point : contour)
        {
            massCenter.x += static_cast<float>(int_point.x);
            massCenter.y += static_cast<float>(int_point.y);
        }

        if (contour.size() > 1)
        {
            const float N = static_cast<float>(contour.size());

            massCenter.x /= N;
            massCenter.y /= N;
        }
    }

    return massCenter;
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
static void commonDeviceOrientationToOpenCVRodrigues(
    const PSVRQuatf &orientation,
    cv::Mat &rvec)
{
    double qw= clampf(orientation.w, -1.0, 1.0);
    double angle = 2.0 * acos(qw);
    double axis_normalizer = sqrt(1.0 - qw*qw);

    if (axis_normalizer > k_real_epsilon) 
    {
        rvec.at<double>(0) = angle * (orientation.x / axis_normalizer);
        rvec.at<double>(1) = angle * (orientation.y / axis_normalizer);
        rvec.at<double>(2) = angle * (orientation.z / axis_normalizer);
    }
    else
    {
        // Angle is either 0 or 360,
        // which is a rotation no-op so we are free to pick any axis we want
        rvec.at<double>(0) = angle; 
        rvec.at<double>(1) = 0.0;
        rvec.at<double>(2) = 0.0;
    }
}

static void openCVRodriguesToAngleAxis(
    const cv::Mat &rvec,
    float &axis_x, float &axis_y, float &axis_z, float &radians)
{
    const float r_x = static_cast<float>(rvec.at<double>(0));
    const float r_y = static_cast<float>(rvec.at<double>(1));
    const float r_z = static_cast<float>(rvec.at<double>(2));
    
    radians = sqrtf(r_x*r_x + r_y*r_y + r_z*r_z);

    axis_x= safe_divide_with_default(r_x, radians, 1.f);
    axis_y= safe_divide_with_default(r_y, radians, 0.f);
    axis_z= safe_divide_with_default(r_z, radians, 0.f);
}

// http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm
// NOTE: This code has the X and Z axis flipped from the code in the link
// because I consider rotation about the X-axis pitch and the Z-axis roll
// whereas the original code had the opposite.
// Also they refer to yaw as "heading", pitch as "attitude", and roll ""
static void angleAxisVectorToEulerAngles(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    float &yaw, float &pitch, float &roll)
{
    float s= sinf(radians);
    float c= cosf(radians);
    float t= 1.f-c;

    if ((axis_x*axis_y*t + axis_z*s) > 0.998) 
    {
        // north pole singularity detected
        yaw = 2*atan2f(axis_z*sinf(radians/2), cosf(radians/2));
        pitch = k_real_half_pi;
        roll = 0;
    }
    else if ((axis_x*axis_y*t + axis_z*s) < -0.998) 
    { 
        // south pole singularity detected
        yaw = -2*atan2(axis_z*sinf(radians/2), cosf(radians/2));
        pitch = -k_real_half_pi;
        roll = 0;
    }
    else
    {
        yaw = atan2f(axis_y*s - axis_x*axis_z*t, 1.f - (axis_y*axis_y + axis_z*axis_z)*t);
        pitch = asinf(axis_z*axis_y*t + axis_x*s) ;
        roll = atan2f(axis_z*s - axis_x*axis_y*t , 1.f - (axis_x*axis_x + axis_z*axis_z)*t);
    }
}

static void angleAxisVectorToCommonDeviceOrientation(
    const float axis_x, const float axis_y, const float axis_z, const float radians,
    PSVRQuatf &orientation)
{
    if (!is_nearly_zero(radians))
    {
        const float sin_theta_over_two = sinf(radians * 0.5f);
        const float w = cosf(radians * 0.5f);
        const float x = axis_x * sin_theta_over_two;
        const float y = axis_y * sin_theta_over_two;
        const float z = axis_z * sin_theta_over_two;
        const float length = sqrtf(w*w + x*x + y*y + z*z);

        if (length > k_normal_epsilon)
        {
            orientation.w = w / length;
            orientation.x = x / length;
            orientation.y = y / length;
            orientation.z = z / length;
        }
        else
        {
            orientation= *k_PSVR_quaternion_identity;
        }
    }
    else
    {
        orientation= *k_PSVR_quaternion_identity;
    }
}

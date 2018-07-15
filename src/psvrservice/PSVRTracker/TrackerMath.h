#ifndef TRACKER_MATH_H
#define TRACKER_MATH_H

// -- includes -----
#include <glm/gtc/quaternion.hpp>
#include "ClientConstants.h"

// -- pre-declarations -----
namespace cv {
    template<typename _Tp, int m, int n> 
	class Matx;

	typedef Matx<float, 3, 4> Matx34f;
	typedef Matx<float, 3, 3> Matx33f;
	typedef Matx<double, 3, 4> Matx34d;
	typedef Matx<double, 3, 3> Matx33d;
}

// -- interface -----
glm::quat computeGLMCameraTransformQuaternion(const class ITrackerInterface *tracker_device);
glm::mat4 computeGLMCameraTransformMatrix(const class ITrackerInterface *tracker_device);

void computeOpenCVCameraExtrinsicMatrix(const class ITrackerInterface *tracker_device, cv::Matx34f &out);
void computeOpenCVCameraIntrinsicMatrix(const class ITrackerInterface *tracker_device,
                                        PSVRVideoFrameSection section,
                                        cv::Matx33f &intrinsicOut,
                                        cv::Matx<float, 5, 1> &distortionOut);
bool computeOpenCVCameraRectification(const class ITrackerInterface *tracker_device,
                                        PSVRVideoFrameSection section,
                                        cv::Matx33d &rotationOut,
                                        cv::Matx34d &projectionOut);
#endif // TRACKER_MATH_H
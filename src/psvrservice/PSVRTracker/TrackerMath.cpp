#include "TrackerMath.h"
#include "DeviceInterface.h"
#include "MathGLM.h"
#include "PSVRClient_CAPI.h"

#include "opencv2/opencv.hpp"

glm::quat computeGLMCameraTransformQuaternion(const ITrackerInterface *tracker_device)
{

    const PSVRPosef pose = tracker_device->getTrackerPose();
    const PSVRQuatf &quat = pose.Orientation;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);

    return glm_quat;
}

glm::mat4 computeGLMCameraTransformMatrix(const ITrackerInterface *tracker_device)
{

    const PSVRPosef pose = tracker_device->getTrackerPose();
    const PSVRQuatf &quat = pose.Orientation;
    const PSVRVector3f &pos = pose.Position;

    const glm::quat glm_quat(quat.w, quat.x, quat.y, quat.z);
    const glm::vec3 glm_pos(pos.x, pos.y, pos.z);
    const glm::mat4 glm_camera_xform = glm_mat4_from_pose(glm_quat, glm_pos);

    return glm_camera_xform;
}

void computeOpenCVCameraExtrinsicMatrix(const ITrackerInterface *tracker_device,
                                               cv::Matx34f &out)
{
    // Extrinsic matrix is the inverse of the camera pose matrix
    const glm::mat4 glm_camera_xform = computeGLMCameraTransformMatrix(tracker_device);
    const glm::mat4 glm_mat = glm::inverse(glm_camera_xform);

    out(0, 0) = glm_mat[0][0]; out(0, 1) = glm_mat[1][0]; out(0, 2) = glm_mat[2][0]; out(0, 3) = glm_mat[3][0];
    out(1, 0) = glm_mat[0][1]; out(1, 1) = glm_mat[1][1]; out(1, 2) = glm_mat[2][1]; out(1, 3) = glm_mat[3][1];
    out(2, 0) = glm_mat[0][2]; out(2, 1) = glm_mat[1][2]; out(2, 2) = glm_mat[2][2]; out(2, 3) = glm_mat[3][2];
}

void computeOpenCVCameraIntrinsicMatrix(const ITrackerInterface *tracker_device,
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

void extractCameraIntrinsicMatrixParameters(const cv::Matx33f &intrinsic_matrix,
											float &out_focal_length_x,
											float &out_focal_length_y,
											float &out_principal_point_x,
											float &out_principal_point_y)
{
	out_focal_length_x= intrinsic_matrix(0, 0);
	out_focal_length_y= intrinsic_matrix(1, 1);
	out_principal_point_x= intrinsic_matrix(0, 2);
	out_principal_point_y= intrinsic_matrix(1, 2);
}

bool computeOpenCVCameraRectification(const ITrackerInterface *tracker_device,
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
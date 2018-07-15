#ifndef MATH_TYPE_CONVERSION_H
#define MATH_TYPE_CONVERSION_H

//-- includes -----
#undef Status
#undef Success
#include <opencv2/opencv.hpp>
#include "MathEigen.h"
#include "ClientGeometry_CAPI.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>

//-- methods -----
// PSVR types to PSVR Types
PSVRQuatf PSVR_matrix3f_to_PSVR_quatf(const PSVRMatrix3f &m);
PSVRMatrix3f PSVR_quatf_to_PSVR_matrix3f(const PSVRQuatf &q);

// PSVR types to GLM types
glm::vec3 PSVR_vector3f_to_glm_vec3(const PSVRVector3f &v);
glm::quat PSVR_quatf_to_glm_quat(const PSVRQuatf &q);
glm::mat3 PSVR_matrix3f_to_glm_mat3(const PSVRMatrix3f &m);
glm::mat4 PSVR_posef_to_glm_mat4(const PSVRQuatf &quat, const PSVRVector3f &pos);
glm::mat4 PSVR_posef_to_glm_mat4(const PSVRPosef &pose);

// GLM Types to PSVR types
PSVRVector3f glm_vec3_to_PSVR_vector3f(const glm::vec3 &v);
PSVRMatrix3f glm_mat3_to_PSVR_matrix3f(const glm::mat3 &m);
PSVRQuatf glm_mat3_to_PSVR_quatf(const glm::quat &q);
PSVRPosef glm_mat4_to_PSVR_posef(const glm::mat4 &m);

// GLM Types to Eigen types
Eigen::Matrix3f glm_mat3_to_eigen_matrix3f(const glm::mat3 &m);
Eigen::Matrix4f glm_mat4_to_eigen_matrix4f(const glm::mat4 &m);

// PSVR types to OpenCV types
cv::Matx33f PSVR_matrix3x3_to_cv_mat33f(const PSVRMatrix3f &in);
cv::Matx33f PSVR_matrix3x3_to_cv_mat33f(const PSVRMatrix3d &in);
cv::Matx33d PSVR_matrix3x3_to_cv_mat33d(const PSVRMatrix3d &in);
cv::Matx34d PSVR_matrix3x4_to_cv_mat34d(const PSVRMatrix34d &in);
cv::Matx44d PSVR_matrix4x4_to_cv_mat44d(const PSVRMatrix4d &in);
cv::Vec3d PSVR_vector3d_to_cv_vec3d(const PSVRVector3d &in);

// OpenCV types to PSVR types
PSVRMatrix3f cv_mat33f_to_PSVR_matrix3x3(const cv::Matx33f &in);
PSVRMatrix3d cv_mat33d_to_PSVR_matrix3x3(const cv::Matx33d &in);
PSVRMatrix4d cv_mat44d_to_PSVR_matrix4x4(const cv::Matx44d &in);
PSVRMatrix34d cv_mat34d_to_PSVR_matrix3x4(const cv::Matx34d &in);
PSVRVector3d cv_vec3d_to_PSVR_vector3d(const cv::Vec3d &in);

// OpenCV types to Eigen tyes
Eigen::Quaterniond cv_rodrigues_vector_to_eigen_quatd(const cv::Mat &in);
Eigen::Vector3d cv_vector3d_to_eigen_vector3d(const cv::Mat &vec);

// PSVRTypes to Eigen types
Eigen::Vector3f PSVR_vector3i_to_eigen_vector3(const PSVRVector3i &v);
Eigen::Vector3f PSVR_vector3f_to_eigen_vector3(const PSVRVector3f &p);
Eigen::Vector3f PSVR_vector3f_to_eigen_vector3(const PSVRVector3f &v);
Eigen::Quaternionf PSVR_quatf_to_eigen_quaternionf(const PSVRQuatf &q);
Eigen::Matrix3f PSVR_matrix3f_to_eigen_matrix3(const PSVRMatrix3f &m);
Eigen::Matrix3f PSVR_matrix3d_to_eigen_matrix3f(const PSVRMatrix3d &m);
Eigen::Matrix4f PSVR_matrix4d_to_eigen_matrix4f(const PSVRMatrix4d &m);
Eigen::Matrix4d PSVR_matrix4d_to_eigen_matrix4d(const PSVRMatrix4d &m);

// Eigen types to GLM types
glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m);
glm::mat4 eigen_matrix4f_to_glm_mat4(const Eigen::Matrix4f &m);
glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v);

// Eigen types to PSVR types
PSVRVector3f eigen_vector3f_to_PSVR_vector3f(const Eigen::Vector3f &v);
PSVRQuatf eigen_quaternionf_to_PSVR_quatf(const Eigen::Quaternionf &q);

// Eigen types to OpenCV types
cv::Mat eigen_quatd_to_cv_rodrigues_vector(const Eigen::Quaterniond &in);
cv::Mat eigen_vector3d_to_cv_vector3d(const Eigen::Vector3d &in);

#endif // MATH_TYPE_CONVERSION_H
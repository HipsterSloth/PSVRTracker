#include "GeometryUtility.h"
#include "MathGLM.h"

//-- methods -----
// PSVR types to PSVR Types
PSVRQuatf PSVR_matrix3f_to_PSVR_quatf(const PSVRMatrix3f &m)
{
    glm::mat3 glm_mat3 = PSVR_matrix3f_to_glm_mat3(m);
    glm::quat glm_quat = glm::quat_cast(glm_mat3);
    PSVRQuatf result = glm_mat3_to_PSVR_quatf(glm_quat);

    return result;
}

PSVRMatrix3f PSVR_quatf_to_PSVR_matrix3f(const PSVRQuatf &q)
{
    glm::quat glm_quat= PSVR_quatf_to_glm_quat(q);
    glm::mat3 glm_mat3 = glm::mat3_cast(glm_quat);
    PSVRMatrix3f result = glm_mat3_to_PSVR_matrix3f(glm_mat3);

    return result;
}

// PSVR types to GLM types
glm::vec3 PSVR_vector3f_to_glm_vec3(const PSVRVector3f &v)
{
    return glm::vec3(v.x, v.y, v.z);
}

glm::quat PSVR_quatf_to_glm_quat(const PSVRQuatf &q)
{
    return glm::quat(q.w, q.x, q.y, q.z);
}

glm::mat3 PSVR_matrix3f_to_glm_mat3(const PSVRMatrix3f &m)
{
    // GLM column matrix constructor
    return glm::mat3x3(
        PSVR_vector3f_to_glm_vec3(PSVR_Matrix3fBasisX(&m)),
        PSVR_vector3f_to_glm_vec3(PSVR_Matrix3fBasisY(&m)),
        PSVR_vector3f_to_glm_vec3(PSVR_Matrix3fBasisZ(&m)));
}

glm::mat4
PSVR_posef_to_glm_mat4(const PSVRQuatf &quat, const PSVRVector3f &pos)
{
    glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
    glm::vec3 position(pos.x, pos.y, pos.z);
    glm::mat4 transform = glm_mat4_from_pose(orientation, position);

    return transform;
}

glm::mat4
PSVR_posef_to_glm_mat4(const PSVRPosef &pose)
{
    return PSVR_posef_to_glm_mat4(pose.Orientation, pose.Position);
}

// GLM Types to PSVR types
PSVRVector3f glm_vec3_to_PSVR_vector3f(const glm::vec3 &v)
{
    return {v.x, v.y, v.z};
}

PSVRMatrix3f glm_mat3_to_PSVR_matrix3f(const glm::mat3 &m)
{
    // Basis vectors are stored in columns in GLM
    PSVRVector3f basis_x= {m[0].x, m[0].y, m[0].z};
    PSVRVector3f basis_y= {m[1].x, m[1].y, m[1].z};
    PSVRVector3f basis_z= {m[2].x, m[2].y, m[2].z};

	return PSVR_Matrix3fCreate(&basis_x, &basis_y, &basis_z);
}

PSVRQuatf glm_mat3_to_PSVR_quatf(const glm::quat &q)
{
    return PSVR_QuatfCreate(q.w, q.x, q.y, q.z);
}

PSVRPosef glm_mat4_to_PSVR_posef(const glm::mat4 &m)
{
    const glm::quat q = glm::quat_cast(m);
    const glm::vec3 p = glm::vec3(m[3]);
    PSVRPosef result;

    result.Orientation = glm_mat3_to_PSVR_quatf(q);
    result.Position = glm_vec3_to_PSVR_vector3f(p);

    return result;
}

// PSVR types to OpenCV types
cv::Matx33f
PSVR_matrix3x3_to_cv_mat33f(const PSVRMatrix3f &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    cv::Matx33f out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

cv::Matx33f
PSVR_matrix3x3_to_cv_mat33f(const PSVRMatrix3d &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    cv::Matx33f out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = static_cast<float>(in.m[i][j]);
        }
    }

    return out;
}

cv::Matx33d PSVR_matrix3x3_to_cv_mat33d(const PSVRMatrix3d &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    cv::Matx33d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

cv::Vec3d PSVR_vector3d_to_cv_vec3d(const PSVRVector3d &in)
{
    return cv::Vec3d(in.x, in.y, in.z);
}

cv::Matx34d PSVR_matrix3x4_to_cv_mat34d(const PSVRMatrix34d &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    cv::Matx34d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

cv::Matx44d PSVR_matrix4x4_to_cv_mat44d(const PSVRMatrix4d &in)
{
    // Both OpenCV and PSVRMatrix4d matrices are stored row-major
    cv::Matx44d out;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out(i, j) = in.m[i][j];
        }
    }

    return out;
}

// OpenCV types to PSVR types
PSVRMatrix3f 
cv_mat33f_to_PSVR_matrix3x3(const cv::Matx33f &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    PSVRMatrix3f out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSVRMatrix3d
cv_mat33d_to_PSVR_matrix3x3(const cv::Matx33d &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    PSVRMatrix3d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSVRMatrix4d cv_mat44d_to_PSVR_matrix4x4(const cv::Matx44d &in)
{
    // Both OpenCV and PSVRMatrix4f matrices are stored row-major
    PSVRMatrix4d out;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSVRMatrix34d 
cv_mat34d_to_PSVR_matrix3x4(const cv::Matx34d &in)
{
    // Both OpenCV and PSVRMatrix3f matrices are stored row-major
    PSVRMatrix34d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSVRVector3d 
cv_vec3d_to_PSVR_vector3d(const cv::Vec3d &in)
{
    return {in(0), in(1), in(2)};
}

// GLM Types to Eigen types
Eigen::Matrix3f glm_mat3_to_eigen_matrix3f(const glm::mat3 &m)
{
    const float *raw_m = glm::value_ptr(m);
    float copy_raw_m[3* 3];
    memcpy(copy_raw_m, raw_m, sizeof(copy_raw_m));
    Eigen::Map<Eigen::Matrix3f> result(copy_raw_m);

    return result;
}

Eigen::Matrix4f glm_mat4_to_eigen_matrix4f(const glm::mat4 &m)
{
    const float *raw_m = glm::value_ptr(m);
    float copy_raw_m[4 * 4];
    memcpy(copy_raw_m, raw_m, sizeof(copy_raw_m));
    Eigen::Map<Eigen::Matrix4f> result(copy_raw_m);

    return result;
}

// PSVRTypes to Eigen types
Eigen::Vector3f PSVR_vector3i_to_eigen_vector3(const PSVRVector3i &v)
{
    return Eigen::Vector3f(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Eigen::Vector3f PSVR_vector3f_to_eigen_vector3(const PSVRVector3f &v)
{
    return Eigen::Vector3f(v.x, v.y, v.z);
}

Eigen::Quaternionf PSVR_quatf_to_eigen_quaternionf(const PSVRQuatf &q)
{
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

Eigen::Matrix3f PSVR_matrix3f_to_eigen_matrix3(const PSVRMatrix3f &m)
{
	Eigen::Matrix3f result;

    PSVRVector3f basis_x= PSVR_Matrix3fBasisX(&m);
    PSVRVector3f basis_y= PSVR_Matrix3fBasisY(&m);
    PSVRVector3f basis_z= PSVR_Matrix3fBasisZ(&m);

	result << basis_x.x, basis_x.y, basis_x.z,
		basis_y.x, basis_y.y, basis_y.z,
		basis_z.x, basis_z.y, basis_z.z;

	return result;
}

Eigen::Matrix3f PSVR_matrix3d_to_eigen_matrix3f(const PSVRMatrix3d &m)
{
	Eigen::Matrix3f result;

    PSVRVector3d basis_x= PSVR_Matrix3dBasisX(&m);
    PSVRVector3d basis_y= PSVR_Matrix3dBasisY(&m);
    PSVRVector3d basis_z= PSVR_Matrix3dBasisZ(&m);

	result << (float)basis_x.x, (float)basis_x.y, (float)basis_x.z,
		(float)basis_y.x, (float)basis_y.y, (float)basis_y.z,
		(float)basis_z.x, (float)basis_z.y, (float)basis_z.z;

	return result;
}

Eigen::Matrix4f PSVR_matrix4d_to_eigen_matrix4f(const PSVRMatrix4d &m)
{
	Eigen::Matrix4f result;

    PSVRVector4d basis_x= PSVR_Matrix4dBasisX(&m);
    PSVRVector4d basis_y= PSVR_Matrix4dBasisY(&m);
    PSVRVector4d basis_z= PSVR_Matrix4dBasisZ(&m);
    PSVRVector4d basis_w= PSVR_Matrix4dBasisW(&m);

	result << (float)basis_x.x, (float)basis_x.y, (float)basis_x.z, (float)basis_x.w,
		(float)basis_y.x, (float)basis_y.y, (float)basis_y.z, (float)basis_y.w,
		(float)basis_z.x, (float)basis_z.y, (float)basis_z.z, (float)basis_z.w,
        (float)basis_w.x, (float)basis_w.y, (float)basis_w.z, (float)basis_w.w;

	return result;
}

Eigen::Matrix4d PSVR_matrix4d_to_eigen_matrix4d(const PSVRMatrix4d &m)
{
    Eigen::Matrix4d result;

    PSVRVector4d basis_x= PSVR_Matrix4dBasisX(&m);
    PSVRVector4d basis_y= PSVR_Matrix4dBasisY(&m);
    PSVRVector4d basis_z= PSVR_Matrix4dBasisZ(&m);
    PSVRVector4d basis_w= PSVR_Matrix4dBasisW(&m);

    result << basis_x.x, basis_x.y, basis_x.z, basis_x.w,
        basis_y.x, basis_y.y, basis_y.z, basis_y.w,
        basis_z.x, basis_z.y, basis_z.z, basis_z.w,
        basis_w.x, basis_w.y, basis_w.z, basis_w.w;

    return result;
}

// Eigen types to GLM types
glm::mat3 eigen_matrix3f_to_glm_mat3(const Eigen::Matrix3f &m)
{
    return glm::make_mat3x3((const float *)m.data());
}

glm::mat4 eigen_matrix4f_to_glm_mat4(const Eigen::Matrix4f &m)
{
    return glm::make_mat4x4((const float *)m.data());
}

glm::vec3 eigen_vector3f_to_glm_vec3(const Eigen::Vector3f &v)
{
    return glm::vec3(v.x(), v.y(), v.z());
}

// Eigen types to PSVR types
PSVRVector3f eigen_vector3f_to_PSVR_vector3f(const Eigen::Vector3f &v)
{
	return {v.x(), v.y(), v.z()};
}
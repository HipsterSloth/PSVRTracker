#include "GeometryUtility.h"
#include "MathGLM.h"

//-- methods -----
// PSMove types to PSMove Types
PSMQuatf psm_matrix3f_to_psm_quatf(const PSMMatrix3f &m)
{
    glm::mat3 glm_mat3 = psm_matrix3f_to_glm_mat3(m);
    glm::quat glm_quat = glm::quat_cast(glm_mat3);
    PSMQuatf result = glm_mat3_to_psm_quatf(glm_quat);

    return result;
}

PSMMatrix3f psm_quatf_to_psm_matrix3f(const PSMQuatf &q)
{
    glm::quat glm_quat= psm_quatf_to_glm_quat(q);
    glm::mat3 glm_mat3 = glm::mat3_cast(glm_quat);
    PSMMatrix3f result = glm_mat3_to_psm_matrix3f(glm_mat3);

    return result;
}

// PSMove types to GLM types
glm::vec3 psm_vector3f_to_glm_vec3(const PSMVector3f &v)
{
    return glm::vec3(v.x, v.y, v.z);
}

glm::quat psm_quatf_to_glm_quat(const PSMQuatf &q)
{
    return glm::quat(q.w, q.x, q.y, q.z);
}

glm::mat3 psm_matrix3f_to_glm_mat3(const PSMMatrix3f &m)
{
    // GLM column matrix constructor
    return glm::mat3x3(
        psm_vector3f_to_glm_vec3(PSM_Matrix3fBasisX(&m)),
        psm_vector3f_to_glm_vec3(PSM_Matrix3fBasisY(&m)),
        psm_vector3f_to_glm_vec3(PSM_Matrix3fBasisZ(&m)));
}

glm::mat4
psm_posef_to_glm_mat4(const PSMQuatf &quat, const PSMVector3f &pos)
{
    glm::quat orientation(quat.w, quat.x, quat.y, quat.z);
    glm::vec3 position(pos.x, pos.y, pos.z);
    glm::mat4 transform = glm_mat4_from_pose(orientation, position);

    return transform;
}

glm::mat4
psm_posef_to_glm_mat4(const PSMPosef &pose)
{
    return psm_posef_to_glm_mat4(pose.Orientation, pose.Position);
}

// GLM Types to PSMove types
PSMVector3f glm_vec3_to_psm_vector3f(const glm::vec3 &v)
{
    return {v.x, v.y, v.z};
}

PSMMatrix3f glm_mat3_to_psm_matrix3f(const glm::mat3 &m)
{
    // Basis vectors are stored in columns in GLM
    PSMVector3f basis_x= {m[0].x, m[0].y, m[0].z};
    PSMVector3f basis_y= {m[1].x, m[1].y, m[1].z};
    PSMVector3f basis_z= {m[2].x, m[2].y, m[2].z};

	return PSM_Matrix3fCreate(&basis_x, &basis_y, &basis_z);
}

PSMQuatf glm_mat3_to_psm_quatf(const glm::quat &q)
{
    return PSM_QuatfCreate(q.w, q.x, q.y, q.z);
}

PSMPosef glm_mat4_to_psm_posef(const glm::mat4 &m)
{
    const glm::quat q = glm::quat_cast(m);
    const glm::vec3 p = glm::vec3(m[3]);
    PSMPosef result;

    result.Orientation = glm_mat3_to_psm_quatf(q);
    result.Position = glm_vec3_to_psm_vector3f(p);

    return result;
}

// PSMove types to OpenCV types
cv::Matx33f
psmove_matrix3x3_to_cv_mat33f(const PSMMatrix3f &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
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
psmove_matrix3x3_to_cv_mat33f(const PSMMatrix3d &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
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

cv::Matx33d psmove_matrix3x3_to_cv_mat33d(const PSMMatrix3d &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
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

cv::Vec3d psmove_vector3d_to_cv_vec3d(const PSMVector3d &in)
{
    return cv::Vec3d(in.x, in.y, in.z);
}

cv::Matx34d psmove_matrix3x4_to_cv_mat34d(const PSMMatrix34d &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
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

cv::Matx44d psmove_matrix4x4_to_cv_mat44d(const PSMMatrix4d &in)
{
    // Both OpenCV and PSMMatrix4d matrices are stored row-major
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

// OpenCV types to PSMove types
PSMMatrix3f 
cv_mat33f_to_psm_matrix3x3(const cv::Matx33f &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
    PSMMatrix3f out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSMMatrix3d
cv_mat33d_to_psm_matrix3x3(const cv::Matx33d &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
    PSMMatrix3d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSMMatrix4d cv_mat44d_to_psm_matrix4x4(const cv::Matx44d &in)
{
    // Both OpenCV and PSMMatrix4f matrices are stored row-major
    PSMMatrix4d out;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSMMatrix34d 
cv_mat34d_to_psm_matrix3x4(const cv::Matx34d &in)
{
    // Both OpenCV and PSMMatrix3f matrices are stored row-major
    PSMMatrix34d out;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            out.m[i][j]= in(i, j);
        }
    }

    return out;
}

PSMVector3d 
cv_vec3d_to_psm_vector3d(const cv::Vec3d &in)
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

// PSMoveTypes to Eigen types
Eigen::Vector3f psm_vector3i_to_eigen_vector3(const PSMVector3i &v)
{
    return Eigen::Vector3f(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

Eigen::Vector3f psm_vector3f_to_eigen_vector3(const PSMVector3f &v)
{
    return Eigen::Vector3f(v.x, v.y, v.z);
}

Eigen::Quaternionf psm_quatf_to_eigen_quaternionf(const PSMQuatf &q)
{
    return Eigen::Quaternionf(q.w, q.x, q.y, q.z);
}

Eigen::Matrix3f psm_matrix3f_to_eigen_matrix3(const PSMMatrix3f &m)
{
	Eigen::Matrix3f result;

    PSMVector3f basis_x= PSM_Matrix3fBasisX(&m);
    PSMVector3f basis_y= PSM_Matrix3fBasisY(&m);
    PSMVector3f basis_z= PSM_Matrix3fBasisZ(&m);

	result << basis_x.x, basis_x.y, basis_x.z,
		basis_y.x, basis_y.y, basis_y.z,
		basis_z.x, basis_z.y, basis_z.z;

	return result;
}

Eigen::Matrix3f psm_matrix3d_to_eigen_matrix3f(const PSMMatrix3d &m)
{
	Eigen::Matrix3f result;

    PSMVector3d basis_x= PSM_Matrix3dBasisX(&m);
    PSMVector3d basis_y= PSM_Matrix3dBasisY(&m);
    PSMVector3d basis_z= PSM_Matrix3dBasisZ(&m);

	result << (float)basis_x.x, (float)basis_x.y, (float)basis_x.z,
		(float)basis_y.x, (float)basis_y.y, (float)basis_y.z,
		(float)basis_z.x, (float)basis_z.y, (float)basis_z.z;

	return result;
}

Eigen::Matrix4f psm_matrix4d_to_eigen_matrix4f(const PSMMatrix4d &m)
{
	Eigen::Matrix4f result;

    PSMVector4d basis_x= PSM_Matrix4dBasisX(&m);
    PSMVector4d basis_y= PSM_Matrix4dBasisY(&m);
    PSMVector4d basis_z= PSM_Matrix4dBasisZ(&m);
    PSMVector4d basis_w= PSM_Matrix4dBasisW(&m);

	result << (float)basis_x.x, (float)basis_x.y, (float)basis_x.z, (float)basis_x.w,
		(float)basis_y.x, (float)basis_y.y, (float)basis_y.z, (float)basis_y.w,
		(float)basis_z.x, (float)basis_z.y, (float)basis_z.z, (float)basis_z.w,
        (float)basis_w.x, (float)basis_w.y, (float)basis_w.z, (float)basis_w.w;

	return result;
}

Eigen::Matrix4d psm_matrix4d_to_eigen_matrix4d(const PSMMatrix4d &m)
{
    Eigen::Matrix4d result;

    PSMVector4d basis_x= PSM_Matrix4dBasisX(&m);
    PSMVector4d basis_y= PSM_Matrix4dBasisY(&m);
    PSMVector4d basis_z= PSM_Matrix4dBasisZ(&m);
    PSMVector4d basis_w= PSM_Matrix4dBasisW(&m);

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

// Eigen types to PSMove types
PSMVector3f eigen_vector3f_to_psm_vector3f(const Eigen::Vector3f &v)
{
	return {v.x(), v.y(), v.z()};
}
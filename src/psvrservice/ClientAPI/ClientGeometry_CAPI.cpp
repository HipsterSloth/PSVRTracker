//-- includes -----
#include "ClientGeometry_CAPI.h"
#include "MathUtility.h"
#include "MathGLM.h"
#include <algorithm>

//-- constants -----

const PSVRVector2f g_PSVR_float_vector2_zero= {0.f, 0.f};
const PSVRVector2f *k_PSVR_float_vector2_zero= &g_PSVR_float_vector2_zero;

const PSVRVector3f g_PSVR_float_vector3_zero= {0.f, 0.f, 0.f};
const PSVRVector3f *k_PSVR_float_vector3_zero= &g_PSVR_float_vector3_zero;

const PSVRVector3f g_PSVR_float_vector3_one= {1.f, 1.f, 1.f};
const PSVRVector3f *k_PSVR_float_vector3_one= &g_PSVR_float_vector3_one;

const PSVRVector3f g_PSVR_float_vector3_i = { 1.f, 0.f, 0.f };
const PSVRVector3f *k_PSVR_float_vector3_i = &g_PSVR_float_vector3_i;

const PSVRVector3f g_PSVR_float_vector3_j = { 0.f, 1.f, 0.f };
const PSVRVector3f *k_PSVR_float_vector3_j = &g_PSVR_float_vector3_j;

const PSVRVector3f g_PSVR_float_vector3_k = { 0.f, 0.f, 1.f };
const PSVRVector3f *k_PSVR_float_vector3_k = &g_PSVR_float_vector3_k;

const PSVRVector3i g_PSVR_int_vector3_zero= {0, 0, 0};
const PSVRVector3i *k_PSVR_int_vector3_zero= &g_PSVR_int_vector3_zero;

const PSVRVector3i g_PSVR_int_vector3_one= {1, 1, 1};
const PSVRVector3i *k_PSVR_int_vector3_one= &g_PSVR_int_vector3_one;

const PSVRVector3f g_PSVR_position_origin= {0.f, 0.f, 0.f};
const PSVRVector3f *k_PSVR_position_origin= &g_PSVR_position_origin;

const PSVRQuatf g_PSVR_quaternion_identity= {1.f, 0.f, 0.f, 0.f};
const PSVRQuatf *k_PSVR_quaternion_identity= &g_PSVR_quaternion_identity;

const PSVRMatrix3f g_PSVR_matrix_identity = { {{1.f, 0.f, 0.f} , {0.f, 1.f, 0.f}, {0.f, 0.f, 1.f}} };
const PSVRMatrix3f *k_PSVR_matrix_identity = &g_PSVR_matrix_identity;

const PSVRPosef g_PSVR_pose_identity = { g_PSVR_position_origin, g_PSVR_quaternion_identity };
const PSVRPosef *k_PSVR_pose_identity = &g_PSVR_pose_identity;

//-- methods -----
// PSVRVector2f Methods
PSVRVector2f PSVR_Vector2fAdd(const PSVRVector2f *a, const PSVRVector2f *b)
{
	return {a->x + b->x, a->y + b->y};
}

PSVRVector2f PSVR_Vector2fSubtract(const PSVRVector2f *a, const PSVRVector2f *b)
{
	return {a->x - b->x, a->y - b->y};
}

PSVRVector2f PSVR_Vector2fScale(const PSVRVector2f *v, const float s)
{
	return {v->x*s, v->y*s};
}

PSVRVector2f PSVR_Vector2fScaleAndAdd(const PSVRVector2f *v, const float s, const PSVRVector2f *b)
{
	return {v->x*s + b->x, v->y*s + b->y};
}

PSVRVector2f PSVR_Vector2fUnsafeScalarDivide(const PSVRVector2f *numerator, const float divisor)
{
	return {numerator->x/divisor, numerator->y/divisor};
}

PSVRVector2f PSVR_Vector2fUnsafeVectorDivide(const PSVRVector2f *numerator, const PSVRVector2f *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y};
}

PSVRVector2f PSVR_Vector2fSafeScalarDivide(const PSVRVector2f *numerator, const float divisor, const PSVRVector2f *default_result)
{
	return !is_nearly_zero(divisor) ? PSVR_Vector2fUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSVRVector2f PSVR_Vector2fSafeVectorDivide(const PSVRVector2f *numerator, const PSVRVector2f *divisor, const PSVRVector2f *default_result)
{
	return {!is_nearly_zero(divisor->x) ? (numerator->x / divisor->x) : default_result->x,
			!is_nearly_zero(divisor->y) ? (numerator->y / divisor->y) : default_result->y};
}

PSVRVector2f PSVR_Vector2fAbs(const PSVRVector2f *v)
{
	return {fabsf(v->x), fabsf(v->y)};
}

PSVRVector2f PSVR_Vector2fSquare(const PSVRVector2f *v)
{
	return {v->x*v->x, v->y*v->y};
}

float PSVR_Vector2fLength(const PSVRVector2f *v)
{
	return sqrtf(v->x*v->x + v->y*v->y);
}

PSVRVector2f PSVR_Vector2fNormalizeWithDefault(const PSVRVector2f *v, const PSVRVector2f *default_result)
{
    return PSVR_Vector2fSafeScalarDivide(v, PSVR_Vector2fLength(v), default_result);
}

float PSVR_Vector2fMinValue(const PSVRVector2f *v)
{
	return fminf(v->x, v->y);
}

float PSVR_Vector2fMaxValue(const PSVRVector2f *v)
{
	return fmaxf(v->x, v->y);
}

float PSVR_Vector2fDot(const PSVRVector2f *a, const PSVRVector2f *b)
{
	return a->x*b->x + a->y*b->y;
}

float PSVR_Vector2fDistanceSquared(const PSVRVector2f *a, const PSVRVector2f *b)
{
    PSVRVector2f diff= PSVR_Vector2fSubtract(a, b);

    return PSVR_Vector2fDot(&diff, &diff);
}

float PSVR_Vector2fDistance(const PSVRVector2f *a, const PSVRVector2f *b)
{
    return sqrtf(PSVR_Vector2fDistanceSquared(a, b));
}

PSVRVector2f PSVR_Vector2fMin(const PSVRVector2f *a, const PSVRVector2f *b)
{
	return { fminf(a->x, b->x), fminf(a->y, b->y) };
}

PSVRVector2f PSVR_Vector2fMax(const PSVRVector2f *a, const PSVRVector2f *b)
{
	return { fmaxf(a->x, b->x), fmaxf(a->y, b->y) };
}

// PSVRVector3f Methods
PSVRVector3f PSVR_Vector3fAdd(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return {a->x + b->x, a->y + b->y, a->z + b->z};
}

PSVRVector3f PSVR_Vector3fSubtract(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return {a->x - b->x, a->y - b->y, a->z - b->z};
}

PSVRVector3f PSVR_Vector3fScale(const PSVRVector3f *v, const float s)
{
	return {v->x*s, v->y*s, v->z*s};
}

PSVRVector3f PSVR_Vector3fScaleAndAdd(const PSVRVector3f *v, const float s, const PSVRVector3f *b)
{
	return {v->x*s + b->x, v->y*s + b->y, v->z*s + b->z};
}

PSVRVector3f PSVR_Vector3fUnsafeScalarDivide(const PSVRVector3f *numerator, const float divisor)
{
	return {numerator->x/divisor, numerator->y/divisor, numerator->z/divisor};
}

PSVRVector3f PSVR_Vector3fUnsafeVectorDivide(const PSVRVector3f *numerator, const PSVRVector3f *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y, numerator->z/divisor->z};
}

PSVRVector3f PSVR_Vector3fSafeScalarDivide(const PSVRVector3f *numerator, const float divisor, const PSVRVector3f *default_result)
{
	return !is_nearly_zero(divisor) ? PSVR_Vector3fUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSVRVector3f PSVR_Vector3fSafeVectorDivide(const PSVRVector3f *numerator, const PSVRVector3f *divisor, const PSVRVector3f *default_result)
{
	return {!is_nearly_zero(divisor->x) ? (numerator->x / divisor->x) : default_result->x,
			!is_nearly_zero(divisor->y) ? (numerator->y / divisor->y) : default_result->y,
			!is_nearly_zero(divisor->z) ? (numerator->z / divisor->z) : default_result->z};
}

PSVRVector3f PSVR_Vector3fAbs(const PSVRVector3f *v)
{
	return {fabsf(v->x), fabsf(v->y), fabsf(v->z)};
}

PSVRVector3f PSVR_Vector3fSquare(const PSVRVector3f *v)
{
	return {v->x*v->x, v->y*v->y, v->z*v->z};
}

float PSVR_Vector3fLength(const PSVRVector3f *v)
{
	return sqrtf(v->x*v->x + v->y*v->y + v->z*v->z);
}

PSVRVector3f PSVR_Vector3fNormalizeWithDefault(const PSVRVector3f *v, const PSVRVector3f *default_result)
{
	return PSVR_Vector3fSafeScalarDivide(v, PSVR_Vector3fLength(v), default_result);
}

PSVRVector3f PSVR_Vector3fNormalizeWithDefaultGetLength(const PSVRVector3f *v, const PSVRVector3f *default_result, float *out_length)
{
	const float length= PSVR_Vector3fLength(v);
		
	if (out_length)
		*out_length= length;

	return PSVR_Vector3fSafeScalarDivide(v, length, default_result);
}

float PSVR_Vector3fMinValue(const PSVRVector3f *v)
{
	return fminf(fminf(v->x, v->y), v->z);
}

float PSVR_Vector3fMaxValue(const PSVRVector3f *v)
{
	return fmaxf(fmaxf(v->x, v->y), v->z);
}

float PSVR_Vector3fDot(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return a->x*b->x + a->y*b->y + a->z*b->z;
}

PSVRVector3f PSVR_Vector3fCross(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return {a->y*b->z - b->y*a->z, a->x*b->z - b->x*a->z, a->x*b->y - b->x*a->y};
}

PSVRVector3f PSVR_Vector3fMin(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return {fminf(a->x, b->x), fminf(a->y, b->y), fminf(a->z, b->z)};
}

PSVRVector3f PSVR_Vector3fMax(const PSVRVector3f *a, const PSVRVector3f *b)
{
	return {fmaxf(a->x, b->x), fmaxf(a->y, b->y), fmaxf(a->z, b->z)};
}

// PSVRVector3i Methods
PSVRVector3i PSVR_Vector3iAdd(const PSVRVector3i *a, const PSVRVector3i *b)
{
	return {a->x + b->x, a->y + b->y, a->z + b->z};
}

PSVRVector3i PSVR_Vector3iSubtract(const PSVRVector3i *a, const PSVRVector3i *b)
{
	return {a->x - b->x, a->y - b->y, a->z - b->z};
}

PSVRVector3i PSVR_Vector3iUnsafeScalarDivide(const PSVRVector3i *numerator, const int divisor)
{
	return {numerator->x/divisor, numerator->y/divisor, numerator->z/divisor};
}

PSVRVector3i PSVR_Vector3iUnsafeVectorDivide(const PSVRVector3i *numerator, const PSVRVector3i *divisor)
{
	return {numerator->x/divisor->x, numerator->y/divisor->y, numerator->z/divisor->z};
}

PSVRVector3i PSVR_Vector3iSafeScalarDivide(const PSVRVector3i *numerator, const int divisor, const PSVRVector3i *default_result)
{
	return divisor != 0 ? PSVR_Vector3iUnsafeScalarDivide(numerator, divisor) : *default_result;
}

PSVRVector3i PSVR_Vector3iSafeVectorDivide(const PSVRVector3i *numerator, const PSVRVector3i *divisor, const PSVRVector3i *default_result)
{
	return {divisor->x != 0 ? (numerator->x / divisor->x) : default_result->x,
			divisor->y != 0 ? (numerator->y / divisor->y) : default_result->y,
			divisor->z != 0 ? (numerator->z / divisor->z) : default_result->z};
}

PSVRVector3i PSVR_Vector3iAbs(const PSVRVector3i *v)
{
	return {std::abs(v->x), std::abs(v->y), std::abs(v->z)};
}

PSVRVector3i PSVR_Vector3iSquare(const PSVRVector3i *v)
{
	return {v->x*v->x, v->y*v->y, v->z*v->z};
}

int PSVR_Vector3iLengthSquared(const PSVRVector3i *v)
{
	return v->x*v->x + v->y*v->y + v->z*v->z;
}

int PSVR_Vector3iMinValue(const PSVRVector3i *v)
{
	return std::min(std::min(v->x, v->y), v->z);
}

int PSVR_Vector3iMaxValue(const PSVRVector3i *v)
{
	return std::max(std::max(v->x, v->y), v->z);
}

PSVRVector3i PSVR_Vector3iMin(const PSVRVector3i *a, const PSVRVector3i *b)
{
	return {std::min(a->x, b->x), std::min(a->y, b->y), std::min(a->z, b->z)};
}

PSVRVector3i PSVR_Vector3iMax(const PSVRVector3i *a, const PSVRVector3i *b)
{
	return {std::max(a->x, b->x), std::max(a->y, b->y), std::max(a->z, b->z)};
}

PSVRVector3f PSVR_Vector3iCastToFloat(const PSVRVector3i *v)
{
	return { static_cast<float>(v->x), static_cast<float>(v->y), static_cast<float>(v->z) };
}

// PSVRQuatf Methods
PSVRQuatf PSVR_QuatfCreate(float w, float x, float y, float z)
{
	return {w, x, y, z};
}

PSVRQuatf PSVR_QuatfCreateFromAngles(const PSVRVector3f *eulerAngles)
{
	PSVRQuatf q;

	// Assuming the angles are in radians.
	float c1 = cosf(eulerAngles->y / 2.f);
	float s1 = sinf(eulerAngles->y / 2.f);
	float c2 = cosf(eulerAngles->z / 2.f);
	float s2 = sinf(eulerAngles->z / 2.f);
	float c3 = cosf(eulerAngles->x / 2.f);
	float s3 = sinf(eulerAngles->x / 2.f);
	float c1c2 = c1*c2;
	float s1s2 = s1*s2;
	q.w = c1c2*c3 - s1s2*s3;
	q.x = c1c2*s3 + s1s2*c3;
	q.y = s1*c2*c3 + c1*s2*s3;
	q.z = c1*s2*c3 - s1*c2*s3;

	return q;
}

PSVRQuatf PSVR_QuatfAdd(const PSVRQuatf *a, const PSVRQuatf *b)
{
	return {a->w + b->w, a->x + b->x, a->y + b->y, a->z + b->z};
}

PSVRQuatf PSVR_QuatfScale(const PSVRQuatf *q, const float s)
{
	return {q->w*s, q->x*s, q->y*s, q->z*s};
}

PSVRQuatf PSVR_QuatfMultiply(const PSVRQuatf *a, const PSVRQuatf *b)
{
	return {a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z,
			a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y,
			a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x,
			a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w};
}

PSVRQuatf PSVR_QuatfUnsafeScalarDivide(const PSVRQuatf *q, const float s)
{
	return {q->w / s, q->x / s, q->y / s, q->z / s};
}

PSVRQuatf PSVR_QuatfSafeScalarDivide(const PSVRQuatf *q, const float s, const PSVRQuatf *default_result)
{
	return !is_nearly_zero(s) ? PSVR_QuatfUnsafeScalarDivide(q, s) : *default_result;
}

PSVRQuatf PSVR_QuatfConjugate(const PSVRQuatf *q)
{
	return {q->w, -q->x, -q->y, -q->z};
}

PSVRQuatf PSVR_QuatfConcat(const PSVRQuatf *first, const PSVRQuatf *second)
{
	return PSVR_QuatfMultiply(second, first);
}

PSVRVector3f PSVR_QuatfRotateVector(const PSVRQuatf *q, const PSVRVector3f *v)
{
	return {q->w*q->w*v->x + 2*q->y*q->w*v->z - 2*q->z*q->w*v->y + q->x*q->x*v->x + 2*q->y*q->x*v->y + 2*q->z*q->x*v->z - q->z*q->z*v->x - q->y*q->y*v->x,
			2*q->x*q->y*v->x + q->y*q->y*v->y + 2*q->z*q->y*v->z + 2*q->w*q->z*v->x - q->z*q->z*v->y + q->w*q->w*v->y - 2*q->x*q->w*v->z - q->x*q->x*v->y,
			2*q->x*q->z*v->x + 2*q->y*q->z*v->y + q->z*q->z*v->z - 2*q->w*q->y*v->x - q->y*q->y*v->z + 2*q->w*q->x*v->y - q->x*q->x*v->z + q->w*q->w*v->z};
}

float PSVR_QuatfLength(const PSVRQuatf *q)
{
    return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

PSVRQuatf PSVR_QuatfNormalizeWithDefault(const PSVRQuatf *q, const PSVRQuatf *default_result)
{
	return PSVR_QuatfSafeScalarDivide(q, PSVR_QuatfLength(q), default_result);
}

// PSVRMatrix3d Methods
PSVRMatrix3d PSVR_Matrix3dCreate(const PSVRVector3d *basis_x, const PSVRVector3d *basis_y, const PSVRVector3d *basis_z)
{
    PSVRMatrix3d mat;

    mat.m[0][0] = basis_x->x; mat.m[0][1] = basis_x->y; mat.m[0][2] = basis_x->z;
    mat.m[1][0] = basis_y->x; mat.m[1][1] = basis_y->y; mat.m[1][2] = basis_y->z;
    mat.m[2][0] = basis_z->x; mat.m[2][1] = basis_z->y; mat.m[2][2] = basis_z->z;

    return mat;
}

PSVRVector3d PSVR_Matrix3dBasisX(const PSVRMatrix3d *mat)
{
    return {mat->m[0][0], mat->m[0][1], mat->m[0][2]};
}

PSVRVector3d PSVR_Matrix3dBasisY(const PSVRMatrix3d *mat)
{
	return {mat->m[1][0], mat->m[1][1], mat->m[1][2]};
}

PSVRVector3d PSVR_Matrix3dBasisZ(const PSVRMatrix3d *mat)
{
	return {mat->m[2][0], mat->m[2][1], mat->m[2][2]};
}

// PSVRMatrix3f Methods
PSVRMatrix3f PSVR_Matrix3fCreate(const PSVRVector3f *basis_x, const PSVRVector3f *basis_y, const PSVRVector3f *basis_z)
{
    PSVRMatrix3f mat;

    mat.m[0][0] = basis_x->x; mat.m[0][1] = basis_x->y; mat.m[0][2] = basis_x->z;
    mat.m[1][0] = basis_y->x; mat.m[1][1] = basis_y->y; mat.m[1][2] = basis_y->z;
    mat.m[2][0] = basis_z->x; mat.m[2][1] = basis_z->y; mat.m[2][2] = basis_z->z;

    return mat;
}

PSVRMatrix3f PSVR_Matrix3fCreateFromQuatf(const PSVRQuatf *q)
{
	PSVRMatrix3f mat;

	const float qw = q->w;
	const float qx = q->x;
	const float qy = q->y;
	const float qz = q->z;

	const float qx2 = qx*qx;
	const float qy2 = qy*qy;
	const float qz2 = qz*qz;

	mat.m[0][0] = 1.f - 2.f*qy2 - 2.f*qz2; mat.m[0][1] = 2.f*qx*qy - 2.f*qz*qw;   mat.m[0][2] = 2.f*qx*qz + 2.f*qy*qw;
	mat.m[1][0] = 2.f*qx*qy + 2.f*qz*qw;   mat.m[1][1] = 1.f - 2.f*qx2 - 2.f*qz2; mat.m[1][2] = 2.f*qy*qz - 2.f*qx*qw;
	mat.m[2][0] = 2.f*qx*qz - 2.f*qy*qw;   mat.m[2][1] = 2.f * qy*qz + 2.f*qx*qw; mat.m[2][2] = 1.f - 2.f*qx2 - 2.f*qy2;

	return mat;
}

PSVRVector3f PSVR_Matrix3fBasisX(const PSVRMatrix3f *mat)
{
	return {mat->m[0][0], mat->m[0][1], mat->m[0][2]};
}

PSVRVector3f PSVR_Matrix3fBasisY(const PSVRMatrix3f *mat)
{
	return {mat->m[1][0], mat->m[1][1], mat->m[1][2]};
}

PSVRVector3f PSVR_Matrix3fBasisZ(const PSVRMatrix3f *mat)
{
	return {mat->m[2][0], mat->m[2][1], mat->m[2][2]};
}

// PSVRMatrix4d Methods
PSVRMatrix4d PSVR_Matrix4dCreate(const PSVRVector4d *basis_x, const PSVRVector4d *basis_y, const PSVRVector4d *basis_z, const PSVRVector4d *basis_w)
{
    PSVRMatrix4d mat;

    mat.m[0][0] = basis_x->x; mat.m[0][1] = basis_x->y; mat.m[0][2] = basis_x->z; mat.m[0][3] = basis_x->w;
    mat.m[1][0] = basis_y->x; mat.m[1][1] = basis_y->y; mat.m[1][2] = basis_y->z; mat.m[1][3] = basis_y->w;
    mat.m[2][0] = basis_z->x; mat.m[2][1] = basis_z->y; mat.m[2][2] = basis_z->z; mat.m[2][3] = basis_z->w;
    mat.m[3][0] = basis_w->x; mat.m[3][1] = basis_w->y; mat.m[3][2] = basis_w->z; mat.m[3][3] = basis_w->w;

    return mat;
}

PSVRVector4d PSVR_Matrix4dBasisX(const PSVRMatrix4d *mat)
{
    return {mat->m[0][0], mat->m[0][1], mat->m[0][2], mat->m[0][3]};
}

PSVRVector4d PSVR_Matrix4dBasisY(const PSVRMatrix4d *mat)
{
    return {mat->m[1][0], mat->m[1][1], mat->m[1][2], mat->m[1][3]};
}

PSVRVector4d PSVR_Matrix4dBasisZ(const PSVRMatrix4d *mat)
{
    return {mat->m[2][0], mat->m[2][1], mat->m[2][2], mat->m[2][3]};
}

PSVRVector4d PSVR_Matrix4dBasisW(const PSVRMatrix4d *mat)
{
    return {mat->m[3][0], mat->m[3][1], mat->m[3][2], mat->m[3][3]};
}

// PSVRPosef
PSVRPosef PSVR_PosefCreate(const PSVRVector3f *position, const PSVRQuatf *orientation)
{
	return {*position, *orientation};
}

PSVRPosef PSVR_PosefInverse(const PSVRPosef *pose)
{
	PSVRQuatf q_inv = PSVR_QuatfConjugate(&pose->Orientation);
	PSVRPosef result;

	result.Orientation = q_inv;
	result.Position = PSVR_QuatfRotateVector(&q_inv, &pose->Position);
	result.Position = PSVR_Vector3fScale(&result.Position, -1.f);

	return result;
}

PSVRPosef PSVR_PosefConcat(const PSVRPosef *first, const PSVRPosef *second)
{
	PSVRPosef result;

	result.Orientation = PSVR_QuatfConcat(&first->Orientation, &second->Orientation);
	result.Position = PSVR_QuatfRotateVector(&second->Orientation, &first->Position);
    result.Position = PSVR_Vector3fAdd(&result.Position, &second->Position);

	return result;
}

PSVRVector3f PSVR_PosefTransformPoint(const PSVRPosef *pose, const PSVRVector3f *p)
{
	PSVRVector3f result= PSVR_QuatfRotateVector(&pose->Orientation, p);
	result= PSVR_Vector3fAdd(&result, &pose->Position);

	return result;
}

PSVRVector3f PSVR_PosefInverseTransformPoint(const PSVRPosef *pose, const PSVRVector3f *p)
{
	PSVRQuatf q_inv = PSVR_QuatfConjugate(&pose->Orientation);
	PSVRVector3f unrotate_p= PSVR_QuatfRotateVector(&q_inv, p);
	PSVRVector3f unrotate_pose_position= PSVR_QuatfRotateVector(&q_inv, &pose->Position);
	PSVRVector3f result = PSVR_Vector3fSubtract(&unrotate_p, &unrotate_pose_position);

	return result;
}

// PSVRFrustumf
void PSVR_FrustumSetPose(PSVRFrustum *frustum, const PSVRPosef *pose)
{
    const glm::quat orientation(pose->Orientation.w, pose->Orientation.x, pose->Orientation.y, pose->Orientation.z);
    const glm::vec3 position(pose->Position.x, pose->Position.y, pose->Position.z);
    const glm::mat4 rot = glm::mat4_cast(orientation);
    const glm::mat4 trans = glm::translate(glm::mat4(1.0f), position);
    const glm::mat4 glm_mat4 = trans * rot;

    frustum->forward = {glm_mat4[2].x, glm_mat4[2].y, glm_mat4[2].z}; // z-axis
    frustum->left = {glm_mat4[0].x, glm_mat4[0].y, glm_mat4[0].z}; // x-axis
    frustum->up = {glm_mat4[1].x, glm_mat4[1].y, glm_mat4[1].z}; // y-axis

    frustum->origin = {glm_mat4[3].x, glm_mat4[3].y, glm_mat4[3].z};
}

// -- PSVRTrackingProjection -- 
float PSVR_TrackingProjectionGetArea(const PSVRTrackingProjection *proj, const PSVRTrackingProjectionCount area_index)
{
	float area = 0.f;

	switch (proj->shape_type)
	{
	case PSVRShape_Ellipse:
		{
			area = k_real_pi
                *proj->projections[area_index].shape.ellipse.half_x_extent
                *proj->projections[area_index].shape.ellipse.half_y_extent;
		} break;
	case PSVRShape_LightBar:
		{
			PSVRVector2f edge1 = 
                PSVR_Vector2fSubtract(
                    &proj->projections[area_index].shape.lightbar.quad[0],
                    &proj->projections[area_index].shape.lightbar.quad[1]);
			PSVRVector2f edge2 = 
                PSVR_Vector2fSubtract(
                    &proj->projections[area_index].shape.lightbar.quad[0],
                    &proj->projections[area_index].shape.lightbar.quad[3]);

			area = PSVR_Vector2fLength(&edge1)*PSVR_Vector2fLength(&edge2);
		} break;
	}

	return area;
}

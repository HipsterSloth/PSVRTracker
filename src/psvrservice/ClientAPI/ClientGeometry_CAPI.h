/**
\file
*/ 

#ifndef __CLIENTGEOMETRY_CAPI_H
#define __CLIENTGEOMETRY_CAPI_H
#include "PSVRClient_export.h"
#include <stdbool.h>
//cut_before

/** 
\brief Geometrical data structures and functions used by the Client API  
\defgroup Geometry_CAPI Client Geometry
\addtogroup Geometry_CAPI 
@{ 
*/

//-- constants -----
/// Conversion factor to go from meters to centimeters
#define PSVR_METERS_TO_CENTIMETERS  100.f

/// Conversion factor to go from centimeters to meters
#define PSVR_CENTIMETERS_TO_METERS  0.01f

/// Conversion factor to go from millimeters to centimeters
#define PSVR_MILLIMETERS_TO_CENTIMETERS  0.1f

/// Conversion factor to go from centimeters to millimeters
#define PSVR_CENTIMETERS_TO_MILLIMETERS  10.f

/// A 2D vector with float components.
typedef struct
{
    float x, y;
} PSVRVector2f;

/// A 3D vector with double components.
typedef struct
{
    double x, y, z;
} PSVRVector3d;

/// A 3D vector with float components.
typedef struct
{
    float x, y, z;
} PSVRVector3f;

/// A 3D vector with int components.
typedef struct
{
    int x, y, z;
} PSVRVector3i;

/// A 4D vector with double components.
typedef struct
{
    double x, y, z, w;
} PSVRVector4d;

/** A 3x3 matrix with float components
	storage is row major order: [x0,x1,x2,y0,y1,y1,z0,z1,z2]
 */
typedef struct
{
    float m[3][3]; 
} PSVRMatrix3f;

/** A 3x3 matrix with double components
	storage is row major order: [x0,x1,x2,y0,y1,y2,z0,z1,z2]
 */
typedef struct
{
    double m[3][3]; 
} PSVRMatrix3d;

/** A 3x4 matrix with double components
	storage is row major order: [x0,x1,x2,x3,y0,y1,y2,y3,z0,z1,z2,z3]
 */
typedef struct
{
    double m[3][4]; 
} PSVRMatrix34d;

/** A 4x4 matrix with double components
	storage is row major order: [x0,x1,x2,x3,y0,y1,y2,y3,z0,z1,z2,z3,w0,w1,w2,w3]
 */
typedef struct
{
    double m[4][4]; 
} PSVRMatrix4d;

/// A quaternion rotation.
typedef struct
{
    float w, x, y, z;
} PSVRQuatf;

/// Position and orientation together.
typedef struct
{
    PSVRVector3f  Position;
    PSVRQuatf     Orientation;
} PSVRPosef;

/// A camera frustum
typedef struct
{
    PSVRVector3f origin; 	///< frustum tip world location, in cm
    PSVRVector3f forward; 	///< forward axis of the frustum
	PSVRVector3f left; 		///< left axis of the frustum
	PSVRVector3f up; 		///< up axis of the frustum
    float HFOV; 			///< horizontal field of view, in radians
	float VFOV; 			///< vertical field of fiew, in radians
    float zNear; 			///< near plane distance of frustum, in cm
	float zFar; 			///< far place distance of frustum, in cm
} PSVRFrustum;

typedef enum 
{
	TRIANGLE_POINT_COUNT = 3,
	QUAD_POINT_COUNT = 4,
	MAX_POINT_CLOUD_POINT_COUNT= 9,

	TRI_VERTEX_LOWER_RIGHT= 0,
	TRI_VERTEX_LOWER_LEFT= 1,
	TRI_VERTEX_UPPER_MIDDLE= 2,

	QUAD_VERTEX_UPPER_RIGHT= 0,
	QUAD_VERTEX_UPPER_LEFT= 1,
	QUAD_VERTEX_LOWER_LEFT= 2,
	QUAD_VERTEX_LOWER_RIGHT= 3
} PSVRTrackingShapeConstants;

typedef struct
{
    union{
        struct {
            PSVRVector2f center;
            float half_x_extent;
            float half_y_extent;
            float angle;
            PSVRVector3f source_position; // The source sphere that the ellipse is a projection of
            float source_radius;
        } ellipse;
        struct {
            PSVRVector2f triangle[TRIANGLE_POINT_COUNT];
			PSVRVector2f quad[QUAD_POINT_COUNT];
        } lightbar;
		struct {
			PSVRVector2f points[MAX_POINT_CLOUD_POINT_COUNT];
			int point_count;
		} pointcloud;
    } shape;

    float screen_area; // area in pixels^2
} PSVRTrackingProjectionData;

typedef enum
{
    LEFT_PROJECTION_INDEX = 0,
    RIGHT_PROJECTION_INDEX = 1,

    MONO_PROJECTION_COUNT = 1,
    STEREO_PROJECTION_COUNT = 2,

    MAX_PROJECTION_COUNT = 2,
    PRIMARY_PROJECTION_INDEX = LEFT_PROJECTION_INDEX,
} PSVRTrackingProjectionCount;


typedef enum 
{
    PSVRShape_INVALID_PROJECTION = -1,
    PSVRShape_Ellipse,					///< The 2D projection of a sphere (think conic section)
    PSVRShape_LightBar,					///< The 2D projection of a 3D quad (bounding shape of DS4 lightbar) 
    PSVRShape_PointCloud					///< The 2D projection of a 3D point cloud (Morpheus tracking lights)
} PSVRProjectionShapeType;


/// The projection of a tracking shape onto the image plane of a mono or stereo tracker video feed
typedef struct
{
    PSVRTrackingProjectionData projections[MAX_PROJECTION_COUNT];
    PSVRTrackingProjectionCount projection_count;
    PSVRProjectionShapeType shape_type;
} PSVRTrackingProjection;

typedef enum 
{
    PSVRTrackingShape_INVALID = -1,
    PSVRTrackingShape_Sphere,
    PSVRTrackingShape_LightBar,
    PSVRTrackingShape_PointCloud,
    PSVRTrackingShape_MAX
} PSVRTrackingShapeType;

/// A tracking shape of a controller or HMD
typedef struct
{
    union{
        struct {
            float radius;
        } sphere;
        struct {
            PSVRVector3f triangle[TRIANGLE_POINT_COUNT];
			PSVRVector3f quad[QUAD_POINT_COUNT];
        } lightbar;
		struct {
			PSVRVector3f points[MAX_POINT_CLOUD_POINT_COUNT];
			int point_count;
		} pointcloud;
    } shape;
    PSVRTrackingShapeType shape_type;
} PSVRTrackingShape;

// Interface
//----------

// PSVRVector2f Methods

/// Adds two 2D vectors together
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fAdd(const PSVRVector2f *a, const PSVRVector2f *b);
/// Subtracts one 2D vector from another 2D vector
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fSubtract(const PSVRVector2f *a, const PSVRVector2f *b);
/// Scales a 2D vector by a scalar
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fScale(const PSVRVector2f *v, const float s);
/// Scales a 2D vector by a scalar and then adds a vector to the scaled result
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fScaleAndAdd(const PSVRVector2f *v, const float s, const PSVRVector2f *b);
/// Divides each component of a 2D vector by a scalar without checking for divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fUnsafeScalarDivide(const PSVRVector2f *numerator, const float divisor);
/// Divides each component of a 2D vector by the corresponding component of another vector without checking for a divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fUnsafeVectorDivide(const PSVRVector2f *numerator, const PSVRVector2f *divisor);
/// Divides each component of a 2D vector by a scalar, returning a default vector in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fSafeScalarDivide(const PSVRVector2f *numerator, const float divisor, const PSVRVector2f *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fSafeVectorDivide(const PSVRVector2f *numerator, const PSVRVector2f *divisor, const PSVRVector2f *default_result);
/// Computes the absolute value of each component
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fAbs(const PSVRVector2f *v);
/// Squares each component of a vector
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fSquare(const PSVRVector2f *v);
/// Computes the length of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fLength(const PSVRVector2f *v);
/// Computes the normalized version of a vector, returning a default vector in the event of a near zero length vector
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fNormalizeWithDefault(const PSVRVector2f *v, const PSVRVector2f *default_result);
/// Computes the minimum component of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fMinValue(const PSVRVector2f *v);
/// Computes the maximum component of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fMaxValue(const PSVRVector2f *v);
/// Computes the 2D dot product of two vectors
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fDot(const PSVRVector2f *a, const PSVRVector2f *b);
/// Computes the 2D squared distance between two vectors
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fDistanceSquared(const PSVRVector2f *a, const PSVRVector2f *b);
/// Computes the 2D squared between two vectors
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector2fDistance(const PSVRVector2f *a, const PSVRVector2f *b);
/// Computes the min value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fMin(const PSVRVector2f *a, const PSVRVector2f *b);
/// Computes the max value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector2f) PSVR_Vector2fMax(const PSVRVector2f *a, const PSVRVector2f *b);
/// Translate an array of points in the given direction
PSVR_PUBLIC_FUNCTION(void) PSVR_Vector2fArrayTranslate(
    const PSVRVector2f *in_points, const int point_count,
    const PSVRVector2f *direction, const float scale, 
    PSVRVector2f *out_points);

// PSVRVector3f Methods

/// Adds two 3D vectors together
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fAdd(const PSVRVector3f *a, const PSVRVector3f *b);
/// Subtracts one 3D vector from another 3D vector
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fSubtract(const PSVRVector3f *a, const PSVRVector3f *b);
/// Scales a 3D vector by a scalar
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fScale(const PSVRVector3f *v, const float s);
/// Scales a 3D vector by a scalar and then adds a vector to the scaled result
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fScaleAndAdd(const PSVRVector3f *v, const float s, const PSVRVector3f *b);
/// Divides each component of a 3D vector by a scalar without checking for divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fUnsafeScalarDivide(const PSVRVector3f *numerator, const float divisor);
/// Divides each component of a 3D vector by the corresponding componenet of another vector without checking for a divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fUnsafeVectorDivide(const PSVRVector3f *numerator, const PSVRVector3f *divisor);
/// Divides each component of a 3D vector by a scalar, returning a default vector in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fSafeScalarDivide(const PSVRVector3f *numerator, const float divisor, const PSVRVector3f *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fSafeVectorDivide(const PSVRVector3f *numerator, const PSVRVector3f *divisor, const PSVRVector3f *default_result);
/// Computes the absolute value of each component
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fAbs(const PSVRVector3f *v);
/// Squares each component of a vector
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fSquare(const PSVRVector3f *v);
/// Computes the length of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector3fLength(const PSVRVector3f *v);
/// Computes the normalized version of a vector, returning a default vector in the event of a near zero length vector
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fNormalizeWithDefault(const PSVRVector3f *v, const PSVRVector3f *default_result);
/// Computes the normalized version of a vector and its original length, returning a default vector in the event of a near zero length vector
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fNormalizeWithDefaultGetLength(const PSVRVector3f *v, const PSVRVector3f *default_result, float *out_length);
/// Computes the minimum component of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector3fMinValue(const PSVRVector3f *v);
/// Computes the maximum component of a given vector
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector3fMaxValue(const PSVRVector3f *v);
/// Computes the 3D dot product of two vectors
PSVR_PUBLIC_FUNCTION(float) PSVR_Vector3fDot(const PSVRVector3f *a, const PSVRVector3f *b);
/// Compute the 3D cross product of two vectors
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fCross(const PSVRVector3f *a, const PSVRVector3f *b);
/// Computes the min value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fMin(const PSVRVector3f *a, const PSVRVector3f *b);
/// Computes the max value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3fMax(const PSVRVector3f *a, const PSVRVector3f *b);

// PSVRVector3i Methods

/// Adds two 3D vectors together
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iAdd(const PSVRVector3i *a, const PSVRVector3i *b);
/// Subtracts one 3D vector from another 3D vector
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iSubtract(const PSVRVector3i *a, const PSVRVector3i *b);
/// Divides each component of a 3D vector by a scalar without checking for divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iUnsafeScalarDivide(const PSVRVector3i *numerator, const int divisor);
/// Divides each component of a 3D vector by the corresponding componenet of another vector without checking for a divide-by-zero
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iUnsafeVectorDivide(const PSVRVector3i *numerator, const PSVRVector3i *divisor);
/// Divides each component of a 3D vector by a scalar, returning a default vector in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iSafeScalarDivide(const PSVRVector3i *numerator, const int divisor, const PSVRVector3i *default_result);
/// Divides each component of a 2D vector by another vector, returning a default value for each component in the case of divide by zero
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iSafeVectorDivide(const PSVRVector3i *numerator, const PSVRVector3i *divisor, const PSVRVector3i *default_result);
/// Computes the absolute value of each component
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iAbs(const PSVRVector3i *v);
/// Squares each component of a vector
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iSquare(const PSVRVector3i *v);
/// Computes the squared-length of a given vector
PSVR_PUBLIC_FUNCTION(int) PSVR_Vector3iLengthSquared(const PSVRVector3i *v);
/// Computes the minimum component of a given vector
PSVR_PUBLIC_FUNCTION(int) PSVR_Vector3iMinValue(const PSVRVector3i *v);
/// Computes the maximum component of a given vector
PSVR_PUBLIC_FUNCTION(int) PSVR_Vector3iMaxValue(const PSVRVector3i *v);
/// Computes the min value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iMin(const PSVRVector3i *a, const PSVRVector3i *b);
/// Computes the max value of two vectors along each component
PSVR_PUBLIC_FUNCTION(PSVRVector3i) PSVR_Vector3iMax(const PSVRVector3i *a, const PSVRVector3i *b);
/// Convertes a 3D int vector to a 3D float vector
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Vector3iCastToFloat(const PSVRVector3i *v);

// PSVRQuatf Methods

/// Construct a quaternion from raw w, x, y, and z components
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfCreate(float w, float x, float y, float z);
/// Construct a quaternion rotation from rotations about the X, Y, and Z axis
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfCreateFromAngles(const PSVRVector3f *eulerAngles);
/// Component-wise add two quaternions together (used by numerical integration)
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfAdd(const PSVRQuatf *a, const PSVRQuatf *b);
/// Scale all components of a quaternion by a scalar (used by numerical integration)
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfScale(const PSVRQuatf *q, const float s);
/// Compute the multiplication of two quaterions
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfMultiply(const PSVRQuatf *a, const PSVRQuatf *b);
/// Divide all components of a quaternion by a scalar without checking for divide by zero
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfUnsafeScalarDivide(const PSVRQuatf *q, const float s);
/// Divide all components of a quaternion by a scalar, returning a default quaternion in case of a degenerate quaternion
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfSafeScalarDivide(const PSVRQuatf *q, const float s, const PSVRQuatf *default_result);
/// Compute the complex conjegate of a quaternion (negate imaginary components)
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfConjugate(const PSVRQuatf *q);
/// Concatenate a second quaternion's rotation on to the end of a first quaternion's quaterion (just a quaternion multiplication)
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfConcat(const PSVRQuatf *first, const PSVRQuatf *second);
/// Rotate a vector by a given quaternion
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_QuatfRotateVector(const PSVRQuatf *q, const PSVRVector3f *v);
/// Compute the length of a quaternion (sum of squared components)
PSVR_PUBLIC_FUNCTION(float) PSVR_QuatfLength(const PSVRQuatf *q);
/// Computes the normalized version of a quaternion, returning a default quaternion in the event of a near zero length quaternion
PSVR_PUBLIC_FUNCTION(PSVRQuatf) PSVR_QuatfNormalizeWithDefault(const PSVRQuatf *q, const PSVRQuatf *default_result);

// PSVRMatrix3d Methods
/// Create a 3x3 matrix from a set of 3 basis vectors (might not be ortho-normal)
PSVR_PUBLIC_FUNCTION(PSVRMatrix3d) PSVR_Matrix3dCreate(const PSVRVector3d *basis_x, const PSVRVector3d *basis_y, const PSVRVector3d *basis_z);
/// Extract the x-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3d) PSVR_Matrix3dBasisX(const PSVRMatrix3d *m);
/// Extract the y-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3d) PSVR_Matrix3dBasisY(const PSVRMatrix3d *m);
/// Extract the z-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3d) PSVR_Matrix3dBasisZ(const PSVRMatrix3d *m);

// PSVRMatrix3f Methods
/// Create a 3x3 matrix from a set of 3 basis vectors (might not be ortho-normal)
PSVR_PUBLIC_FUNCTION(PSVRMatrix3f) PSVR_Matrix3fCreate(const PSVRVector3f *basis_x, const PSVRVector3f *basis_y, const PSVRVector3f *basis_z);
/// Create a 3x3 rotation matrix from a quaternion
PSVR_PUBLIC_FUNCTION(PSVRMatrix3f) PSVR_Matrix3fCreateFromQuatf(const PSVRQuatf *q);
/// Extract the x-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Matrix3fBasisX(const PSVRMatrix3f *m);
/// Extract the y-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Matrix3fBasisY(const PSVRMatrix3f *m);
/// Extract the z-axis basis vector from a 3x3 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_Matrix3fBasisZ(const PSVRMatrix3f *m);

// PSVRMatrix4d Methods
/// Create a 4x4 matrix from a set of 4 basis vectors (might not be ortho-normal)
PSVR_PUBLIC_FUNCTION(PSVRMatrix4d) PSVR_Matrix4dCreate(const PSVRVector4d *basis_x, const PSVRVector4d *basis_y, const PSVRVector4d *basis_z, const PSVRVector4d *basis_w);
/// Extract the x-axis basis vector from a 4x4 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector4d) PSVR_Matrix4dBasisX(const PSVRMatrix4d *m);
/// Extract the y-axis basis vector from a 4x4 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector4d) PSVR_Matrix4dBasisY(const PSVRMatrix4d *m);
/// Extract the z-axis basis vector from a 4x4 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector4d) PSVR_Matrix4dBasisZ(const PSVRMatrix4d *m);
/// Extract the w-axis basis vector from a 4x4 matrix
PSVR_PUBLIC_FUNCTION(PSVRVector4d) PSVR_Matrix4dBasisW(const PSVRMatrix4d *m);

// PSVRPosef
/// Create a pose from a given position and orientation
PSVR_PUBLIC_FUNCTION(PSVRPosef) PSVR_PosefCreate(const PSVRVector3f *position, const PSVRQuatf *orientation);
/// Create a pose that inverts the transform (rotation and translation) of a given pose
PSVR_PUBLIC_FUNCTION(PSVRPosef) PSVR_PosefInverse(const PSVRPosef *pose);
/// Concatenate the transformation of one pose onto the transformation of another pose
PSVR_PUBLIC_FUNCTION(PSVRPosef) PSVR_PosefConcat(const PSVRPosef *first, const PSVRPosef *second);
/// Transform point by a pose
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_PosefTransformPoint(const PSVRPosef *pose, const PSVRVector3f *p);
/// Transform a point by the inverse of a pose
PSVR_PUBLIC_FUNCTION(PSVRVector3f) PSVR_PosefInverseTransformPoint(const PSVRPosef *pose, const PSVRVector3f *p);

// PSVRFrustumf
/// Update the basis (position and orientation) of a fustum to match that of a given pose
PSVR_PUBLIC_FUNCTION(void) PSVR_FrustumSetPose(PSVRFrustum *frustum, const PSVRPosef *pose);

// PSVRTrackingProjection
/// Compute the area in pixels^2 of a tracking projection
PSVR_PUBLIC_FUNCTION(float) PSVR_TrackingProjectionGetArea(const PSVRTrackingProjection *proj, const PSVRTrackingProjectionCount area_index);

//-- constants -----
/// A 2D float vector whose components are all 0.0f
PSVR_PUBLIC_CLASS extern const PSVRVector2f *k_PSVR_float_vector2_zero;
/// A 3D integer vector whose components are all 0
PSVR_PUBLIC_CLASS extern const PSVRVector3i *k_PSVR_int_vector3_zero;
/// A 3D float vector whose components are all 0.0f
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_float_vector3_zero;
/// A 3D integer vector whose components are all 1
PSVR_PUBLIC_CLASS extern const PSVRVector3i *k_PSVR_int_vector3_one;
/// A 3D float vector whose components are all 1.0f
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_float_vector3_one;
/// The 3D float vector <1.0f, 0.0f, 0.0f>
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_float_vector3_i;
/// The 3D float vector <0.0f, 1.0f, 0.0f>
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_float_vector3_j;
/// The 3D float vector <0.0f, 0.0f, 1.0f>
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_float_vector3_k;
/// A 3D float vector that represents the world origin <0.f, 0.f, 0.f>
PSVR_PUBLIC_CLASS extern const PSVRVector3f *k_PSVR_position_origin;
/// The quaterion <1.f, 0.f, 0.f, 0.f> that represents no rotation
PSVR_PUBLIC_CLASS extern const PSVRQuatf *k_PSVR_quaternion_identity;
/// The 3x3 matrix that represent no transform (diagonal values 1.f, off diagonal values 0.f)
PSVR_PUBLIC_CLASS extern const PSVRMatrix3f *k_PSVR_matrix_identity;
/// The pose that represents no transformation (identity quaternion, zero vector)
PSVR_PUBLIC_CLASS extern const PSVRPosef *k_PSVR_pose_identity;

/** 
@} 
*/ 

//cut_after
#endif

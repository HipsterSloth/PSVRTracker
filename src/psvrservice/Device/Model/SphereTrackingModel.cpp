// -- include -----
#include "SphereTrackingModel.h"
#include "MathUtility.h"
#include "MathEigen.h"
#include "MathTypeConversion.h"
#include "ServerTrackerView.h"
#include "TrackingModelMath.h"

//-- constants ---

//-- private structures ----
struct SphereTrackingModelState
{
    PSVRVector3f spherePosition;
    float sphereRadius;
    bool bIsSpherePositionValid;
};

//-- private methods -----

//-- public implementation -----
SphereTrackingModel::SphereTrackingModel() :
    m_state(new SphereTrackingModelState)
{
}

SphereTrackingModel::~SphereTrackingModel()
{
    delete m_state;
}

bool SphereTrackingModel::init(PSVRTrackingShape *tracking_shape)
{
    bool bSuccess= false;

    if (tracking_shape->shape_type == PSVRTrackingShape_Sphere &&
        tracking_shape->shape.sphere.radius > k_real_epsilon)
    {
        m_state->spherePosition= tracking_shape->shape.sphere.center;
        m_state->sphereRadius= tracking_shape->shape.sphere.radius;
        m_state->bIsSpherePositionValid= false;
        bSuccess= true;
    }

    return bSuccess;
}

bool SphereTrackingModel::applyShapeProjectionFromTracker(
	const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
    const class ServerTrackerView *tracker_view,
	const ShapeTimestampedPose *last_filtered_pose,
    const PSVRTrackingProjection &projection)
{
    bool bSuccess= false;

    if (projection.projection_count == STEREO_PROJECTION_COUNT)
    {
		Eigen::Vector3f left_position= 
			PSVR_vector3f_to_eigen_vector3(
				projection.projections[LEFT_PROJECTION_INDEX].shape.ellipse.source_position);
		Eigen::Vector3f right_position= 
			PSVR_vector3f_to_eigen_vector3(
				projection.projections[RIGHT_PROJECTION_INDEX].shape.ellipse.source_position);

		Eigen::Vector3f triangulated_position;
		compute_triangulation_from_stereo_position_estimates(
			left_position, right_position, tracker_view->getTrackerDevice(),
			triangulated_position);

		m_state->spherePosition= eigen_vector3f_to_PSVR_vector3f(triangulated_position);
		m_state->bIsSpherePositionValid= true;

        bSuccess= true;
    }
    else if (projection.projection_count == MONO_PROJECTION_COUNT)
    {
        // Use the position derived from the best fit cone to sphere
        m_state->spherePosition= projection.projections[PRIMARY_PROJECTION_INDEX].shape.ellipse.source_position;
        m_state->bIsSpherePositionValid= true;

        bSuccess= true;
    }

    return bSuccess;
}

bool SphereTrackingModel::getShapeOrientation(PSVRQuatf &out_orientation) const
{
    // Sphere projections never have an orientation
    return false;
}

bool SphereTrackingModel::getShapePosition(PSVRVector3f &out_position) const
{
    if (m_state->bIsSpherePositionValid)
    {
        out_position= m_state->spherePosition;
    }

    return m_state->bIsSpherePositionValid;
}

bool SphereTrackingModel::getShape(PSVRTrackingShape &out_shape) const
{
    if (m_state->bIsSpherePositionValid)
    {
        out_shape.shape_type= PSVRTrackingShape_Sphere;
        out_shape.shape.sphere.radius= m_state->sphereRadius;
        out_shape.shape.sphere.center= m_state->spherePosition;
    }

    return m_state->bIsSpherePositionValid;
}
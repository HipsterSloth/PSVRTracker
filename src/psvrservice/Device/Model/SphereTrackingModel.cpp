// -- include -----
#include "SphereTrackingModel.h"
#include "MathUtility.h"

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
    const class ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &projection)
{
    bool bSuccess= false;

    if (projection.projection_count == STEREO_PROJECTION_COUNT)
    {
        //###HipsterSloth $TODO - Properly triangulate using stereo projection
        m_state->spherePosition= projection.projections[0].shape.ellipse.source_position;
        m_state->bIsSpherePositionValid= true;

        bSuccess= true;
    }
    else if (projection.projection_count == MONO_PROJECTION_COUNT)
    {
        // Use the position derived from the best fit cone to sphere
        m_state->spherePosition= projection.projections[0].shape.ellipse.source_position;
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
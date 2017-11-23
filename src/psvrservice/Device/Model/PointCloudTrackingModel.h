#ifndef POINT_CLOUD_TRACKING_MODEL_INTERFACE_H
#define POINT_CLOUD_TRACKING_MODEL_INTERFACE_H

// -- include -----
#include "ShapeTrackingModelInterface.h"

// -- public interface -----
class PointCloudTrackingModel : public IShapeTrackingModel
{
public:
	PointCloudTrackingModel();
	virtual ~PointCloudTrackingModel();

    bool init(PSVRTrackingShape *tracking_shape) override;
	bool applyShapeProjectionFromTracker(
        const class ServerTrackerView *tracker_view, 
        const PSVRTrackingProjection &projection) override;
    bool getShapeOrientation(PSVRQuatf &out_orientation) const override;
    bool getShapePosition(PSVRVector3f &out_position) const override;
    bool getShape(PSVRTrackingShape &out_shape) const override;

private:
    struct PointCloudTrackingModelState *m_state;
};

#endif // SHAPE_TRACKING_MODEL_INTERFACE_H
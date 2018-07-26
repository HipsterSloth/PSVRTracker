#ifndef SPHERE_TRACKING_MODEL_INTERFACE_H
#define SPHERE_TRACKING_MODEL_INTERFACE_H

// -- include -----
#include "ShapeTrackingModelInterface.h"

// -- public interface -----
class SphereTrackingModel : public IShapeTrackingModel
{
public:
	SphereTrackingModel();
	virtual ~SphereTrackingModel();

    bool init(PSVRTrackingShape *tracking_shape) override;
	bool applyShapeProjectionFromTracker(
		const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
        const class ServerTrackerView *tracker_view,
		const ShapeTimestampedPose *last_filtered_pose,
        const PSVRTrackingProjection &projection) override;
    bool getShapeOrientation(PSVRQuatf &out_orientation) const override;
    bool getShapePosition(PSVRVector3f &out_position) const override;
    bool getShape(PSVRTrackingShape &out_shape) const override;

private:
    struct SphereTrackingModelState *m_state;
};

#endif // SPHERE_TRACKING_MODEL_INTERFACE_H
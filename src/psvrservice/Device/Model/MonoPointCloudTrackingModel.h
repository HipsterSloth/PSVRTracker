#ifndef MONO_POINT_CLOUD_TRACKING_MODEL_INTERFACE_H
#define MONO_POINT_CLOUD_TRACKING_MODEL_INTERFACE_H

// -- include -----
#include "ShapeTrackingModelInterface.h"

// -- public interface -----
class MonoPointCloudTrackingModel : public IShapeTrackingModel
{
public:
	MonoPointCloudTrackingModel();
	virtual ~MonoPointCloudTrackingModel();

    bool init(PSVRTrackingShape *tracking_shape) override;
	bool applyShapeProjectionFromTracker(
		const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
        const class ServerTrackerView *tracker_view, 
        const PSVRTrackingProjection &projection) override;
    bool getShapeOrientation(PSVRQuatf &out_orientation) const override;
    bool getShapePosition(PSVRVector3f &out_position) const override;
    bool getShape(PSVRTrackingShape &out_shape) const override;
	bool getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const;

private:
    struct MonoPointCloudTrackingModelState *m_state;
};

#endif // MONO_POINT_CLOUD_TRACKING_MODEL_INTERFACE_H
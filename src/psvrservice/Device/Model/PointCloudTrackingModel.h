#ifndef CLOUD_TRACKING_MODEL_INTERFACE_H
#define CLOUD_TRACKING_MODEL_INTERFACE_H

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
		const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
        const class ServerTrackerView *tracker_view, 
		const ShapeTimestampedPose *last_filtered_pose,
        const PSVRTrackingProjection &projection) override;
    bool getShapeOrientation(PSVRQuatf &out_orientation) const override;
    bool getShapePosition(PSVRVector3f &out_position) const override;
    bool getShape(PSVRTrackingShape &out_shape) const override;
	bool getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const;

private:
	class IShapeTrackingModel *m_currentModel;
    class MonoPointCloudTrackingModel *m_monoModel;
	class StereoPointCloudTrackingModel *m_stereoModel;
};

#endif // POINT_CLOUD_TRACKING_MODEL_INTERFACE_H
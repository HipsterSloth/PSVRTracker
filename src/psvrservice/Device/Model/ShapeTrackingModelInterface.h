#ifndef SHAPE_TRACKING_MODEL_INTERFACE_H
#define SHAPE_TRACKING_MODEL_INTERFACE_H

// -- include -----
#include "PSVRClient_CAPI.h"
#include <chrono>

// -- interfaces -----
struct ShapeTimestampedPose
{
	std::chrono::time_point<std::chrono::high_resolution_clock> timestamp;
	PSVRPosef pose_cm;
	bool bIsValid;

	ShapeTimestampedPose()
	{
		pose_cm= *k_PSVR_pose_identity;
		bIsValid= false;
	}
};

class IShapeTrackingModel
{
public:
	IShapeTrackingModel() {}
	virtual ~IShapeTrackingModel() {}

    virtual bool init(PSVRTrackingShape *TrackingShape) = 0;
	virtual bool applyShapeProjectionFromTracker(
		const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
        const class ServerTrackerView *tracker_view,
		const ShapeTimestampedPose *last_filtered_pose,
        const PSVRTrackingProjection &projection) = 0;
    virtual bool getShapeOrientation(PSVRQuatf &out_orientation) const = 0;
    virtual bool getShapePosition(PSVRVector3f &out_position) const = 0;
    virtual bool getShape(PSVRTrackingShape &out_shape) const = 0;
};

#endif // SHAPE_TRACKING_MODEL_INTERFACE_H
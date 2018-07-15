// -- include -----
#include "PointCloudTrackingModel.h"
#include "MonoPointCloudTrackingModel.h"
#include "StereoPointCloudTrackingModel.h"
#include "ServerTrackerView.h"

//-- public implementation -----
PointCloudTrackingModel::PointCloudTrackingModel()
	: m_currentModel(nullptr)
	, m_monoModel(new MonoPointCloudTrackingModel)
	, m_stereoModel(new StereoPointCloudTrackingModel)
{
}

PointCloudTrackingModel::~PointCloudTrackingModel()
{
    delete m_monoModel;
	delete m_stereoModel;
}

bool PointCloudTrackingModel::init(PSVRTrackingShape *tracking_shape)
{
    return m_monoModel->init(tracking_shape) && m_stereoModel->init(tracking_shape);
}

bool PointCloudTrackingModel::applyShapeProjectionFromTracker(
	const std::chrono::time_point<std::chrono::high_resolution_clock> &now,
    const class ServerTrackerView *tracker_view,
    const PSVRTrackingProjection &projection)
{
    bool bSuccess= false;

	if (projection.projection_count == MONO_PROJECTION_COUNT)
	{
		m_currentModel= m_monoModel;
		bSuccess= m_monoModel->applyShapeProjectionFromTracker(now, tracker_view, projection);
	}
	else if (projection.projection_count == STEREO_PROJECTION_COUNT)
	{
		m_currentModel= m_stereoModel;
		bSuccess= m_stereoModel->applyShapeProjectionFromTracker(now, tracker_view, projection);
	}

    return bSuccess;
}

bool PointCloudTrackingModel::getShapeOrientation(PSVRQuatf &out_orientation) const
{
    bool bSuccess= false;

	if (m_currentModel != nullptr)
	{
		bSuccess= m_currentModel->getShapeOrientation(out_orientation);
	}

    return bSuccess;
}

bool PointCloudTrackingModel::getShapePosition(PSVRVector3f &out_position) const
{
    bool bSuccess= false;

	if (m_currentModel != nullptr)
	{
		bSuccess= m_currentModel->getShapePosition(out_position);
	}

    return bSuccess;
}

bool PointCloudTrackingModel::getShape(PSVRTrackingShape &out_shape) const
{
    bool bSuccess= false;

	if (m_currentModel != nullptr)
	{
		bSuccess= m_currentModel->getShape(out_shape);
	}

    return bSuccess;
}

bool PointCloudTrackingModel::getPointCloudProjectionShapeCorrelation(PSVRTrackingProjection &projection) const
{
    bool bSuccess= false;

	if (m_currentModel == m_monoModel)
	{
		bSuccess= m_monoModel->getPointCloudProjectionShapeCorrelation(projection);
	}
	else if (m_currentModel == m_stereoModel)
	{
		bSuccess= m_stereoModel->getPointCloudProjectionShapeCorrelation(projection);
	}

    return bSuccess;
}

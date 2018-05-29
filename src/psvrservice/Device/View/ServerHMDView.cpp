//-- includes -----
#include "DeviceManager.h"
#include "ServerHMDView.h"
#include "MathTypeConversion.h"
#include "MathAlignment.h"
#include "MorpheusHMD.h"
#include "VirtualHMD.h"
#include "CompoundPoseFilter.h"
#include "KalmanPoseFilter.h"
#include "PoseFilterInterface.h"
#include "Logger.h"
#include "PointCloudTrackingModel.h"
#include "SphereTrackingModel.h"
#include "ServiceRequestHandler.h"
#include "ServerTrackerView.h"
#include "TrackerManager.h"

//-- constants -----
static const float k_min_time_delta_seconds = 1 / 120.f;
static const float k_max_time_delta_seconds = 1 / 30.f;

//-- private methods -----
static void init_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static IPoseFilter *pose_filter_factory(
    const CommonSensorState::eDeviceType deviceType,
    const std::string &position_filter_type, const std::string &orientation_filter_type,
    const PoseFilterConstants &constants);
static void update_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD, const MorpheusHMDSensorState *morpheusHMDState,
    const float delta_time,
    const HMDOpticalPoseEstimation *poseEstimation, const PoseFilterSpace *poseFilterSpace, IPoseFilter *poseFilter);
static void update_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, const VirtualHMDSensorState *virtualHMDState,
    const float delta_time,
    const HMDOpticalPoseEstimation *poseEstimation, const PoseFilterSpace *poseFilterSpace, IPoseFilter *poseFilter);
static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame);
static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view, const HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame);

//-- public implementation -----
ServerHMDView::ServerHMDView(const int device_id)
    : ServerDeviceView(device_id)
    , m_tracking_listener_count(0)
    , m_tracking_enabled(false)
    , m_roi_disable_count(0)
    , m_device(nullptr)
    , m_shape_tracking_models(nullptr)
    , m_tracker_pose_estimations(nullptr)
    , m_multicam_pose_estimation(nullptr)
    , m_pose_filter(nullptr)
    , m_pose_filter_space(nullptr)
    , m_lastPollSeqNumProcessed(-1)
    , m_last_filter_update_timestamp()
    , m_last_filter_update_timestamp_valid(false)
{
}

ServerHMDView::~ServerHMDView()
{
}

bool ServerHMDView::allocate_device_interface(const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
    case CommonSensorState::Morpheus:
        {
            m_device = new MorpheusHMD();
            m_pose_filter = nullptr; // no pose filter until the device is opened

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);
            assert(tracking_shape.shape_type == PSVRTrackingShape_PointCloud);

            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_tracker_pose_estimations[tracker_index].clear();

                m_shape_tracking_models[tracker_index] = new PointCloudTrackingModel();            
                m_shape_tracking_models[tracker_index]->init(&tracking_shape);
            }

            m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
            m_multicam_pose_estimation->clear();
        } break;
    case CommonSensorState::VirtualHMD:
        {
            m_device = new VirtualHMD();
            m_pose_filter = nullptr; // no pose filter until the device is opened

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);

            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_tracker_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_tracker_pose_estimations[tracker_index].clear();

                switch (tracking_shape.shape_type)
                {
                case PSVRTrackingShape_PointCloud:
                    m_shape_tracking_models[tracker_index] = new PointCloudTrackingModel();
                    break;
                case PSVRTrackingShape_Sphere:
                    m_shape_tracking_models[tracker_index] = new SphereTrackingModel();
                    break;
                case PSVRTrackingShape_LightBar:
                    assert(false && "Lightbar tracking shape not yet implemented");
                    break;
                }
                m_shape_tracking_models[tracker_index]->init(&tracking_shape);
            }

            m_multicam_pose_estimation = new HMDOpticalPoseEstimation();
            m_multicam_pose_estimation->clear();
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerHMDView::free_device_interface()
{
    if (m_multicam_pose_estimation != nullptr)
    {
        delete m_multicam_pose_estimation;
        m_multicam_pose_estimation = nullptr;
    }

    if (m_tracker_pose_estimations != nullptr)
    {
        delete[] m_tracker_pose_estimations;
        m_tracker_pose_estimations = nullptr;
    }

    if (m_pose_filter_space != nullptr)
    {
        delete m_pose_filter_space;
        m_pose_filter_space = nullptr;
    }

    if (m_pose_filter != nullptr)
    {
        delete m_pose_filter;
        m_pose_filter = nullptr;
    }

    if (m_device != nullptr)
    {
        delete m_device;
        m_device = nullptr;
    }
}

bool ServerHMDView::open(const class DeviceEnumerator *enumerator)
{
    // Attempt to open the controller
    bool bSuccess = ServerDeviceView::open(enumerator);

    // Setup the orientation filter based on the controller configuration
    if (bSuccess)
    {
        IDeviceInterface *device = getDevice();
        bool bAllocateTrackingColor = false;

        switch (device->getDeviceType())
        {
        case CommonSensorState::Morpheus:
        case CommonSensorState::VirtualHMD:
            {
                // Create a pose filter based on the HMD type
                resetPoseFilter();
                m_multicam_pose_estimation->clear();
                bAllocateTrackingColor= true;
            } break;
        default:
            break;
        }

        // If needed for this kind of HMD, assign a tracking color id
        if (bAllocateTrackingColor)
        {
            PSVRTrackingColorType tracking_color_id;
            assert(m_device != nullptr);

            // If this device already has a valid tracked color assigned, 
            // claim it from the pool (or another controller/hmd that had it previously)
            if (m_device->getTrackingColorID(tracking_color_id) && tracking_color_id != PSVRTrackingColorType_INVALID)
            {
                DeviceManager::getInstance()->m_tracker_manager->claimTrackingColorID(this, tracking_color_id);
            }
            else
            {
                // Allocate a color from the list of remaining available color ids
                PSVRTrackingColorType allocatedColorID= DeviceManager::getInstance()->m_tracker_manager->allocateTrackingColorID();

                // Attempt to assign the tracking color id to the controller
                if (!m_device->setTrackingColorID(allocatedColorID))
                {
                    // If the device can't be assigned a tracking color, release the color back to the pool
                    DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(allocatedColorID);
                }
            }
        }

        // Reset the poll sequence number high water mark
        m_lastPollSeqNumProcessed = -1;
    }

    return bSuccess;
}

void ServerHMDView::close()
{
    set_tracking_enabled_internal(false);

    PSVRTrackingColorType tracking_color_id= PSVRTrackingColorType_INVALID;
    if (m_device != nullptr && m_device->getTrackingColorID(tracking_color_id))
    {
        if (tracking_color_id != PSVRTrackingColorType_INVALID)
        {
            DeviceManager::getInstance()->m_tracker_manager->freeTrackingColorID(tracking_color_id);
        }
    }

    ServerDeviceView::close();
}

void ServerHMDView::resetPoseFilter()
{
    assert(m_device != nullptr);

    if (m_pose_filter != nullptr)
    {
        delete m_pose_filter;
        m_pose_filter = nullptr;
    }

    if (m_pose_filter_space != nullptr)
    {
        delete m_pose_filter_space;
        m_pose_filter_space = nullptr;
    }

    switch (m_device->getDeviceType())
    {
    case CommonSensorState::Morpheus:
        {
            const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();

            init_filters_for_morpheus_hmd(morpheusHMD, &m_pose_filter_space, &m_pose_filter);
        } break;
    case CommonSensorState::VirtualHMD:
        {
            const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();

            init_filters_for_virtual_hmd(virtualHMD, &m_pose_filter_space, &m_pose_filter);
        } break;
    default:
        break;
    }
}

void ServerHMDView::updateOpticalPoseEstimation(TrackerManager* tracker_manager)
{
    const std::chrono::time_point<std::chrono::high_resolution_clock> now= std::chrono::high_resolution_clock::now();

    // TODO: Probably need to first update IMU state to get velocity.
    // If velocity is too high, don't bother getting a new position.
    // Though it may be enough to just use the camera ROI as the limit.
    
    if (getIsTrackingEnabled())
    {
        int valid_projection_tracker_ids[TrackerManager::k_max_devices];
        int projections_found = 0;

        PSVRTrackingShape trackingShape;
        m_device->getTrackingShape(trackingShape);
        assert(trackingShape.shape_type != PSVRTrackingShape_INVALID);

        // Find the projection of the HMD from the perspective of each tracker.
        // In the case of sphere projections, go ahead and compute the tracker relative position as well.
        for (int tracker_id = 0; tracker_id < tracker_manager->getMaxDevices(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            HMDOpticalPoseEstimation &trackerPoseEstimateRef = m_tracker_pose_estimations[tracker_id];

            // Assume we're going to lose tracking this frame
            const bool bWasTracking= trackerPoseEstimateRef.bCurrentlyTracking;
            bool bCurrentlyTracking = false;

            if (tracker->getIsOpen())
            {
                // See how long it's been since we got a new video frame
                const std::chrono::time_point<std::chrono::high_resolution_clock> now= 
                    std::chrono::high_resolution_clock::now();
                const std::chrono::duration<float, std::milli> timeSinceNewDataMillis= 
                    now - tracker->getLastNewDataTimestamp();
                const float timeoutMilli= 
                    static_cast<float>(DeviceManager::getInstance()->m_tracker_manager->getConfig().optical_tracking_timeout);

                // Can't compute tracking on video data that's too old
                if (timeSinceNewDataMillis.count() < timeoutMilli)
                {
                    // Initially the newTrackerPoseEstimate is a copy of the existing pose
                    bool bIsVisibleThisUpdate= false;

                    // If a new video frame is available this tick, 
                    // attempt to update the tracking location
                    if (tracker->getHasUnpublishedState())
                    {
                        // Create a copy of the pose estimate state so that in event of a 
                        // failure part way through computing the projection we don't
                        // set partially valid state
                        PSVRTrackingProjection newTrackerProjection;

                        // Compute the projection of the shape on the tracker
                        if (tracker->computeProjectionForHMD(
                                this, 
                                &trackingShape,
                                &newTrackerProjection))
                        {
                            IShapeTrackingModel *shape_tracking_model= m_shape_tracking_models[tracker_id];

                            // Apply the projection to the tracking model 
                            // to compute a tracker relative pose
                            if (shape_tracking_model->applyShapeProjectionFromTracker(
                                    tracker.get(),
                                    newTrackerProjection) == true)
                            {
                                bIsVisibleThisUpdate= true;
                                
                                // Extract the tracker relative optical pose from the tracking model
                                trackerPoseEstimateRef.bOrientationValid=
                                    shape_tracking_model->getShapeOrientation(
                                        trackerPoseEstimateRef.orientation);
                                trackerPoseEstimateRef.bCurrentlyTracking=
                                    shape_tracking_model->getShapePosition(
                                        trackerPoseEstimateRef.position_cm);

                                if (trackerPoseEstimateRef.bCurrentlyTracking)
                                {
                                    shape_tracking_model->getShape(trackerPoseEstimateRef.shape);

									// If the tracking model is a point cloud, 
									// also get the correlation from 3d tracking model point to 2d camera projection.
									// This is used to get the projection area for a given point,
									// which in turn is used to compute measurement confidence
									if (trackingShape.shape_type == PSVRTrackingShape_PointCloud)
									{
										PointCloudTrackingModel *point_cloud_tracking_model=
											static_cast<PointCloudTrackingModel *>(shape_tracking_model);

										point_cloud_tracking_model->getPointCloudProjectionShapeCorrelation(newTrackerProjection);
									}
                                }
                                else
                                {
                                    memset(&trackerPoseEstimateRef.shape, 0, sizeof(PSVRTrackingShape));
                                }

                                // Actually apply the pose estimate state
                                trackerPoseEstimateRef.projection= newTrackerProjection;
                                trackerPoseEstimateRef.last_visible_timestamp = now;
                            }

							// Draw projection debugging now that shape tracking model has been applied
							tracker->drawPoseProjection(&newTrackerProjection);
                        }
                    }

                    // If the projection isn't too old (or updated this tick), 
                    // say we have a valid tracked location
                    if (bWasTracking || bIsVisibleThisUpdate)
                    {
                        const std::chrono::duration<float, std::milli> timeSinceLastVisibleMillis= 
                            now - trackerPoseEstimateRef.last_visible_timestamp;

                        if (timeSinceLastVisibleMillis.count() < timeoutMilli)
                        {
                            // If this tracker has a valid projection for the controller
                            // add it to the tracker id list
                            valid_projection_tracker_ids[projections_found] = tracker_id;
                            ++projections_found;

                            // Flag this pose estimate as invalid
                            bCurrentlyTracking = true;
                        }
                    }
                }
            }

            // Keep track of the last time the position estimate was updated
            trackerPoseEstimateRef.last_update_timestamp = now;
            trackerPoseEstimateRef.bValidTimestamps = true;
            trackerPoseEstimateRef.bCurrentlyTracking = bCurrentlyTracking;
        }

        // How we compute the final world pose estimate varies based on
        // * Number of trackers that currently have a valid projections of the controller
        // * The kind of projection shape (psmove sphere, ds4 lightbar, or psvr led cloud)
        //###HipsterSloth $TODO - Multiple trackers
        if (projections_found > 0)
        {
            const int tracker_id = valid_projection_tracker_ids[0];
            const ServerTrackerViewPtr tracker = tracker_manager->getTrackerViewPtr(tracker_id);
            const HMDOpticalPoseEstimation &tracker_pose_estimate = m_tracker_pose_estimations[tracker_id];

            if (tracker_pose_estimate.bOrientationValid)
            {
                m_multicam_pose_estimation->orientation= 
                        tracker->computeWorldOrientation(&tracker_pose_estimate.orientation);
                m_multicam_pose_estimation->bOrientationValid= true;
            }
            else
            {
                m_multicam_pose_estimation->orientation= *k_PSVR_quaternion_identity;
                m_multicam_pose_estimation->bOrientationValid= false;
            }

            if (tracker_pose_estimate.bCurrentlyTracking)
            {
                tracker->computeWorldShape(
                    &tracker_pose_estimate.shape, 
                    &m_multicam_pose_estimation->shape);
                m_multicam_pose_estimation->position_cm = 
                    tracker->computeWorldPosition(&tracker_pose_estimate.position_cm);
                m_multicam_pose_estimation->bCurrentlyTracking = true;
            }
            else
            {
                memset(&m_multicam_pose_estimation->shape, 0, sizeof(PSVRTrackingShape));
                m_multicam_pose_estimation->position_cm = *k_PSVR_float_vector3_zero;
                m_multicam_pose_estimation->bCurrentlyTracking = true;
            }

            // Copy over the screen projection area
            m_multicam_pose_estimation->projection = tracker_pose_estimate.projection;
        }
        // If no trackers can see the controller, maintain the last known position and time it was seen
        else
        {
            m_multicam_pose_estimation->bCurrentlyTracking= false;
        }

        // Update the position estimation timestamps
        if (m_multicam_pose_estimation->bCurrentlyTracking)
        {
            m_multicam_pose_estimation->last_visible_timestamp = now;
        }
        m_multicam_pose_estimation->last_update_timestamp = now;
        m_multicam_pose_estimation->bValidTimestamps = true;
    }
}

void ServerHMDView::updateStateAndPredict()
{
    if (!getHasUnpublishedState())
    {
        return;
    }

    // Look backward in time to find the first HMD update state with a poll sequence number 
    // newer than the last sequence number we've processed.
    int firstLookBackIndex = -1;
    int testLookBack = 0;
    const CommonHMDSensorState *state = getState(testLookBack);
    while (state != nullptr && state->PollSequenceNumber > m_lastPollSeqNumProcessed)
    {
        firstLookBackIndex = testLookBack;
        testLookBack++;
        state = getState(testLookBack);
    }
    assert(firstLookBackIndex >= 0);

    // Compute the time in seconds since the last update
    const std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
    float time_delta_seconds;
    if (m_last_filter_update_timestamp_valid)
    {
        const std::chrono::duration<float, std::milli> time_delta = now - m_last_filter_update_timestamp;
        const float time_delta_milli = time_delta.count();

        // convert delta to seconds clamp time delta between 120hz and 30hz
        time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);
    }
    else
    {
        time_delta_seconds = k_max_time_delta_seconds;
    }
    m_last_filter_update_timestamp = now;
    m_last_filter_update_timestamp_valid = true;

    // Evenly apply the list of hmd state updates over the time since last filter update
    float per_state_time_delta_seconds = time_delta_seconds / static_cast<float>(firstLookBackIndex + 1);

    // Process the polled hmd states forward in time
    // computing the new orientation along the way.
    for (int lookBackIndex = firstLookBackIndex; lookBackIndex >= 0; --lookBackIndex)
    {
        const CommonHMDSensorState *hmdState = getState(lookBackIndex);

        switch (hmdState->DeviceType)
        {
        case CommonHMDSensorState::Morpheus:
            {
                const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();
                const MorpheusHMDSensorState *morpheusHMDState = static_cast<const MorpheusHMDSensorState *>(hmdState);

                // Only update the position filter when tracking is enabled
                update_filters_for_morpheus_hmd(
                    morpheusHMD, morpheusHMDState,
                    per_state_time_delta_seconds,
                    m_multicam_pose_estimation,
                    m_pose_filter_space,
                    m_pose_filter);
            } break;
        case CommonHMDSensorState::VirtualHMD:
            {
                const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();
                const VirtualHMDSensorState *virtualHMDState = static_cast<const VirtualHMDSensorState *>(hmdState);

                // Only update the position filter when tracking is enabled
                update_filters_for_virtual_hmd(
                    virtualHMD, virtualHMDState,
                    per_state_time_delta_seconds,
                    m_multicam_pose_estimation,
                    m_pose_filter_space,
                    m_pose_filter);
            } break;
        default:
            assert(0 && "Unhandled HMD type");
        }

        // Consider this hmd state sequence num processed
        m_lastPollSeqNumProcessed = hmdState->PollSequenceNumber;
    }
}

PSVRPosef
ServerHMDView::getFilteredPose(float time) const
{
    PSVRPosef pose= *k_PSVR_pose_identity;


    if (m_pose_filter != nullptr)
    {
        const Eigen::Quaternionf orientation = m_pose_filter->getOrientation(time);
        const Eigen::Vector3f position_cm = m_pose_filter->getPositionCm(time);

        pose.Orientation.w = orientation.w();
        pose.Orientation.x = orientation.x();
        pose.Orientation.y = orientation.y();
        pose.Orientation.z = orientation.z();

        pose.Position.x = position_cm.x();
        pose.Position.y = position_cm.y();
        pose.Position.z = position_cm.z();
    }

    return pose;
}

PSVRPhysicsData
ServerHMDView::getFilteredPhysics() const
{
    PSVRPhysicsData physics;

    if (m_pose_filter != nullptr)
    {
        const Eigen::Vector3f first_derivative = m_pose_filter->getAngularVelocityRadPerSec();
        const Eigen::Vector3f second_derivative = m_pose_filter->getAngularAccelerationRadPerSecSqr();
        const Eigen::Vector3f velocity(m_pose_filter->getVelocityCmPerSec());
        const Eigen::Vector3f acceleration(m_pose_filter->getAccelerationCmPerSecSqr());

        physics.AngularVelocityRadPerSec.x = first_derivative.x();
        physics.AngularVelocityRadPerSec.y = first_derivative.y();
        physics.AngularVelocityRadPerSec.z = first_derivative.z();

        physics.AngularAccelerationRadPerSecSqr.x = second_derivative.x();
        physics.AngularAccelerationRadPerSecSqr.y = second_derivative.y();
        physics.AngularAccelerationRadPerSecSqr.z = second_derivative.z();

        physics.LinearVelocityCmPerSec.x = velocity.x();
        physics.LinearVelocityCmPerSec.y = velocity.y();
        physics.LinearVelocityCmPerSec.z = velocity.z();

        physics.LinearAccelerationCmPerSecSqr.x = acceleration.x();
        physics.LinearAccelerationCmPerSecSqr.y = acceleration.y();
        physics.LinearAccelerationCmPerSecSqr.z = acceleration.z();

        physics.TimeInSeconds= m_pose_filter->getTimeInSeconds();
    }

    return physics;
}

// Returns the full usb device path for the controller
std::string
ServerHMDView::getUSBDevicePath() const
{
    return m_device->getUSBDevicePath();
}

// Returns the "controller_" + serial number for the controller
std::string
ServerHMDView::getConfigIdentifier() const
{
    std::string	identifier = "";

    if (m_device != nullptr)
    {
        if (m_device->getDeviceType() == CommonSensorState::Morpheus)
        {
            identifier = "hmd_morpheus";
        }
        else if (m_device->getDeviceType() == CommonSensorState::VirtualHMD)
        {
            // THe "USB device path" for a Virtual HMD is actually just the Virtual HMD identifier, i.e. "VirtualHMD_0"
            identifier = "hmd_"+m_device->getUSBDevicePath();
        }
        else
        {
            identifier = "hmd_unknown";
        }
    }

    return identifier;
}

CommonSensorState::eDeviceType
ServerHMDView::getHMDDeviceType() const
{
    return m_device->getDeviceType();
}

// Fetch the controller state at the given sample index.
// A lookBack of 0 corresponds to the most recent data.
const struct CommonHMDSensorState * ServerHMDView::getState(
    int lookBack) const
{
    const struct CommonSensorState *device_state = m_device->getSensorState(lookBack);
    assert(device_state == nullptr ||
        ((int)device_state->DeviceType >= (int)CommonSensorState::HeadMountedDisplay &&
        device_state->DeviceType < CommonSensorState::SUPPORTED_HMD_TYPE_COUNT));

    return static_cast<const CommonHMDSensorState *>(device_state);
}

void ServerHMDView::startTracking()
{
    if (!m_tracking_enabled)
    {
        set_tracking_enabled_internal(true);
    }

    ++m_tracking_listener_count;
}

void ServerHMDView::stopTracking()
{
    assert(m_tracking_listener_count > 0);
    --m_tracking_listener_count;

    if (m_tracking_listener_count <= 0 && m_tracking_enabled)
    {
        set_tracking_enabled_internal(false);
    }
}

void ServerHMDView::set_tracking_enabled_internal(bool bEnabled)
{
    if (m_tracking_enabled != bEnabled)
    {
        switch (getHMDDeviceType())
        {
        case CommonHMDSensorState::Morpheus:
            castChecked<MorpheusHMD>()->setTrackingEnabled(bEnabled);
            break;
        case CommonHMDSensorState::VirtualHMD:
            castChecked<VirtualHMD>()->setTrackingEnabled(bEnabled);
            break;
        default:
            assert(0 && "unreachable");
        }

        m_tracking_enabled = bEnabled;
    }
}

// Get the tracking shape for the controller
bool ServerHMDView::getTrackingShape(PSVRTrackingShape &trackingShape) const
{
    m_device->getTrackingShape(trackingShape);

    return trackingShape.shape_type != PSVRTrackingShape_INVALID;
}

PSVRTrackingColorType ServerHMDView::getTrackingColorID() const
{
    PSVRTrackingColorType tracking_color_id = PSVRTrackingColorType_INVALID;

    if (m_device != nullptr)
    {
        m_device->getTrackingColorID(tracking_color_id);
    }

    return tracking_color_id;
}

bool ServerHMDView::setTrackingColorID(PSVRTrackingColorType colorID)
{
    bool bSuccess= true;

    if (colorID != getTrackingColorID())
    {
        if (m_device != nullptr)
        {
            bSuccess= m_device->setTrackingColorID(colorID);

            if (bSuccess && getIsTrackingEnabled())
            {
                set_tracking_enabled_internal(false);
                set_tracking_enabled_internal(true);
            }
        }
        else
        {
            bSuccess= false;
        }
    }

    return bSuccess;
}

float ServerHMDView::getROIPredictionTime() const
{
    return m_device->getPredictionTime();
}

void ServerHMDView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out HMD updates.
    // This will call generate_hmd_data_frame_for_stream for each listening connection.
    ServiceRequestHandler::get_instance()->publish_hmd_data_frame(
        this, &ServerHMDView::generate_hmd_data_frame_for_stream);
}

void ServerHMDView::generate_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const struct HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    HMDDataPacket *hmd_data_frame = &data_frame.device.hmd_data_packet;

    hmd_data_frame->hmd_id= hmd_view->getDeviceID();
    hmd_data_frame->output_sequence_num= hmd_view->m_sequence_number;
    hmd_data_frame->is_connected= hmd_view->getDevice()->getIsOpen();

    switch (hmd_view->getHMDDeviceType())
    {
    case CommonHMDSensorState::Morpheus:
        {
            generate_morpheus_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    case CommonHMDSensorState::VirtualHMD:
        {
            generate_virtual_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled HMD type");
    }

    data_frame.device_category= DeviceCategory_HMD;
}

static void
init_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
    PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter)
{
    const MorpheusHMDConfig *hmd_config = morpheusHMD->getConfig();

    // Setup the space the pose filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
    pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
    pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
    pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

    PSVRVector3f accel_var = hmd_config->get_calibrated_accelerometer_variance();
    PSVRVector3f gyro_var = hmd_config->get_calibrated_gyro_variance();
    PSVRVector3f gyro_drift = hmd_config->get_calibrated_gyro_drift();

    // Copy the pose filter constants from the controller config
    PoseFilterConstants constants;
    constants.clear();

    morpheusHMD->getTrackingShape(constants.shape);

    constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.orientation_constants.accelerometer_variance = Eigen::Vector3f(accel_var.x, accel_var.y, accel_var.z);
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
    constants.orientation_constants.gyro_drift = Eigen::Vector3f(gyro_drift.x, gyro_drift.y, gyro_drift.z);
    constants.orientation_constants.gyro_variance = Eigen::Vector3f(gyro_var.x, gyro_var.y, gyro_var.z);
    constants.orientation_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
    constants.orientation_constants.orientation_variance_curve.A = hmd_config->orientation_variance;
    constants.orientation_constants.orientation_variance_curve.B = 0.f;
    constants.orientation_constants.orientation_variance_curve.MaxValue = 1.f;
    constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
    constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

    constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.position_constants.accelerometer_variance = Eigen::Vector3f(accel_var.x, accel_var.y, accel_var.z);
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
    constants.position_constants.max_velocity = hmd_config->max_velocity;
    constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
    constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
    constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
    constants.position_constants.position_variance_curve.MaxValue = 1.f;

    *out_pose_filter_space = pose_filter_space;
    *out_pose_filter = pose_filter_factory(
        CommonSensorState::eDeviceType::Morpheus,
        hmd_config->position_filter_type,
        hmd_config->orientation_filter_type,
        constants);
}

static void init_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter)
{
    const VirtualHMDConfig *hmd_config = virtualHMD->getConfig();

    // Setup the space the pose filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
    pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());
    pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
    pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

    // Copy the pose filter constants from the controller config
    PoseFilterConstants constants;
    constants.clear();

    virtualHMD->getTrackingShape(constants.shape);

    constants.orientation_constants.gravity_calibration_direction = Eigen::Vector3f::Zero();
    constants.orientation_constants.accelerometer_variance = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.orientation_constants.magnetometer_calibration_direction = Eigen::Vector3f::Zero();
    constants.orientation_constants.gyro_drift = Eigen::Vector3f::Zero();
    constants.orientation_constants.gyro_variance = Eigen::Vector3f::Zero();
    constants.orientation_constants.mean_update_time_delta = 0.f;
    constants.orientation_constants.orientation_variance_curve.A = 0.f;
    constants.orientation_constants.orientation_variance_curve.B = 0.f;
    constants.orientation_constants.orientation_variance_curve.MaxValue = 0.f;
    constants.orientation_constants.magnetometer_variance = Eigen::Vector3f::Zero();
    constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

    constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.position_constants.accelerometer_variance = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_noise_radius = 0.f; // TODO
    constants.position_constants.max_velocity = hmd_config->max_velocity;
    constants.position_constants.mean_update_time_delta = hmd_config->mean_update_time_delta;
    constants.position_constants.position_variance_curve.A = hmd_config->position_variance_exp_fit_a;
    constants.position_constants.position_variance_curve.B = hmd_config->position_variance_exp_fit_b;
    constants.position_constants.position_variance_curve.MaxValue = 1.f;

    *out_pose_filter_space = pose_filter_space;
    *out_pose_filter = pose_filter_factory(
        CommonSensorState::eDeviceType::VirtualHMD,
        hmd_config->position_filter_type,
        hmd_config->orientation_filter_type,
        constants);
}

static IPoseFilter *
pose_filter_factory(
    const CommonSensorState::eDeviceType deviceType,
    const std::string &position_filter_type,
    const std::string &orientation_filter_type,
    const PoseFilterConstants &constants)
{
    IPoseFilter *filter = nullptr;

    if ((position_filter_type == "KalmanPose" || orientation_filter_type == "KalmanPose") && 
        constants.shape.shape_type == PSVRTrackingShape_PointCloud &&
        constants.shape.shape.pointcloud.point_count == 9) //###HipsterSloth $TODO support other point counts
    {
        switch (deviceType)
        {
        case CommonSensorState::Morpheus:
            {
                KalmanPoseFilterMorpheus *kalman_filter = new KalmanPoseFilterMorpheus();
                kalman_filter->init(constants);
                filter= kalman_filter;
            } break;
        case CommonSensorState::VirtualHMD:
            {
                KalmanPoseFilterPointCloud *kalman_filter = new KalmanPoseFilterPointCloud();
                kalman_filter->init(constants);
                filter= kalman_filter;
            } break;
        default:
            assert(0 && "unreachable");
        }
    }
    else
    {
        // Convert the position filter type string into an enum
        PositionFilterType position_filter_enum = PositionFilterTypeNone;
        if (position_filter_type == "PassThru")
        {
            position_filter_enum = PositionFilterTypePassThru;
        }
        else if (position_filter_type == "LowPassOptical")
        {
            position_filter_enum = PositionFilterTypeLowPassOptical;
        }
        else if (position_filter_type == "LowPassIMU")
        {
            position_filter_enum = PositionFilterTypeLowPassIMU;
        }
        else if (position_filter_type == "LowPassExponential")
        {
            position_filter_enum = PositionFilterTypeLowPassExponential;
        }
        else if (position_filter_type == "ComplimentaryOpticalIMU")
        {
            position_filter_enum = PositionFilterTypeComplimentaryOpticalIMU;
        }
        else
        {
            PSVR_LOG_INFO("pose_filter_factory()") <<
                "Unknown position filter type: " << position_filter_type << ". Using default.";

            // fallback to a default based on hmd type
            switch (deviceType)
            {
            case CommonSensorState::Morpheus:
            case CommonSensorState::VirtualHMD:
                position_filter_enum = PositionFilterTypeLowPassIMU;
                break;
            default:
                assert(0 && "unreachable");
            }
        }

        // Convert the orientation filter type string into an enum
        OrientationFilterType orientation_filter_enum = OrientationFilterTypeNone;
        if (orientation_filter_type == "")
        {
            orientation_filter_enum = OrientationFilterTypeNone;
        }
        else if (orientation_filter_type == "PassThru")
        {
            orientation_filter_enum = OrientationFilterTypePassThru;
        }
        else if (orientation_filter_type == "MadgwickARG")
        {
            orientation_filter_enum = OrientationFilterTypeMadgwickARG;
        }
        else if (orientation_filter_type == "ComplementaryOpticalARG")
        {
            orientation_filter_enum = OrientationFilterTypeComplementaryOpticalARG;
        }
        else if (orientation_filter_type == "KalmanOrientation")
        {
            orientation_filter_enum = OrientationFilterTypeKalman;
        }
        else
        {
            PSVR_LOG_INFO("pose_filter_factory()") <<
                "Unknown orientation filter type: " << orientation_filter_type << ". Using default.";

            // fallback to a default based on controller type
            switch (deviceType)
            {
            case CommonSensorState::Morpheus:
                orientation_filter_enum = OrientationFilterTypeComplementaryOpticalARG;
                break;
            case CommonSensorState::VirtualHMD:
                orientation_filter_enum = OrientationFilterTypePassThru;
                break;
            default:
                assert(0 && "unreachable");
            }
        }

        CompoundPoseFilter *compound_pose_filter = new CompoundPoseFilter();
        compound_pose_filter->init(deviceType, orientation_filter_enum, position_filter_enum, constants);
        filter = compound_pose_filter;
    }

    assert(filter != nullptr);

    return filter;
}

static void
update_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
    const MorpheusHMDSensorState *morpheusHMDState,
    const float delta_time,
    const HMDOpticalPoseEstimation *poseEstimation,
    const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *poseFilter)
{
    const MorpheusHMDConfig *config = morpheusHMD->getConfig();

    // Update the orientation filter
    if (poseFilter != nullptr)
    {
        PoseSensorPacket sensorPacket;

        sensorPacket.clear();

        if (poseEstimation->bOrientationValid)
        {
            sensorPacket.optical_orientation =
                Eigen::Quaternionf(
                    poseEstimation->orientation.w,
                    poseEstimation->orientation.x,
                    poseEstimation->orientation.y,
                    poseEstimation->orientation.z);
        }

        if (poseEstimation->bCurrentlyTracking)
        {
            sensorPacket.optical_tracking_shape_cm= poseEstimation->shape;
            sensorPacket.optical_position_cm =
                Eigen::Vector3f(
                    poseEstimation->position_cm.x,
                    poseEstimation->position_cm.y,
                    poseEstimation->position_cm.z);
			sensorPacket.optical_tracking_projection= poseEstimation->projection;
        }

        // Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
        for (int frame = 0; frame < 2; ++frame)
        {
            const MorpheusHMDSensorFrame &sensorFrame= morpheusHMDState->SensorFrames[frame];

            sensorPacket.imu_accelerometer_g_units =
                PSVR_vector3f_to_eigen_vector3(sensorFrame.CalibratedAccel);
            sensorPacket.imu_gyroscope_rad_per_sec =
                PSVR_vector3f_to_eigen_vector3(sensorFrame.CalibratedGyro);
            sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

            {
                PoseFilterPacket filterPacket;

				filterPacket.clear();

                // Create a filter input packet from the sensor data 
                // and the filter's previous orientation and position
                poseFilterSpace->createFilterPacket(
                    sensorPacket,
                    poseFilter,
                    filterPacket);

                poseFilter->update(delta_time / 2.f, filterPacket);
            }
        }
    }
}

static void
update_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD,
    const VirtualHMDSensorState *virtualHMDState,
    const float delta_time,
    const HMDOpticalPoseEstimation *poseEstimation,
    const PoseFilterSpace *poseFilterSpace,
    IPoseFilter *poseFilter)
{
    const VirtualHMDConfig *config = virtualHMD->getConfig();

    // Update the orientation filter
    if (poseFilter != nullptr)
    {
        PoseSensorPacket sensorPacket;

        sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

        if (poseEstimation->bOrientationValid)
        {
            sensorPacket.optical_orientation =
                Eigen::Quaternionf(
                    poseEstimation->orientation.w,
                    poseEstimation->orientation.x,
                    poseEstimation->orientation.y,
                    poseEstimation->orientation.z);
        }
        else
        {
            sensorPacket.optical_orientation = Eigen::Quaternionf::Identity();
        }

        if (poseEstimation->bCurrentlyTracking)
        {
            sensorPacket.optical_tracking_shape_cm= poseEstimation->shape;
            sensorPacket.optical_position_cm =
                PSVR_vector3f_to_eigen_vector3(poseEstimation->position_cm);
            sensorPacket.optical_tracking_projection= poseEstimation->projection;
        }
        else
        {
            sensorPacket.optical_position_cm = Eigen::Vector3f::Zero();
            sensorPacket.optical_tracking_projection.shape_type= PSVRShape_INVALID_PROJECTION;
        }

        sensorPacket.imu_accelerometer_g_units = Eigen::Vector3f::Zero();
        sensorPacket.imu_gyroscope_rad_per_sec = Eigen::Vector3f::Zero();
        sensorPacket.imu_magnetometer_unit = Eigen::Vector3f::Zero();

        {
            PoseFilterPacket filterPacket;

            // Create a filter input packet from the sensor data 
            // and the filter's previous orientation and position
            poseFilterSpace->createFilterPacket(sensorPacket, poseFilter, filterPacket);

            poseFilter->update(delta_time, filterPacket);
        }
    }
}

static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    const MorpheusHMD *morpheus_hmd = hmd_view->castCheckedConst<MorpheusHMD>();
    const MorpheusHMDConfig *morpheus_config = morpheus_hmd->getConfig();
    const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDSensorState *hmd_state = hmd_view->getState();
    const PSVRPosef hmd_pose = hmd_view->getFilteredPose();

    HMDDataPacket *hmd_data_frame = &data_frame.device.hmd_data_packet;

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonSensorState::Morpheus);
        const MorpheusHMDSensorState * morpheus_hmd_state = static_cast<const MorpheusHMDSensorState *>(hmd_state);

        PSVRMorpheus* morpheus_data_frame = &hmd_data_frame->hmd_state.morpheus_state;

        morpheus_data_frame->bIsCurrentlyTracking= hmd_view->getIsCurrentlyTracking();
        morpheus_data_frame->bIsTrackingEnabled= hmd_view->getIsTrackingEnabled();
        morpheus_data_frame->bIsOrientationValid= pose_filter->getIsStateValid();
        morpheus_data_frame->bIsPositionValid= pose_filter->getIsStateValid();

        morpheus_data_frame->Pose.Orientation= hmd_pose.Orientation;

        if (stream_info->include_position_data)
        {
            morpheus_data_frame->Pose.Position= hmd_pose.Position;
        }
        else
        {
            morpheus_data_frame->Pose.Position= *k_PSVR_float_vector3_zero;
        }

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_physics_data)
        {
            morpheus_data_frame->PhysicsData= hmd_view->getFilteredPhysics();
        }

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_raw_sensor_data)
        {
            PSVRMorpheusRawSensorData *raw_sensor_data = &morpheus_data_frame->RawSensorData;

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
            raw_sensor_data->Accelerometer= morpheus_hmd_state->SensorFrames[1].RawAccel;

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
            raw_sensor_data->Gyroscope= morpheus_hmd_state->SensorFrames[1].RawGyro;
        }

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_calibrated_sensor_data)
        {
            PSVRMorpheusCalibratedSensorData *calibrated_sensor_data= &morpheus_data_frame->CalibratedSensorData;

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
            calibrated_sensor_data->Accelerometer= morpheus_hmd_state->SensorFrames[1].CalibratedAccel;

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
            calibrated_sensor_data->Gyroscope= morpheus_hmd_state->SensorFrames[1].CalibratedGyro;
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSVRRawTrackerData *raw_tracker_data = &morpheus_data_frame->RawTrackerData;
            int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
                const HMDOpticalPoseEstimation *poseEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (poseEstimate != nullptr && poseEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
                        const ServerTrackerViewPtr tracker_view = 
                            DeviceManager::getInstance()->getTrackerViewPtr(trackerId);
                        const int projection_count= poseEstimate->projection.projection_count;

                        raw_tracker_data->TrackerID= trackerId;
                        raw_tracker_data->RelativePositionCm= poseEstimate->position_cm;
                        raw_tracker_data->RelativeOrientation= poseEstimate->orientation;
                        raw_tracker_data->TrackingProjection= poseEstimate->projection;

                        for (int projection_index = 0; projection_index < projection_count; ++projection_index)
                        {
                            // Project the 3d camera position back onto the tracker screen
                            raw_tracker_data->ScreenLocations[projection_index] =
                                tracker_view->projectTrackerRelativePosition(
                                    (PSVRVideoFrameSection)projection_index, 
                                    &poseEstimate->position_cm);
                        }
                    }
                }
            }

            raw_tracker_data->ValidTrackerBitmask= validTrackerBitmask;
        }
    }

    hmd_data_frame->hmd_type= PSVRHmd_Morpheus;
}

static void generate_virtual_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    const VirtualHMD *virtual_hmd = hmd_view->castCheckedConst<VirtualHMD>();
    const VirtualHMDConfig *virtual_hmd_config = virtual_hmd->getConfig();
    const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const CommonHMDSensorState *hmd_state = hmd_view->getState();
    const PSVRPosef hmd_pose = hmd_view->getFilteredPose();

    HMDDataPacket *hmd_data_frame = &data_frame.device.hmd_data_packet;

    if (hmd_state != nullptr)
    {
        assert(hmd_state->DeviceType == CommonSensorState::VirtualHMD);
        const VirtualHMDSensorState * virtual_hmd_state = static_cast<const VirtualHMDSensorState *>(hmd_state);

        PSVRVirtualHMD *virtual_hmd_data_frame = &hmd_data_frame->hmd_state.virtual_hmd_state;

        virtual_hmd_data_frame->bIsCurrentlyTracking= hmd_view->getIsCurrentlyTracking();
        virtual_hmd_data_frame->bIsTrackingEnabled= hmd_view->getIsTrackingEnabled();
        virtual_hmd_data_frame->bIsOrientationValid= pose_filter->getIsStateValid();
        virtual_hmd_data_frame->bIsPositionValid= pose_filter->getIsStateValid();

        virtual_hmd_data_frame->Pose.Orientation= hmd_pose.Orientation;

        if (stream_info->include_position_data)
        {
            virtual_hmd_data_frame->Pose.Position= hmd_pose.Position;
        }
        else
        {
            virtual_hmd_data_frame->Pose.Position= *k_PSVR_float_vector3_zero;
        }

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_physics_data)
        {
            virtual_hmd_data_frame->PhysicsData= hmd_view->getFilteredPhysics();
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSVRRawTrackerData *raw_tracker_data = &virtual_hmd_data_frame->RawTrackerData;
            int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
                const HMDOpticalPoseEstimation *poseEstimate = hmd_view->getTrackerPoseEstimate(trackerId);

                if (poseEstimate != nullptr && poseEstimate->bCurrentlyTracking)
                {
                    validTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
                        const ServerTrackerViewPtr tracker_view = 
                            DeviceManager::getInstance()->getTrackerViewPtr(trackerId);
                        const int projection_count= poseEstimate->projection.projection_count;

                        raw_tracker_data->TrackerID= trackerId;
                        raw_tracker_data->RelativePositionCm= poseEstimate->position_cm;
                        raw_tracker_data->RelativeOrientation= poseEstimate->orientation;
                        raw_tracker_data->TrackingProjection= poseEstimate->projection;

                        for (int projection_index = 0; projection_index < projection_count; ++projection_index)
                        {
                            // Project the 3d camera position back onto the tracker screen
                            raw_tracker_data->ScreenLocations[projection_index] =
                                tracker_view->projectTrackerRelativePosition(
                                    (PSVRVideoFrameSection)projection_index,
                                    &poseEstimate->position_cm);
                        }
                    }
                }
            }

            raw_tracker_data->ValidTrackerBitmask= validTrackerBitmask;
        }
    }

    hmd_data_frame->hmd_type= PSVRHmd_Virtual;
}
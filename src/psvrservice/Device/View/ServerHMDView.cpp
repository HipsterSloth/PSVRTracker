//-- includes -----
#include "DeviceManager.h"
#include "AtomicPrimitives.h"
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

//-- typedefs ----
using t_high_resolution_timepoint= std::chrono::time_point<std::chrono::high_resolution_clock>;
using t_high_resolution_duration= t_high_resolution_timepoint::duration;

//-- constants -----
static const float k_min_time_delta_seconds = 1 / 2500.f;
static const float k_max_time_delta_seconds = 1 / 30.f;

//-- private methods -----
static IPoseFilter *pose_filter_factory(
    const CommonSensorState::eDeviceType deviceType,
    const std::string &position_filter_type, const std::string &orientation_filter_type,
    const PoseFilterConstants &constants);

static void init_filters_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);
static void init_filters_for_virtual_hmd(
    const VirtualHMD *virtualHMD, PoseFilterSpace **out_pose_filter_space, IPoseFilter **out_pose_filter);

static void post_imu_filter_packets_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD, const MorpheusHMDSensorState *morpheusHMDState,
    const t_high_resolution_timepoint now, 
	const t_high_resolution_duration secondsSinceLastUpdate,
	t_hmd_pose_sensor_queue *pose_filter_queue);
static void post_optical_filter_packet_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const HMDOpticalPoseEstimation *poseEstimation,
	t_hmd_pose_sensor_queue *pose_filter_queue);
static void post_optical_filter_packet_for_virtual_hmd(
    const VirtualHMD *virtualHMD,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const HMDOpticalPoseEstimation *poseEstimation,
	t_hmd_pose_sensor_queue *pose_filter_queue);

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
	, m_tracking_enabled({false})
    , m_roi_disable_count(0)
    , m_device(nullptr)
    , m_shape_tracking_models(nullptr)
    , m_optical_pose_estimations(nullptr)
	, m_sharedFilteredPose(nullptr)
	, m_currentlyTrackingBitmask({0})
	, m_lastIMUSensorPacket(nullptr)
	, m_lastOpticalSensorPacket(nullptr)
    , m_pose_filter(nullptr)
    , m_pose_filter_space(nullptr)
    , m_lastPollSeqNumProcessed(-1)
    , m_lastFilterUpdateTimestamp()
    , m_bIsLastFilterUpdateTimestampValid(false)
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
			m_device->setHMDListener(this);

            m_pose_filter = nullptr; // no pose filter until the device is opened

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);
            assert(tracking_shape.shape_type == PSVRTrackingShape_PointCloud);

			m_lastIMUSensorPacket = new PoseSensorPacket;
			m_lastIMUSensorPacket->clear();

			m_sharedFilteredPose= new AtomicObject<ShapeTimestampedPose>;
            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_optical_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			m_lastOpticalSensorPacket = new PoseSensorPacket[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_optical_pose_estimations[tracker_index].clear();
				m_lastOpticalSensorPacket[tracker_index].clear();

                m_shape_tracking_models[tracker_index] = new PointCloudTrackingModel();            
                m_shape_tracking_models[tracker_index]->init(&tracking_shape);
            }

        } break;
    case CommonSensorState::VirtualHMD:
        {
            m_device = new VirtualHMD();
			m_device->setHMDListener(this);

            m_pose_filter = nullptr; // no pose filter until the device is opened

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);

			m_lastIMUSensorPacket= nullptr;

			m_sharedFilteredPose= new AtomicObject<ShapeTimestampedPose>;
            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_optical_pose_estimations = new HMDOpticalPoseEstimation[TrackerManager::k_max_devices];
			m_lastOpticalSensorPacket = new PoseSensorPacket[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_optical_pose_estimations[tracker_index].clear();
				m_lastOpticalSensorPacket[tracker_index].clear();

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
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerHMDView::free_device_interface()
{
	if (m_sharedFilteredPose)
	{
		delete m_sharedFilteredPose;
		m_sharedFilteredPose= nullptr;
	}

    if (m_optical_pose_estimations != nullptr)
    {
        delete[] m_optical_pose_estimations;
        m_optical_pose_estimations = nullptr;
    }

	if (m_lastIMUSensorPacket != nullptr)
	{
		delete m_lastIMUSensorPacket;
		m_lastIMUSensorPacket= nullptr;
	}

	if (m_lastOpticalSensorPacket != nullptr)
	{
		delete[] m_lastOpticalSensorPacket;
		m_lastOpticalSensorPacket= nullptr;
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

	m_bIsLastFilterUpdateTimestampValid= false;
	m_bIsLastSensorDataTimestampValid= false;
}

void ServerHMDView::notifyTrackerDataReceived(ServerTrackerView* tracker)
{
    const t_high_resolution_timepoint now= std::chrono::high_resolution_clock::now();

	int tracker_id= tracker->getDeviceID();
	HMDOpticalPoseEstimation &tracker_pose_estimate_ref = m_optical_pose_estimations[tracker_id];
    IShapeTrackingModel *shape_tracking_model= m_shape_tracking_models[tracker_id];

	// Compute an optical pose estimate from latest projection
    if (getIsTrackingEnabled())
    {
        PSVRTrackingShape trackingShape;
        m_device->getTrackingShape(trackingShape);
        assert(trackingShape.shape_type != PSVRTrackingShape_INVALID);

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
			// Get the last filtered pose from the main thread
			ShapeTimestampedPose filteredPose;
			m_sharedFilteredPose->fetchValue(filteredPose);

            // Apply the projection to the tracking model 
            // to compute a tracker relative pose
            if (shape_tracking_model->applyShapeProjectionFromTracker(
					now,
                    tracker,
					&filteredPose,
                    newTrackerProjection) == true)
            {
				// Extract the optical position from the tracking model
				tracker_pose_estimate_ref.bCurrentlyTracking=
                    shape_tracking_model->getShapePosition(
                        tracker_pose_estimate_ref.tracker_relative_position_cm);
				if (tracker_pose_estimate_ref.bCurrentlyTracking)
				{
					tracker_pose_estimate_ref.world_relative_position_cm = 
						tracker->computeWorldPosition(&tracker_pose_estimate_ref.tracker_relative_position_cm);
				}
				else
				{
					tracker_pose_estimate_ref.tracker_relative_position_cm = *k_PSVR_float_vector3_zero;
					tracker_pose_estimate_ref.world_relative_position_cm = *k_PSVR_float_vector3_zero;
				}

                // Extract the optical orientation from the tracking model
                tracker_pose_estimate_ref.bOrientationValid=
                    shape_tracking_model->getShapeOrientation(
                        tracker_pose_estimate_ref.tracker_relative_orientation);
				if (tracker_pose_estimate_ref.bOrientationValid)
				{
					tracker_pose_estimate_ref.world_relative_orientation= 
							tracker->computeWorldOrientation(&tracker_pose_estimate_ref.tracker_relative_orientation);
				}
				else
				{
					tracker_pose_estimate_ref.tracker_relative_orientation= *k_PSVR_quaternion_identity;
					tracker_pose_estimate_ref.world_relative_orientation= *k_PSVR_quaternion_identity;
				}

				// Extract the tracking shape info from the optical projection
                if (tracker_pose_estimate_ref.bCurrentlyTracking)
                {
					shape_tracking_model->getShape(tracker_pose_estimate_ref.tracker_relative_shape);
					tracker->computeWorldShape(
						&tracker_pose_estimate_ref.tracker_relative_shape, 
						&tracker_pose_estimate_ref.world_relative_shape);

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
					memset(&tracker_pose_estimate_ref.tracker_relative_shape, 0, sizeof(PSVRTrackingShape));
					memset(&tracker_pose_estimate_ref.world_relative_shape, 0, sizeof(PSVRTrackingShape));
                }

                // Actually apply the pose estimate state
                tracker_pose_estimate_ref.projection= newTrackerProjection;
                tracker_pose_estimate_ref.last_visible_timestamp = now;
            }

			// Draw projection debugging now that shape tracking model has been applied
			tracker->drawPoseProjection(&newTrackerProjection);
        }

        // Keep track of the last time the position estimate was updated
        tracker_pose_estimate_ref.last_update_timestamp = now;
        tracker_pose_estimate_ref.bValidTimestamps = true;
    }
	else
	{
		tracker_pose_estimate_ref.bCurrentlyTracking= false;
	}

	// Update the filter if we have a valid optically tracked pose
	if (tracker_pose_estimate_ref.bCurrentlyTracking)
	{
		switch (getHMDDeviceType())
		{
		case CommonSensorState::Morpheus:
			{
				const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();

				// Only update the position filter when tracking is enabled
				post_optical_filter_packet_for_morpheus_hmd(
					morpheusHMD,
					tracker_id,
					now,
					&tracker_pose_estimate_ref,
					&m_PoseSensorOpticalPacketQueue);
			} break;
		case CommonSensorState::VirtualHMD:
			{
				const VirtualHMD *virtualHMD = this->castCheckedConst<VirtualHMD>();

				// Only update the position filter when tracking is enabled
				post_optical_filter_packet_for_virtual_hmd(
					virtualHMD,
					tracker_id,
					now,
					&tracker_pose_estimate_ref,
					&m_PoseSensorOpticalPacketQueue);
			} break;
		default:
			assert(0 && "Unhandled HMD type");
		}

		// Set the flag corresponding to this tracker index to 1
		unsigned long tracker_bitmask= (1 << tracker_id);
		m_currentlyTrackingBitmask|= tracker_bitmask;
	}
	else
	{
		// Set the flag corresponding to this tracker index to 1
		unsigned long tracker_bitmask= ~(1 << tracker_id);
		m_currentlyTrackingBitmask&= tracker_bitmask;
	}
}

void 
ServerHMDView::notifySensorDataReceived(const CommonSensorState *sensor_state)
{
    // Compute the time in seconds since the last update
    const t_high_resolution_timepoint now = std::chrono::high_resolution_clock::now();
	t_high_resolution_duration durationSinceLastUpdate= t_high_resolution_duration::zero();

	if (m_bIsLastSensorDataTimestampValid)
	{
		durationSinceLastUpdate = now - m_lastSensorDataTimestamp;
	}
	m_lastSensorDataTimestamp= now;
	m_bIsLastSensorDataTimestampValid= true;

	// Apply device specific filtering
    switch (sensor_state->DeviceType)
    {
    case CommonSensorState::Morpheus:
        {
            const MorpheusHMD *morpheusHMD = this->castCheckedConst<MorpheusHMD>();
            const MorpheusHMDSensorState *morpheusHMDState = static_cast<const MorpheusHMDSensorState *>(sensor_state);

            // Only update the position filter when tracking is enabled
            post_imu_filter_packets_for_morpheus_hmd(
                morpheusHMD, morpheusHMDState,
                now, durationSinceLastUpdate,
				&m_PoseSensorIMUPacketQueue);
        } break;
    default:
        assert(0 && "Unhandled HMD type");
    }

    // Consider this HMD state sequence num processed
    m_lastPollSeqNumProcessed = sensor_state->PollSequenceNumber;
}

// Update Pose Filter using update packets from the tracker and IMU threads
void ServerHMDView::updatePoseFilter()
{
	std::vector<PoseSensorPacket> timeSortedPackets;

	// Drain the packet queues filled by the threads
	PoseSensorPacket packet;
	while (m_PoseSensorIMUPacketQueue.try_dequeue(packet))
	{
		timeSortedPackets.push_back(packet);
	}
	while (m_PoseSensorOpticalPacketQueue.try_dequeue(packet))
	{
		timeSortedPackets.push_back(packet);
	}

	// Sort the packets in order of ascending time
	if (timeSortedPackets.size() > 1)
	{
		std::sort(
			timeSortedPackets.begin(), timeSortedPackets.end(), 
			[](const PoseSensorPacket & a, const PoseSensorPacket & b) -> bool
			{
				return a.timestamp < b.timestamp; 
			});

		t_high_resolution_duration duration= 
			timeSortedPackets[timeSortedPackets.size()-1].timestamp - timeSortedPackets[0].timestamp;
		std::chrono::duration<float, std::milli> milli_duration= duration;

		const size_t k_max_process_count= 100;
		if (timeSortedPackets.size() > k_max_process_count)
		{
			const size_t excess= timeSortedPackets.size() - k_max_process_count;

			//PSVR_LOG_WARNING("updatePoseFilter()") << "Incoming packet count: " << timeSortedPackets.size() << " (" << milli_duration.count() << "ms)" << ", trimming: " << excess;
			timeSortedPackets.erase(timeSortedPackets.begin(), timeSortedPackets.begin()+excess);
		}
		else
		{
			//PSVR_LOG_INFO("updatePoseFilter()") << "Incoming packet count: " << timeSortedPackets.size() << " (" << milli_duration.count() << "ms)";
		}
	}

	const PoseSensorPacket *lastIMUPacket= nullptr;
	const PoseSensorPacket *lastOpticalPacket[TrackerManager::k_max_devices];
	memset(lastOpticalPacket, 0, sizeof(lastOpticalPacket));

	// Process the sensor packets from oldest to newest
	for (const PoseSensorPacket &sensorPacket : timeSortedPackets)
    {
		// Compute the time since the last packet
		float time_delta_seconds;
		if (m_bIsLastFilterUpdateTimestampValid)
		{
			const std::chrono::duration<float, std::milli> time_delta = sensorPacket.timestamp - m_lastFilterUpdateTimestamp;
			const float time_delta_milli = time_delta.count();

			// convert delta to seconds clamp time delta between 2500hz and 30hz
			time_delta_seconds = clampf(time_delta_milli / 1000.f, k_min_time_delta_seconds, k_max_time_delta_seconds);
		}
		else
		{
			time_delta_seconds = k_max_time_delta_seconds;
		}
		m_lastFilterUpdateTimestamp = sensorPacket.timestamp;
		m_bIsLastFilterUpdateTimestampValid = true;

		// Store a pointer to the last imu sensor packet we saw
		if (sensorPacket.has_imu_measurements())
		{
			lastIMUPacket= &sensorPacket;
		}

		// Store a pointer to the last optical sensor packet we saw
		if (sensorPacket.has_optical_measurement())
		{
			lastOpticalPacket[sensorPacket.tracker_id]=  &sensorPacket;
		}

		{
			PoseFilterPacket filter_packet;
			filter_packet.clear();

			// Create a filter input packet from the sensor data 
			// and the filter's previous orientation and position
			m_pose_filter_space->createFilterPacket(
				sensorPacket,
				m_pose_filter,
				filter_packet);

			// Process the filter packet
			m_pose_filter->update(time_delta_seconds, filter_packet);
		}

		// Flag the state as unpublished, which will trigger an update to the client
		markStateAsUnpublished();
	}

	// Publish the filtered state to the shared filtered pose.
	// This lets the optical pose processing threads use the the most recent filter out
	// as a guess for the next optical tracking estimate.
	if (m_bHasUnpublishedState)
	{
		if (m_pose_filter->getIsOrientationStateValid())
		{
			ShapeTimestampedPose filtered_pose;
			filtered_pose.timestamp= std::chrono::high_resolution_clock::now();
			filtered_pose.pose_cm= getFilteredPose();
			filtered_pose.bIsValid= true;

			m_sharedFilteredPose->storeValue(filtered_pose);
		}
	}

	// Copy off the last IMU packet
	if (lastIMUPacket != nullptr)
	{
		*m_lastIMUSensorPacket= *lastIMUPacket;
	}

	// Copy off the last optical packet for each tracker
	for (int tracker_id = 0; tracker_id < TrackerManager::k_max_devices; ++tracker_id)
	{
		if (lastOpticalPacket[tracker_id] != nullptr)
		{
			m_lastOpticalSensorPacket[tracker_id]= *lastOpticalPacket[tracker_id];
		}
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

// Returns the "hmd_" + serial number for the controller
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

const PoseSensorPacket *
ServerHMDView::getLastIMUSensorPacket() const
{
	return m_lastIMUSensorPacket;
}

const PoseSensorPacket *
ServerHMDView::getLastOpticalSensorPacket(int tracker_id) const
{
	return (m_lastOpticalSensorPacket != nullptr) ? &m_lastOpticalSensorPacket[tracker_id] : nullptr;
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
        case CommonSensorState::Morpheus:
            castChecked<MorpheusHMD>()->setTrackingEnabled(bEnabled);
            break;
        case CommonSensorState::VirtualHMD:
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
    case CommonSensorState::Morpheus:
        {
            generate_morpheus_hmd_data_frame_for_stream(hmd_view, stream_info, data_frame);
        } break;
    case CommonSensorState::VirtualHMD:
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
post_imu_filter_packets_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
    const MorpheusHMDSensorState *morpheusHMDState,
    const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_hmd_pose_sensor_queue *pose_filter_queue)
{
    const MorpheusHMDConfig *config = morpheusHMD->getConfig();

    PoseSensorPacket sensor_packet;

    sensor_packet.clear();

	const int start_frame_index= (duration_since_last_update == t_high_resolution_duration::zero()) ? 1 : 0;
	const t_high_resolution_timepoint prev_timestamp= now - (duration_since_last_update / 2);
	t_high_resolution_timepoint timestamps[2] = {prev_timestamp, now};

    // Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
    for (int frame = start_frame_index; frame < 2; ++frame)
    {
        const MorpheusHMDSensorFrame &sensorFrame= morpheusHMDState->SensorFrames[frame];

		sensor_packet.timestamp= timestamps[frame];

		sensor_packet.raw_imu_accelerometer= sensorFrame.RawAccel;
        sensor_packet.imu_accelerometer_g_units =
            PSVR_vector3f_to_eigen_vector3(sensorFrame.CalibratedAccel);
		sensor_packet.has_accelerometer_measurement= true;

		sensor_packet.raw_imu_gyroscope= sensorFrame.RawGyro;
		sensor_packet.imu_gyroscope_rad_per_sec =
            PSVR_vector3f_to_eigen_vector3(sensorFrame.CalibratedGyro);
		sensor_packet.has_gyroscope_measurement= true;

		sensor_packet.raw_imu_magnetometer = {0, 0, 0};
        sensor_packet.imu_magnetometer_unit = Eigen::Vector3f::Zero();
		sensor_packet.has_magnetometer_measurement= false;

        pose_filter_queue->enqueue(sensor_packet);
    }
}

static void
post_optical_filter_packet_for_morpheus_hmd(
    const MorpheusHMD *morpheusHMD,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const HMDOpticalPoseEstimation *pose_estimation,
    t_hmd_pose_sensor_queue *pose_sensor_queue)
{
    const MorpheusHMDConfig *config = morpheusHMD->getConfig();

    PoseSensorPacket sensor_packet;

    sensor_packet.clear();
	sensor_packet.timestamp= now;
	sensor_packet.tracker_id= tracker_id;

    if (pose_estimation->bOrientationValid)
    {
		sensor_packet.tracker_relative_orientation= 
			pose_estimation->tracker_relative_orientation;
        sensor_packet.optical_orientation =
			PSVR_quatf_to_eigen_quaternionf(pose_estimation->world_relative_orientation);
    }

    if (pose_estimation->bCurrentlyTracking)
    {
		sensor_packet.tracker_relative_position_cm= 
			pose_estimation->tracker_relative_position_cm;
        sensor_packet.optical_tracking_shape_cm= pose_estimation->world_relative_shape;
		sensor_packet.optical_position_cm =
			PSVR_vector3f_to_eigen_vector3(pose_estimation->world_relative_position_cm);
		sensor_packet.optical_tracking_projection= pose_estimation->projection;
    }

	pose_sensor_queue->enqueue(sensor_packet);
}

static void
post_optical_filter_packet_for_virtual_hmd(
    const VirtualHMD *virtualHMD,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const HMDOpticalPoseEstimation *pose_estimation,
	t_hmd_pose_sensor_queue *pose_sensor_queue)
{
    const VirtualHMDConfig *config = virtualHMD->getConfig();

    PoseSensorPacket sensor_packet;

	sensor_packet.clear();
	sensor_packet.timestamp= now;
	sensor_packet.tracker_id= tracker_id;

    if (pose_estimation->bOrientationValid)
    {
		sensor_packet.tracker_relative_orientation= 
			pose_estimation->tracker_relative_orientation;
        sensor_packet.optical_orientation =
			PSVR_quatf_to_eigen_quaternionf(pose_estimation->world_relative_orientation);
    }

    if (pose_estimation->bCurrentlyTracking)
    {
		sensor_packet.tracker_relative_position_cm= 
			pose_estimation->tracker_relative_position_cm;
        sensor_packet.optical_tracking_shape_cm= pose_estimation->world_relative_shape;
        sensor_packet.optical_position_cm =
            PSVR_vector3f_to_eigen_vector3(pose_estimation->world_relative_position_cm);
        sensor_packet.optical_tracking_projection= pose_estimation->projection;
    }

    pose_sensor_queue->enqueue(sensor_packet);
}

static void generate_morpheus_hmd_data_frame_for_stream(
    const ServerHMDView *hmd_view,
    const HMDStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    const MorpheusHMD *morpheus_hmd = hmd_view->castCheckedConst<MorpheusHMD>();
    const MorpheusHMDConfig *morpheus_config = morpheus_hmd->getConfig();
    const IPoseFilter *pose_filter = hmd_view->getPoseFilter();
    const PoseSensorPacket *imu_sensor_packet = hmd_view->getLastIMUSensorPacket();
    const PSVRPosef hmd_pose = hmd_view->getFilteredPose();

    HMDDataPacket *hmd_data_frame = &data_frame.device.hmd_data_packet;

    if (imu_sensor_packet != nullptr)
    {
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

            raw_sensor_data->Accelerometer= imu_sensor_packet->raw_imu_accelerometer;
            raw_sensor_data->Gyroscope= imu_sensor_packet->raw_imu_gyroscope;
        }

        // If requested, get the raw sensor data for the hmd
        if (stream_info->include_calibrated_sensor_data)
        {
            PSVRMorpheusCalibratedSensorData *calibrated_sensor_data= &morpheus_data_frame->CalibratedSensorData;

            calibrated_sensor_data->Accelerometer= eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_accelerometer_g_units);
            calibrated_sensor_data->Gyroscope= eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_gyroscope_rad_per_sec);
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSVRRawTrackerData *raw_tracker_data = &morpheus_data_frame->RawTrackerData;
            int selectedTrackerId= stream_info->selected_tracker_index;
            unsigned int validTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
				const PoseSensorPacket *optical_sensor_packet = hmd_view->getLastOpticalSensorPacket(trackerId);

                if (optical_sensor_packet != nullptr && hmd_view->getIsTrackedByTracker(trackerId))
                {
                    validTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
                        const ServerTrackerViewPtr tracker_view = 
                            DeviceManager::getInstance()->getTrackerViewPtr(trackerId);
                        const int projection_count= optical_sensor_packet->optical_tracking_projection.projection_count;

                        raw_tracker_data->TrackerID= trackerId;
                        raw_tracker_data->RelativePositionCm= optical_sensor_packet->tracker_relative_position_cm;
                        raw_tracker_data->RelativeOrientation= optical_sensor_packet->tracker_relative_orientation;
                        raw_tracker_data->TrackingProjection= optical_sensor_packet->optical_tracking_projection;
						raw_tracker_data->WorldRelativeShape= optical_sensor_packet->optical_tracking_shape_cm;

                        for (int projection_index = 0; projection_index < projection_count; ++projection_index)
                        {
                            // Project the 3d camera position back onto the tracker screen
                            raw_tracker_data->ScreenLocations[projection_index] =
                                tracker_view->projectTrackerRelativePosition(
                                    (PSVRVideoFrameSection)projection_index, 
                                    &optical_sensor_packet->tracker_relative_position_cm);
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
    const PSVRPosef hmd_pose = hmd_view->getFilteredPose();

    HMDDataPacket *hmd_data_frame = &data_frame.device.hmd_data_packet;
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
			const PoseSensorPacket *optical_sensor_packet = hmd_view->getLastOpticalSensorPacket(trackerId);

            if (optical_sensor_packet != nullptr && hmd_view->getIsTrackedByTracker(trackerId))
            {
                validTrackerBitmask&= (1 << trackerId);

                if (trackerId == selectedTrackerId)
                {
                    const ServerTrackerViewPtr tracker_view = 
                        DeviceManager::getInstance()->getTrackerViewPtr(trackerId);
					const int projection_count= optical_sensor_packet->optical_tracking_projection.projection_count;

                    raw_tracker_data->TrackerID= trackerId;
                    raw_tracker_data->RelativePositionCm= optical_sensor_packet->tracker_relative_position_cm;
                    raw_tracker_data->RelativeOrientation= optical_sensor_packet->tracker_relative_orientation;
                    raw_tracker_data->TrackingProjection= optical_sensor_packet->optical_tracking_projection;
					raw_tracker_data->WorldRelativeShape= optical_sensor_packet->optical_tracking_shape_cm;

                    for (int projection_index = 0; projection_index < projection_count; ++projection_index)
                    {
                        // Project the 3d camera position back onto the tracker screen
                        raw_tracker_data->ScreenLocations[projection_index] =
                            tracker_view->projectTrackerRelativePosition(
                                (PSVRVideoFrameSection)projection_index,
                                &optical_sensor_packet->tracker_relative_position_cm);
                    }
                }
            }
        }

        raw_tracker_data->ValidTrackerBitmask= validTrackerBitmask;
    }

    hmd_data_frame->hmd_type= PSVRHmd_Virtual;
}
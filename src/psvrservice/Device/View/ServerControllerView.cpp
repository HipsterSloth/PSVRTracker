//-- includes -----
#include "ServerControllerView.h"

#include "AtomicPrimitives.h"
//#include "BluetoothRequests.h"
#include "ControllerManager.h"
#include "DeviceManager.h"
#include "DualShock4Controller.h"
#include "Logger.h"
#include "CompoundPoseFilter.h"
#include "MathTypeConversion.h"
#include "KalmanPoseFilter.h"
#include "PoseFilterInterface.h"
#include "PSMoveController.h"
#include "SphereTrackingModel.h"
#include "ServiceRequestHandler.h"
#include "ServerTrackerView.h"
#include "Utility.h"
#include "TrackerManager.h"

//-- typedefs ----
using t_high_resolution_timepoint= std::chrono::time_point<std::chrono::high_resolution_clock>;
using t_high_resolution_duration= t_high_resolution_timepoint::duration;

//-- constants -----
static const float k_min_time_delta_seconds = 1 / 2500.f;
static const float k_max_time_delta_seconds = 1 / 30.f;

//-- macros -----
#define SET_BUTTON_BIT(bitmask, bit_index, button_state) \
    bitmask|= (button_state == CommonControllerState::Button_DOWN || button_state == CommonControllerState::Button_PRESSED) ? (0x1 << (bit_index)) : 0x0;

//-- private methods -----
static IPoseFilter *pose_filter_factory(
    const CommonSensorState::eDeviceType deviceType,
    const std::string &position_filter_type,
    const std::string &orientation_filter_type,
    const PoseFilterConstants &constants);

static void init_filters_for_psmove(
    const PSMoveController *psmoveController, 
    PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter);
static void init_filters_for_dualshock4(
    const DualShock4Controller *dualshock4Controller,
    PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter);

static void post_imu_filter_packets_for_psmove(
	const PSMoveController *psmove, 
	const PSMoveControllerInputState *psmoveState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_controller_imu_sensor_queue *pose_filter_queue);
static void post_optical_filter_packet_for_psmove(
    const PSMoveController *psmove,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const ControllerOpticalPoseEstimation *pose_estimation,
    t_controller_optical_sensor_queue *pose_sensor_queue);

static void post_imu_filter_packets_for_dualshock4(
	const DualShock4Controller *ds4, 
	const DualShock4ControllerInputState *ds4State,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_controller_imu_sensor_queue *pose_filter_queue);
static void post_optical_filter_packet_for_dualshock4(
	const DualShock4Controller *ds4,
	const int tracker_id,
	const t_high_resolution_timepoint now,
	const ControllerOpticalPoseEstimation *pose_estimation,
	t_controller_optical_sensor_queue *pose_sensor_queue);

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, 
	DeviceOutputDataFrame &data_frame);
static void generate_psdualshock4_data_frame_for_stream(
    const ServerControllerView *controller_view, const ControllerStreamInfo *stream_info, 
	DeviceOutputDataFrame &data_frame);

//-- public implementation -----
ServerControllerView::ServerControllerView(const int device_id)
    : ServerDeviceView(device_id)
    , m_tracking_listener_count(0)
    , m_tracking_enabled({false})
    , m_roi_disable_count(0)
	, m_device(nullptr)
    , m_LED_override_active(false)
    , m_shape_tracking_models(nullptr)
    , m_optical_pose_estimations(nullptr)
	, m_PoseSensorOpticalPacketQueues(nullptr)
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
    m_tracking_color = std::make_tuple(0x00, 0x00, 0x00);
    m_LED_override_color = std::make_tuple(0x00, 0x00, 0x00);
}

ServerControllerView::~ServerControllerView()
{
}

bool ServerControllerView::allocate_device_interface(
    const class DeviceEnumerator *enumerator)
{
    switch (enumerator->get_device_type())
    {
	case CommonSensorState::PSMove:
        {
            m_device = new PSMoveController();
			m_device->setControllerListener(this);

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);
            assert(tracking_shape.shape_type == PSVRTrackingShape_Sphere);

			m_lastIMUSensorPacket = new PoseSensorPacket;
			m_lastIMUSensorPacket->clear();

			m_sharedFilteredPose= new AtomicObject<ShapeTimestampedPose>;
            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_optical_pose_estimations = new ControllerOpticalPoseEstimation[TrackerManager::k_max_devices];
			m_PoseSensorOpticalPacketQueues = new t_controller_optical_sensor_queue[TrackerManager::k_max_devices];
			m_lastOpticalSensorPacket = new PoseSensorPacket[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_optical_pose_estimations[tracker_index].clear();
				m_lastOpticalSensorPacket[tracker_index].clear();

                m_shape_tracking_models[tracker_index] = new SphereTrackingModel();
                m_shape_tracking_models[tracker_index]->init(&tracking_shape);
            }
        } break;
    case CommonSensorState::DualShock4:
        {
            m_device = new DualShock4Controller();
			m_device->setControllerListener(this);

            PSVRTrackingShape tracking_shape;
            m_device->getTrackingShape(tracking_shape);
            assert(tracking_shape.shape_type == PSVRTrackingShape_LightBar);

			m_lastIMUSensorPacket = new PoseSensorPacket;
			m_lastIMUSensorPacket->clear();

			m_sharedFilteredPose= new AtomicObject<ShapeTimestampedPose>;
            m_shape_tracking_models = new IShapeTrackingModel *[TrackerManager::k_max_devices]; 
            m_optical_pose_estimations = new ControllerOpticalPoseEstimation[TrackerManager::k_max_devices];
			m_PoseSensorOpticalPacketQueues = new t_controller_optical_sensor_queue[TrackerManager::k_max_devices];
			m_lastOpticalSensorPacket = new PoseSensorPacket[TrackerManager::k_max_devices];
            for (int tracker_index = 0; tracker_index < TrackerManager::k_max_devices; ++tracker_index)
            {
                m_optical_pose_estimations[tracker_index].clear();
				m_lastOpticalSensorPacket[tracker_index].clear();

				//TODO: Need a lightbar tracking model
                m_shape_tracking_models[tracker_index] = NULL; //new LightBarTrackingModel();
                m_shape_tracking_models[tracker_index]->init(&tracking_shape);
            }
        } break;
    default:
        break;
    }

    return m_device != nullptr;
}

void ServerControllerView::free_device_interface()
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

	if (m_PoseSensorOpticalPacketQueues != nullptr)
	{
		delete[] m_PoseSensorOpticalPacketQueues;
		m_PoseSensorOpticalPacketQueues= nullptr;
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

bool ServerControllerView::open(const class DeviceEnumerator *enumerator)
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
        case CommonSensorState::PSMove:
        case CommonSensorState::DualShock4:
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

void ServerControllerView::close()
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

bool ServerControllerView::recenterOrientation(const PSVRQuatf& q_pose_relative_to_identity_pose)
{
    bool bSuccess = false;
    IPoseFilter *filter = getPoseFilterMutable();

    if (filter != nullptr)
    {
        // Get the pose that we expect the controller to be in (relative to the pose it's in by default).
        // For example, the psmove controller's default mesh has it laying flat,
        // but when we call reset_orientation in the HMD alignment tool, we expect the controller is pointing up.
        const Eigen::Quaternionf q_pose(
            q_pose_relative_to_identity_pose.w,
            q_pose_relative_to_identity_pose.x,
            q_pose_relative_to_identity_pose.y,
            q_pose_relative_to_identity_pose.z);

        // Get the rotation that would align the global +X axis with the global forward direction
        const float global_forward_degrees = DeviceManager::getInstance()->m_tracker_manager->getConfig().global_forward_degrees;
        const float global_forward_radians = global_forward_degrees * k_degrees_to_radians;
        const Eigen::EulerAnglesf global_forward_euler(Eigen::Vector3f(0.f, global_forward_radians, 0.f));
        const Eigen::Quaternionf global_forward_quat = eigen_euler_angles_to_quaternionf(global_forward_euler);

        // Get the rotation that would align the global +X axis with the controller identity forward
        const float controller_forward_degrees = m_device->getIdentityForwardDegrees();
        const float controller_forward_radians = global_forward_degrees * k_degrees_to_radians;
        const Eigen::EulerAnglesf controller_forward_euler(Eigen::Vector3f(0.f, controller_forward_radians, 0.f));
        const Eigen::Quaternionf controller_forward_quat = eigen_euler_angles_to_quaternionf(controller_forward_euler);

        // Compute the relative rotation from global forward to controller identity forward.
        // If the controllers identity forward and global forward are the same this will cancel out.
        // Usually both are -Z.
        const Eigen::Quaternionf identity_pose_relative_to_global_forward= 
            eigen_quaternion_concatenate(global_forward_quat.conjugate(), controller_forward_quat);

        // Compute the pose that the controller claims to be in relative to global forward.
        // For the normal re-centering case q_pose_relative_to_identity_pose is the identity quaternion.
        const Eigen::Quaternionf controller_pose_relative_to_global_forward=
            eigen_quaternion_concatenate(q_pose, identity_pose_relative_to_global_forward);

        // Tell the pose filter that the orientation state should now be relative to controller_pose_relative_to_global_forward
        filter->recenterOrientation(controller_pose_relative_to_global_forward);
        bSuccess = true;
    }

    return bSuccess;
}

void ServerControllerView::resetPoseFilter()
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
    case CommonSensorState::PSMove:
        {
            init_filters_for_psmove(
                static_cast<PSMoveController *>(m_device),
                &m_pose_filter_space, &m_pose_filter);
        } break;
    case CommonSensorState::DualShock4:
        {
            init_filters_for_dualshock4(
                static_cast<DualShock4Controller *>(m_device),
                &m_pose_filter_space, &m_pose_filter);
        } break;
    }

	m_bIsLastFilterUpdateTimestampValid= false;
	m_bIsLastSensorDataTimestampValid= false;
}

void ServerControllerView::notifyTrackerDataReceived(ServerTrackerView* tracker)
{
    const t_high_resolution_timepoint now= std::chrono::high_resolution_clock::now();

	int tracker_id= tracker->getDeviceID();
	ControllerOpticalPoseEstimation &tracker_pose_estimate_ref = m_optical_pose_estimations[tracker_id];
    IShapeTrackingModel *shape_tracking_model= m_shape_tracking_models[tracker_id];
	t_controller_optical_sensor_queue &optical_sensor_queue= m_PoseSensorOpticalPacketQueues[tracker_id];

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
        if (tracker->computeProjectionForController(
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
		switch (getControllerDeviceType())
		{
		case CommonSensorState::PSMove:
			{
				const PSMoveController *psmove = this->castCheckedConst<PSMoveController>();

				// Only update the position filter when tracking is enabled
				post_optical_filter_packet_for_psmove(
					psmove,
					tracker_id,
					now,
					&tracker_pose_estimate_ref,
					&optical_sensor_queue);
			} break;
		case CommonSensorState::DualShock4:
			{
				const DualShock4Controller *ds4 = this->castCheckedConst<DualShock4Controller>();

				// Only update the position filter when tracking is enabled
				post_optical_filter_packet_for_dualshock4(
					ds4,
					tracker_id,
					now,
					&tracker_pose_estimate_ref,
					&optical_sensor_queue);
			} break;
		default:
			assert(0 && "Unhandled controller type");
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
ServerControllerView::notifySensorDataReceived(const CommonSensorState *sensor_state)
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
    case CommonSensorState::PSMove:
        {
            const PSMoveController *psmove = this->castCheckedConst<PSMoveController>();
            const PSMoveControllerInputState *psmoveState = static_cast<const PSMoveControllerInputState *>(sensor_state);

            // Only update the position filter when tracking is enabled
            post_imu_filter_packets_for_psmove(
                psmove, psmoveState,
                now, durationSinceLastUpdate,
				&m_PoseSensorIMUPacketQueue);
        } break;
    case CommonSensorState::DualShock4:
        {
            const DualShock4Controller *ds4 = this->castCheckedConst<DualShock4Controller>();
            const DualShock4ControllerInputState *ds4State = static_cast<const DualShock4ControllerInputState *>(sensor_state);

            // Only update the position filter when tracking is enabled
            post_imu_filter_packets_for_dualshock4(
                ds4, ds4State,
                now, durationSinceLastUpdate,
				&m_PoseSensorIMUPacketQueue);
        } break;
    default:
        assert(0 && "Unhandled Controller type");
    }

    // Consider this HMD state sequence num processed
    m_lastPollSeqNumProcessed = sensor_state->PollSequenceNumber;
}

// Update Pose Filter using update packets from the tracker and IMU threads
void ServerControllerView::updatePoseFilter()
{
	std::vector<PoseSensorPacket> timeSortedPackets;

	// Drain the packet queues filled by the threads
	PoseSensorPacket packet;
	while (m_PoseSensorIMUPacketQueue.try_dequeue(packet))
	{
		timeSortedPackets.push_back(packet);
	}
	for (int tracker_id = 0; tracker_id < TrackerManager::k_max_devices; ++tracker_id)
	{
		while (m_PoseSensorOpticalPacketQueues[tracker_id].try_dequeue(packet))
		{
			timeSortedPackets.push_back(packet);
		}
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

bool ServerControllerView::setHostBluetoothAddress(
    const std::string &address)
{
    return m_device->setHostBluetoothAddress(address);
}

PSVRPosef
ServerControllerView::getFilteredPose(float time) const
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
ServerControllerView::getFilteredPhysics() const
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

bool 
ServerControllerView::getIsBluetooth() const
{
    return (m_device != nullptr) ? m_device->getIsBluetooth() : false;
}

bool 
ServerControllerView::getUsesBluetoothAuthentication() const
{
	bool bUsesAuthentication= false;

	if (m_device != nullptr && m_device->getDeviceType() == CommonSensorState::PSMove)
	{
        const PSMoveController *psmove = this->castCheckedConst<PSMoveController>();

		bUsesAuthentication= psmove->getIsPS4Controller();
	}

	return bUsesAuthentication;
}

bool
ServerControllerView::getIsStreamable() const
{
    bool bIsStreamableController= false;

    if (getIsOpen())
    {
        switch (getControllerDeviceType())
        {
        case CommonSensorState::PSMove:
        case CommonSensorState::DualShock4:
            {
                bIsStreamableController= getIsBluetooth();
            } break;
        }
    }

    return bIsStreamableController;
}

// Returns the full usb device path for the controller
std::string 
ServerControllerView::getUSBDevicePath() const
{
    return (m_device != nullptr) ? m_device->getUSBDevicePath() : "";
}

// Returns the vendor ID of the controller
int 
ServerControllerView::getVendorID() const
{
    return (m_device != nullptr) ? m_device->getVendorID() : -1;
}

// Returns the product ID of the controller
int 
ServerControllerView::getProductID() const
{
    return (m_device != nullptr) ? m_device->getProductID() : -1;
}

// Returns the serial number for the controller
std::string 
ServerControllerView::getSerial() const
{
    return (m_device != nullptr) ? m_device->getSerial() : "";
}

// Returns the "controller_" + serial number for the controller
std::string
ServerControllerView::getConfigIdentifier() const
{
    std::string	identifier= "";

    if (m_device != nullptr)
    {
        std::string	prefix= "controller_";
        
        identifier= prefix+m_device->getSerial();
    }

    return identifier;
}

std::string 
ServerControllerView::getAssignedHostBluetoothAddress() const
{
    return (m_device != nullptr) ? m_device->getAssignedHostBluetoothAddress() : "";
}

CommonSensorState::eDeviceType
ServerControllerView::getControllerDeviceType() const
{
    return m_device->getDeviceType();
}

const PoseSensorPacket *ServerControllerView::getLastOpticalSensorPacket(int tracker_id) const
{ 
    return (tracker_id < TrackerManager::k_max_devices) ? &m_lastOpticalSensorPacket[tracker_id] : nullptr; 
}

const struct CommonControllerState * ServerControllerView::getControllerState() const
{
    const struct CommonControllerState *device_state = m_device->getControllerState();
    assert(device_state == nullptr ||
        ((int)device_state->DeviceType >= (int)CommonSensorState::Controller &&
        device_state->DeviceType < CommonSensorState::SUPPORTED_CONTROLLER_TYPE_COUNT));

    return static_cast<const CommonControllerState *>(device_state);
}

void ServerControllerView::setLEDOverride(unsigned char r, unsigned char g, unsigned char b)
{
    m_LED_override_color = std::make_tuple(r, g, b);
    m_LED_override_active = true;
    update_LED_color_internal();
}

void ServerControllerView::clearLEDOverride()
{
    m_LED_override_color = std::make_tuple(0x00, 0x00, 0x00);
    m_LED_override_active = false;
    update_LED_color_internal();
}

PSVRTrackingColorType ServerControllerView::getTrackingColorID() const
{
    PSVRTrackingColorType tracking_color_id = PSVRTrackingColorType_INVALID;

    if (m_device != nullptr)
    {
        m_device->getTrackingColorID(tracking_color_id);
    }

    return tracking_color_id;
}

bool ServerControllerView::setTrackingColorID(PSVRTrackingColorType colorID)
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

void ServerControllerView::startTracking()
{
    if (!m_tracking_enabled)
    {
        set_tracking_enabled_internal(true);
    }

    ++m_tracking_listener_count;
}

void ServerControllerView::stopTracking()
{
    assert(m_tracking_listener_count > 0);
    --m_tracking_listener_count;

    if (m_tracking_listener_count <= 0 && m_tracking_enabled)
    {
        set_tracking_enabled_internal(false);
    }
}

void ServerControllerView::set_tracking_enabled_internal(bool bEnabled)
{
    if (m_tracking_enabled != bEnabled)
    {
        if (bEnabled)
        {
            assert(m_device != nullptr);

            PSVRTrackingColorType tracking_color_id= PSVRTrackingColorType_INVALID;
            m_device->getTrackingColorID(tracking_color_id);

            switch (tracking_color_id)
            {
            case PSVRTrackingColorType_Magenta:
                m_tracking_color= std::make_tuple(0xFF, 0x00, 0xFF);
                break;
            case PSVRTrackingColorType_Cyan:
                m_tracking_color = std::make_tuple(0x00, 0xFF, 0xFF);
                break;
            case PSVRTrackingColorType_Yellow:
                m_tracking_color = std::make_tuple(0xFF, 0xFF, 0x00);
                break;
            case PSVRTrackingColorType_Red:
                m_tracking_color = std::make_tuple(0xFF, 0x00, 0x00);
                break;
            case PSVRTrackingColorType_Green:
                m_tracking_color = std::make_tuple(0x00, 0xFF, 0x00);
                break;
            case PSVRTrackingColorType_Blue:
                m_tracking_color = std::make_tuple(0x00, 0x00, 0xFF);
                break;
            default:
                assert(0 && "unreachable");
            }
        }
        else
        {
            m_tracking_color = std::make_tuple(0x00, 0x00, 0x00);
        }

        m_tracking_enabled = bEnabled;

        update_LED_color_internal();
    }
}

void ServerControllerView::update_LED_color_internal()
{
    unsigned char r, g, b;
    if (m_LED_override_active)
    {
        r = std::get<0>(m_LED_override_color);
        g = std::get<1>(m_LED_override_color);
        b = std::get<2>(m_LED_override_color);
    }
    else if (m_tracking_enabled)
    {
        r = std::get<0>(m_tracking_color);
        g = std::get<1>(m_tracking_color);
        b = std::get<2>(m_tracking_color);
    }
    else
    {
        r = g = b = 0;
    }

    switch (getControllerDeviceType())
    {
    case CommonSensorState::PSMove:
        {
            this->castChecked<PSMoveController>()->setLED(r, g, b);
        } break;
    case CommonSensorState::DualShock4:
        {
            this->castChecked<DualShock4Controller>()->setLED(r, g, b);
        } break;
    default:
        assert(false && "Unhanded controller type!");
    }
}

// Get the tracking shape for the controller
bool ServerControllerView::getTrackingShape(PSVRTrackingShape &trackingShape) const
{
    m_device->getTrackingShape(trackingShape);

    return trackingShape.shape_type != PSVRTrackingShape_INVALID;
}

float ServerControllerView::getROIPredictionTime() const
{
    static float k_max_roi_prediction_speec_cm = 30.f;
    static float k_max_roi_prediction_time = 0.1f;

    const Eigen::Vector3f velocityCmPerSec= getPoseFilter()->getVelocityCmPerSec();
    const float speedCmPerSec= velocityCmPerSec.norm();
    const float predictionTime = clampf01(speedCmPerSec / k_max_roi_prediction_speec_cm)*k_max_roi_prediction_time;

    return predictionTime;
}

// Set the rumble value between 0.f - 1.f on a given channel
bool ServerControllerView::setControllerRumble(
    float rumble_amount,
    PSVRControllerRumbleChannel channel)
{
    bool result= false;

    if (getIsOpen())
    {
        switch(getControllerDeviceType())
        {
        case CommonSensorState::PSMove:
            {
                unsigned char rumble_byte= static_cast<unsigned char>(clampf01(rumble_amount)*255.f);

                static_cast<PSMoveController *>(m_device)->setRumbleIntensity(rumble_byte);
                result = true;
            } break;

        case CommonSensorState::DualShock4:
            {
                unsigned char rumble_byte = static_cast<unsigned char>(clampf01(rumble_amount)*255.f);
                DualShock4Controller *controller= static_cast<DualShock4Controller *>(m_device);

                if (channel == PSVRControllerRumbleChannel_Left ||
                    channel == PSVRControllerRumbleChannel_All)
                {
                    controller->setLeftRumbleIntensity(rumble_byte);
                }

                if (channel == PSVRControllerRumbleChannel_Right ||
                    channel == PSVRControllerRumbleChannel_All)
                {
                    controller->setRightRumbleIntensity(rumble_byte);
                }

                result = true;
            } break;

        default:
            assert(false && "Unhanded controller type!");
        }
    }

    return result;
}

void ServerControllerView::publish_device_data_frame()
{
    // Tell the server request handler we want to send out HMD updates.
    // This will call generate_hmd_data_frame_for_stream for each listening connection.
    ServiceRequestHandler::get_instance()->publish_controller_data_frame(
        this, &ServerControllerView::generate_controller_data_frame_for_stream);
}

void ServerControllerView::generate_controller_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    ControllerOutputDataPacket *controller_data_frame= &data_frame.device.controller_data_packet;

    controller_data_frame->controller_id= controller_view->getDeviceID();
    controller_data_frame->output_sequence_num= controller_view->m_sequence_number;
    controller_data_frame->is_connected= controller_view->getDevice()->getIsOpen();
	controller_data_frame->is_valid= true;

    switch (controller_view->getControllerDeviceType())
    {
    case CommonControllerState::PSMove:
        {            
            generate_psmove_data_frame_for_stream(controller_view, stream_info, data_frame);
        } break;
    case CommonControllerState::DualShock4:
        {
            generate_psdualshock4_data_frame_for_stream(controller_view, stream_info, data_frame);
        } break;
    default:
        assert(0 && "Unhandled controller type");
    }

	data_frame.device_category= DeviceCategory_CONTROLLER;
}

static void generate_psmove_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    const PSMoveController *psmove_controller= controller_view->castCheckedConst<PSMoveController>();
    const IPoseFilter *pose_filter= controller_view->getPoseFilter();
    const PSMoveControllerConfig *psmove_config= psmove_controller->getConfig();
    const PoseSensorPacket *imu_sensor_packet = controller_view->getLastIMUSensorPacket();
    const PSVRPosef controller_pose = controller_view->getFilteredPose();

    ControllerOutputDataPacket *controller_data_frame = &data_frame.device.controller_data_packet;
    PSVRPSMove *psmove_data_frame = &controller_data_frame->controller_state.psmove_state;
   
    if (imu_sensor_packet != nullptr)
    {
		const double time_seconds= pose_filter->getTimeInSeconds();
        const PSMoveControllerInputState * psmove_state= 
			static_cast<const PSMoveControllerInputState *>(controller_view->getControllerState());

        psmove_data_frame->bHasValidHardwareCalibration= psmove_config->is_valid;
        psmove_data_frame->bIsCurrentlyTracking= controller_view->getIsCurrentlyTracking();
        psmove_data_frame->bIsTrackingEnabled= controller_view->getIsTrackingEnabled();
        psmove_data_frame->bIsOrientationValid= pose_filter->getIsOrientationStateValid();
        psmove_data_frame->bIsPositionValid= pose_filter->getIsPositionStateValid();

        psmove_data_frame->Pose.Orientation= controller_pose.Orientation;

        if (stream_info->include_position_data)
        {
            psmove_data_frame->Pose.Position= controller_pose.Position;
        }
        else
        {
            psmove_data_frame->Pose.Position= *k_PSVR_float_vector3_zero;
        }

        psmove_data_frame->TriggerValue= psmove_state->TriggerValue;
        psmove_data_frame->BatteryValue= psmove_state->BatteryValue;
		psmove_data_frame->TriangleButton= psmove_state->Triangle;
		psmove_data_frame->CircleButton= psmove_state->Circle;
		psmove_data_frame->CrossButton= psmove_state->Cross;
		psmove_data_frame->SquareButton= psmove_state->Square;
		psmove_data_frame->SelectButton= psmove_state->Select;
		psmove_data_frame->StartButton= psmove_state->Start;
		psmove_data_frame->PSButton= psmove_state->PS;
		psmove_data_frame->MoveButton= psmove_state->Move;
		psmove_data_frame->TriggerButton= psmove_state->Trigger;

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            PSVRPSMoveRawSensorData *raw_sensor_data= &psmove_data_frame->RawSensorData;
			memset(raw_sensor_data, 0, sizeof(PSVRPSMoveRawSensorData));

            // One frame: [mx, my, mz] 
			if (imu_sensor_packet->has_magnetometer_measurement)
			{
				raw_sensor_data->Magnetometer= imu_sensor_packet->raw_imu_magnetometer;
			}

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
			if (imu_sensor_packet->has_accelerometer_measurement)
			{
				raw_sensor_data->Accelerometer= imu_sensor_packet->raw_imu_accelerometer;
			}

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
			if (imu_sensor_packet->has_gyroscope_measurement)
			{
				raw_sensor_data->Gyroscope= imu_sensor_packet->raw_imu_gyroscope;
			}

			raw_sensor_data->TimeInSeconds= time_seconds;
        }

        // If requested, get the calibrated sensor data for the controller
        if (stream_info->include_calibrated_sensor_data)
        {
            PSVRPSMoveCalibratedSensorData *calibrated_sensor_data = &psmove_data_frame->CalibratedSensorData;
			memset(calibrated_sensor_data, 0, sizeof(PSVRPSMoveCalibratedSensorData));

            // One frame: [mx, my, mz]
			if (imu_sensor_packet->has_magnetometer_measurement)
			{
				calibrated_sensor_data->Magnetometer= 
					eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_magnetometer_unit);
			}

            // Two frames: [[ax0, ay0, az0], [ax1, ay1, az1]] 
            // Take the most recent frame: [ax1, ay1, az1]
			if (imu_sensor_packet->has_accelerometer_measurement)
			{
				calibrated_sensor_data->Accelerometer= 
					eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_accelerometer_g_units);
			}

            // Two frames: [[wx0, wy0, wz0], [wx1, wy1, wz1]] 
            // Take the most recent frame: [wx1, wy1, wz1]
			if (imu_sensor_packet->has_gyroscope_measurement)
			{
				calibrated_sensor_data->Gyroscope= 
					eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_gyroscope_rad_per_sec);
			}

			calibrated_sensor_data->TimeInSeconds= time_seconds;
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSVRRawTrackerData *raw_tracker_data = &psmove_data_frame->RawTrackerData;
            int selectedTrackerId= stream_info->selected_tracker_index;

			raw_tracker_data->ValidTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
				const PoseSensorPacket *optical_packet= 
					controller_view->getLastOpticalSensorPacket(trackerId);

                if (optical_packet != nullptr && controller_view->getIsTrackedByTracker(trackerId))
                {
                    raw_tracker_data->ValidTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
						const PSVRVector3f trackerRelativePosition=
							eigen_vector3f_to_PSVR_vector3f(optical_packet->optical_position_cm);
                        const ServerTrackerViewPtr tracker_view = 
							DeviceManager::getInstance()->getTrackerViewPtr(selectedTrackerId);

                        // Use the center of the ellipse projection as our "screen location"
						if (tracker_view->getIsStereoCamera())
                        {
                            const PSVRTrackingProjectionData &left_projection_data=
                                optical_packet->optical_tracking_projection.projections[PSVRVideoFrameSection_Left];
                            const PSVRTrackingProjectionData &right_projection_data =
                                optical_packet->optical_tracking_projection.projections[PSVRVideoFrameSection_Right];

                            raw_tracker_data->ScreenLocations[PSVRVideoFrameSection_Left] =
                                left_projection_data.shape.ellipse.center;
							raw_tracker_data->ScreenLocations[PSVRVideoFrameSection_Right]= 
                                right_projection_data.shape.ellipse.center;
                        }
						else
						{
                            const PSVRTrackingProjectionData &mono_projection_data =
                                optical_packet->optical_tracking_projection.projections[PSVRVideoFrameSection_Primary];

                            raw_tracker_data->ScreenLocations[PSVRVideoFrameSection_Primary]=
                                mono_projection_data.shape.ellipse.center;
						}

                        // Add the tracker relative 3d position
						raw_tracker_data->RelativePositionCm= 
							eigen_vector3f_to_PSVR_vector3f(optical_packet->optical_position_cm);
						raw_tracker_data->RelativeOrientation=
							eigen_quaternionf_to_PSVR_quatf(optical_packet->optical_orientation);

                        // Add the tracker relative projection shapes
						raw_tracker_data->TrackingProjection= optical_packet->optical_tracking_projection;

						// Add the world relative tracking shape
						raw_tracker_data->WorldRelativeShape= optical_packet->optical_tracking_shape_cm;

                        raw_tracker_data->TrackerID= selectedTrackerId;
                    }
                }
            }
        }

        // if requested, get the physics data for the controller
        if (stream_info->include_physics_data)
        {
            psmove_data_frame->PhysicsData = controller_view->getFilteredPhysics();
        }
    }   

    controller_data_frame->controller_type= PSVRController_Move;
}

static void generate_psdualshock4_data_frame_for_stream(
    const ServerControllerView *controller_view,
    const ControllerStreamInfo *stream_info,
    DeviceOutputDataFrame &data_frame)
{
    const DualShock4Controller *ds4_controller = controller_view->castCheckedConst<DualShock4Controller>();
    const IPoseFilter *pose_filter= controller_view->getPoseFilter();
    const DualShock4ControllerConfig *ds4_config = ds4_controller->getConfig();
    const PoseSensorPacket *imu_sensor_packet = controller_view->getLastIMUSensorPacket();
    const PSVRPosef controller_pose = controller_view->getFilteredPose();

    ControllerOutputDataPacket *controller_data_frame = &data_frame.device.controller_data_packet;
    PSVRDualShock4 *ds4_data_frame = &controller_data_frame->controller_state.ds4_state;

    if (imu_sensor_packet != nullptr)
    {
		const double time_seconds= pose_filter->getTimeInSeconds();
        const DualShock4ControllerInputState * ds4_state= 
			static_cast<const DualShock4ControllerInputState *>(controller_view->getControllerState());

        ds4_data_frame->bHasValidHardwareCalibration= ds4_config->is_valid;
        ds4_data_frame->bIsCurrentlyTracking= controller_view->getIsCurrentlyTracking();
        ds4_data_frame->bIsTrackingEnabled= controller_view->getIsTrackingEnabled();
        ds4_data_frame->bIsOrientationValid= pose_filter->getIsOrientationStateValid();
        ds4_data_frame->bIsPositionValid= pose_filter->getIsPositionStateValid();

        ds4_data_frame->Pose.Orientation= controller_pose.Orientation;

        if (stream_info->include_position_data)
        {
            ds4_data_frame->Pose.Position= controller_pose.Position;
        }
        else
        {
            ds4_data_frame->Pose.Position= *k_PSVR_float_vector3_zero;
        }

        ds4_data_frame->LeftAnalogX= ds4_state->LeftAnalogX;
        ds4_data_frame->LeftAnalogY= ds4_state->LeftAnalogY;

        ds4_data_frame->RightAnalogX= ds4_state->RightAnalogX;
        ds4_data_frame->RightAnalogY= ds4_state->RightAnalogY;

        ds4_data_frame->LeftTriggerValue= ds4_state->LeftTrigger;
        ds4_data_frame->RightTriggerValue= ds4_state->RightTrigger;

		ds4_data_frame->DPadUpButton= ds4_state->DPad_Up;
		ds4_data_frame->DPadDownButton= ds4_state->DPad_Down;
		ds4_data_frame->DPadLeftButton= ds4_state->DPad_Left;
		ds4_data_frame->DPadRightButton= ds4_state->DPad_Right;

		ds4_data_frame->L1Button= ds4_state->L1;
		ds4_data_frame->R1Button= ds4_state->R1;
		ds4_data_frame->L2Button= ds4_state->L2;
		ds4_data_frame->R2Button= ds4_state->R2;
		ds4_data_frame->L3Button= ds4_state->L3;
		ds4_data_frame->R3Button= ds4_state->R3;

		ds4_data_frame->TriangleButton= ds4_state->Triangle;
		ds4_data_frame->CircleButton= ds4_state->Circle;
		ds4_data_frame->CrossButton= ds4_state->Cross;
		ds4_data_frame->SquareButton= ds4_state->Square;

		ds4_data_frame->ShareButton= ds4_state->Share;
		ds4_data_frame->OptionsButton= ds4_state->Options;

		ds4_data_frame->PSButton= ds4_state->PS;
		ds4_data_frame->TrackPadButton= ds4_state->TrackPadButton;

        // If requested, get the raw sensor data for the controller
        if (stream_info->include_raw_sensor_data)
        {
            PSVRDS4RawSensorData *raw_sensor_data= &ds4_data_frame->RawSensorData;
			memset(raw_sensor_data, 0, sizeof(PSVRPSMoveRawSensorData));

			if (imu_sensor_packet->has_accelerometer_measurement)
			{
				raw_sensor_data->Accelerometer= imu_sensor_packet->raw_imu_accelerometer;
			}

			if (imu_sensor_packet->has_gyroscope_measurement)
			{
				raw_sensor_data->Gyroscope= imu_sensor_packet->raw_imu_gyroscope;
			}

			raw_sensor_data->TimeInSeconds= time_seconds;
        }

        // If requested, get the calibrated sensor data for the controller
        if (stream_info->include_calibrated_sensor_data)
        {
            PSVRDS4CalibratedSensorData *calibrated_sensor_data = &ds4_data_frame->CalibratedSensorData;
			memset(calibrated_sensor_data, 0, sizeof(PSVRDS4CalibratedSensorData));

			if (imu_sensor_packet->has_accelerometer_measurement)
			{
				calibrated_sensor_data->Accelerometer=
					eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_accelerometer_g_units);
			}

			if (imu_sensor_packet->has_gyroscope_measurement)
			{
				calibrated_sensor_data->Gyroscope=
					eigen_vector3f_to_PSVR_vector3f(imu_sensor_packet->imu_gyroscope_rad_per_sec);
			}

			calibrated_sensor_data->TimeInSeconds= time_seconds;
        }

        // If requested, get the raw tracker data for the controller
        if (stream_info->include_raw_tracker_data)
        {
            PSVRRawTrackerData *raw_tracker_data = &ds4_data_frame->RawTrackerData;
            int selectedTrackerId= stream_info->selected_tracker_index;

			raw_tracker_data->ValidTrackerBitmask= 0;

            for (int trackerId = 0; trackerId < TrackerManager::k_max_devices; ++trackerId)
            {
				const PoseSensorPacket *optical_packet= 
					controller_view->getLastOpticalSensorPacket(trackerId);

                if (optical_packet != nullptr && controller_view->getIsTrackedByTracker(trackerId))
                {
                    raw_tracker_data->ValidTrackerBitmask&= (1 << trackerId);

                    if (trackerId == selectedTrackerId)
                    {
						const PSVRVector3f trackerRelativePosition=
							eigen_vector3f_to_PSVR_vector3f(optical_packet->optical_position_cm);
                        const ServerTrackerViewPtr tracker_view = 
							DeviceManager::getInstance()->getTrackerViewPtr(selectedTrackerId);

                        // Project the 3d camera position back onto the tracker screen
						if (tracker_view->getIsStereoCamera())
                        {
							raw_tracker_data->ScreenLocations[PSVRVideoFrameSection_Left]= 
                                tracker_view->projectTrackerRelativePosition(
									PSVRVideoFrameSection_Left, &trackerRelativePosition);
							raw_tracker_data->ScreenLocations[PSVRVideoFrameSection_Right]= 
                                tracker_view->projectTrackerRelativePosition(
									PSVRVideoFrameSection_Right, &trackerRelativePosition);
                        }
						else
						{
							raw_tracker_data->ScreenLocations[0]= 
                                tracker_view->projectTrackerRelativePosition(
									PSVRVideoFrameSection_Primary, &trackerRelativePosition);
						}

                        // Add the tracker relative 3d position
						raw_tracker_data->RelativePositionCm= 
							eigen_vector3f_to_PSVR_vector3f(optical_packet->optical_position_cm);
						raw_tracker_data->RelativeOrientation=
							eigen_quaternionf_to_PSVR_quatf(optical_packet->optical_orientation);

                        // Add the tracker relative projection shapes
						raw_tracker_data->TrackingProjection= optical_packet->optical_tracking_projection;

						// Add the world relative tracking shape
						raw_tracker_data->WorldRelativeShape= optical_packet->optical_tracking_shape_cm;

                        raw_tracker_data->TrackerID= selectedTrackerId;
                    }
                }
            }
        }

        // if requested, get the physics data for the controller
        if (stream_info->include_physics_data)
        {
			ds4_data_frame->PhysicsData = controller_view->getFilteredPhysics();
        }
    }

    controller_data_frame->controller_type= PSVRController_DualShock4;
}

static IPoseFilter *
pose_filter_factory(
    const CommonSensorState::eDeviceType deviceType,
    const std::string &position_filter_type,
    const std::string &orientation_filter_type,
    const PoseFilterConstants &constants)
{
    static IPoseFilter *filter= nullptr;

    if (position_filter_type == "PoseKalman" && orientation_filter_type == "PoseKalman")
    {
        switch (deviceType)
        {
        case CommonSensorState::PSMove:
            {
                KalmanPoseFilterPSMove *kalmanFilter = new KalmanPoseFilterPSMove();
                kalmanFilter->init(constants);
                filter= kalmanFilter;
            } break;
        case CommonSensorState::DualShock4:
            {
                KalmanPoseFilterDS4 *kalmanFilter = new KalmanPoseFilterDS4();
                kalmanFilter->init(constants);
                filter= kalmanFilter;
            } break;
        default:
            assert(0 && "unreachable");
        }
    }
    else
    {
        // Convert the position filter type string into an enum
        PositionFilterType position_filter_enum= PositionFilterTypeNone;
        if (position_filter_type == "")
        {
            position_filter_enum= PositionFilterTypeNone;
        }
        else if (position_filter_type == "PassThru")
        {
            position_filter_enum= PositionFilterTypePassThru;
        }
        else if (position_filter_type == "LowPassOptical")
        {
            position_filter_enum= PositionFilterTypeLowPassOptical;
        }
        else if (position_filter_type == "LowPassIMU")
        {
            position_filter_enum= PositionFilterTypeLowPassIMU;
        }
        else if (position_filter_type == "LowPassExponential")
        {
            position_filter_enum = PositionFilterTypeLowPassExponential;
        }
        else if (position_filter_type == "ComplimentaryOpticalIMU")
        {
            position_filter_enum= PositionFilterTypeComplimentaryOpticalIMU;
        }
        else if (position_filter_type == "PositionKalman")
        {
            position_filter_enum= PositionFilterTypeKalman;
        }
        else
        {
            PSVR_LOG_INFO("pose_filter_factory()") << 
                "Unknown position filter type: " << position_filter_type << ". Using default.";

            // fallback to a default based on controller type
            switch (deviceType)
            {
            case CommonSensorState::PSMove:
                position_filter_enum= PositionFilterTypeLowPassExponential;
                break;
            case CommonSensorState::DualShock4:
                position_filter_enum= PositionFilterTypeComplimentaryOpticalIMU;
                break;
            default:
                assert(0 && "unreachable");
            }
        }
        
        // Convert the orientation filter type string into an enum
        OrientationFilterType orientation_filter_enum= OrientationFilterTypeNone;
        if (orientation_filter_type == "")
        {
            orientation_filter_enum= OrientationFilterTypeNone;
        }
        else if (orientation_filter_type == "PassThru")
        {
            orientation_filter_enum= OrientationFilterTypePassThru;
        }
        else if (orientation_filter_type == "MadgwickARG")
        {
            orientation_filter_enum= OrientationFilterTypeMadgwickARG;
        }
        else if (orientation_filter_type == "MadgwickMARG")
        {
            orientation_filter_enum= OrientationFilterTypeMadgwickMARG;
        }
        else if (orientation_filter_type == "ComplementaryOpticalARG")
        {
            orientation_filter_enum= OrientationFilterTypeComplementaryOpticalARG;
        }
        else if (orientation_filter_type == "ComplementaryMARG")
        {
            orientation_filter_enum= OrientationFilterTypeComplementaryMARG;
        }
        else if (orientation_filter_type == "OrientationKalman")
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
            case CommonSensorState::PSMove:
                orientation_filter_enum= OrientationFilterTypeComplementaryMARG;
                break;
            case CommonSensorState::DualShock4:
                orientation_filter_enum= OrientationFilterTypeComplementaryOpticalARG;
                break;
            default:
                assert(0 && "unreachable");
            }
        }

        CompoundPoseFilter *compound_pose_filter = new CompoundPoseFilter();
        compound_pose_filter->init(deviceType, orientation_filter_enum, position_filter_enum, constants);
        filter= compound_pose_filter;
    }

    assert(filter != nullptr);

    return filter;
}

static void
init_filters_for_psmove(
    const PSMoveController *psmoveController, 
    PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter)
{
    const PSMoveControllerConfig *psmove_config = psmoveController->getConfig();

        // Setup the space the orientation filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(Eigen::Vector3f(0.f, 1.f, 0.f));
    pose_filter_space->setIdentityMagnetometer(
        Eigen::Vector3f(psmove_config->magnetometer_identity.x,
            psmove_config->magnetometer_identity.y,
                        psmove_config->magnetometer_identity.z));
    pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_laying_flat);
    pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_opengl);

    // Copy the pose filter constants from the controller config
    PoseFilterConstants constants;
    constants.clear();

	psmoveController->getTrackingShape(constants.orientation_constants.tracking_shape);
    constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.orientation_constants.accelerometer_variance =
        Eigen::Vector3f(psmove_config->accelerometer_variance, psmove_config->accelerometer_variance, psmove_config->accelerometer_variance);
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();

    constants.orientation_constants.gyro_drift= 
        Eigen::Vector3f(psmove_config->gyro_drift, psmove_config->gyro_drift, psmove_config->gyro_drift);
    constants.orientation_constants.gyro_variance= 
        Eigen::Vector3f(psmove_config->gyro_variance, psmove_config->gyro_variance, psmove_config->gyro_variance);
    constants.orientation_constants.mean_update_time_delta= psmove_config->mean_update_time_delta;
    constants.orientation_constants.position_variance_curve.A = psmove_config->position_variance_exp_fit_a;
    constants.orientation_constants.position_variance_curve.B = psmove_config->position_variance_exp_fit_b;
    constants.orientation_constants.orientation_variance_curve.A = psmove_config->orientation_variance;
    constants.orientation_constants.orientation_variance_curve.B = 0.f;
    constants.orientation_constants.orientation_variance_curve.MaxValue = psmove_config->orientation_variance;
    constants.orientation_constants.magnetometer_variance= 
        Eigen::Vector3f(psmove_config->magnetometer_variance, psmove_config->magnetometer_variance, psmove_config->magnetometer_variance);
    constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero();

    constants.position_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.position_constants.accelerometer_variance= 
        Eigen::Vector3f(psmove_config->accelerometer_variance, psmove_config->accelerometer_variance, psmove_config->accelerometer_variance);
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_noise_radius= psmove_config->accelerometer_noise_radius;
    constants.position_constants.max_velocity= psmove_config->max_velocity;
    constants.position_constants.mean_update_time_delta= psmove_config->mean_update_time_delta;
    constants.position_constants.position_variance_curve.A = psmove_config->position_variance_exp_fit_a;
    constants.position_constants.position_variance_curve.B = psmove_config->position_variance_exp_fit_b;
    constants.position_constants.position_variance_curve.MaxValue = 1.f;

    *out_pose_filter_space= pose_filter_space;
    *out_pose_filter= pose_filter_factory(
        CommonSensorState::PSMove,
        psmove_config->position_filter_type,
        psmove_config->orientation_filter_type,
        constants);
}

static void
init_filters_for_dualshock4(
    const DualShock4Controller *ds4Controller,
    PoseFilterSpace **out_pose_filter_space,
    IPoseFilter **out_pose_filter)
{
    const DualShock4ControllerConfig *ds4_config = ds4Controller->getConfig();

        // Setup the space the orientation filter operates in
    PoseFilterSpace *pose_filter_space = new PoseFilterSpace();
    pose_filter_space->setIdentityGravity(
            Eigen::Vector3f(
                ds4_config->identity_gravity_direction.x,
                ds4_config->identity_gravity_direction.y,
            ds4_config->identity_gravity_direction.z));
    pose_filter_space->setIdentityMagnetometer(Eigen::Vector3f::Zero());  // No magnetometer on DS4 :(
    pose_filter_space->setCalibrationTransform(*k_eigen_identity_pose_upright);
    pose_filter_space->setSensorTransform(*k_eigen_sensor_transform_identity);

    // Copy the pose filter constants from the controller config
    PoseFilterConstants constants;
    constants.clear();

	ds4Controller->getTrackingShape(constants.orientation_constants.tracking_shape);
    constants.orientation_constants.gravity_calibration_direction = pose_filter_space->getGravityCalibrationDirection();
    constants.orientation_constants.magnetometer_calibration_direction = pose_filter_space->getMagnetometerCalibrationDirection();
    constants.orientation_constants.mean_update_time_delta= ds4_config->mean_update_time_delta;
    constants.orientation_constants.magnetometer_drift = Eigen::Vector3f::Zero(); // no magnetometer on ds4
    constants.orientation_constants.magnetometer_variance= Eigen::Vector3f::Zero(); // no magnetometer on ds4
    constants.orientation_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.orientation_constants.accelerometer_variance =
        Eigen::Vector3f(ds4_config->accelerometer_variance, ds4_config->accelerometer_variance, ds4_config->accelerometer_variance);
    constants.orientation_constants.gyro_drift =
        Eigen::Vector3f(ds4_config->gyro_drift, ds4_config->gyro_drift, ds4_config->gyro_drift);
    constants.orientation_constants.gyro_variance=
        Eigen::Vector3f(ds4_config->gyro_variance, ds4_config->gyro_variance, ds4_config->gyro_variance);
    constants.orientation_constants.position_variance_curve.A = ds4_config->position_variance_exp_fit_a;
    constants.orientation_constants.position_variance_curve.B = ds4_config->position_variance_exp_fit_b;
    constants.orientation_constants.orientation_variance_curve.A = ds4_config->orientation_variance_exp_fit_a;
    constants.orientation_constants.orientation_variance_curve.B = ds4_config->orientation_variance_exp_fit_b;
    constants.orientation_constants.orientation_variance_curve.MaxValue = 1.f;

    constants.position_constants.use_linear_acceleration = ds4_config->position_use_linear_acceleration;
    constants.position_constants.apply_gravity_mask = ds4_config->position_apply_gravity_mask;
    constants.position_constants.gravity_calibration_direction= pose_filter_space->getGravityCalibrationDirection();
    constants.position_constants.accelerometer_drift = Eigen::Vector3f::Zero();
    constants.position_constants.accelerometer_variance= 
        Eigen::Vector3f(ds4_config->accelerometer_variance, ds4_config->accelerometer_variance, ds4_config->accelerometer_variance);
    constants.position_constants.accelerometer_noise_radius= ds4_config->accelerometer_noise_radius;
    constants.position_constants.max_velocity= ds4_config->max_velocity;
    constants.position_constants.mean_update_time_delta= ds4_config->mean_update_time_delta;
    constants.position_constants.position_variance_curve.A = ds4_config->position_variance_exp_fit_a;
    constants.position_constants.position_variance_curve.B = ds4_config->position_variance_exp_fit_b;
    constants.position_constants.position_variance_curve.MaxValue = 1.f;

    *out_pose_filter_space= pose_filter_space;
    *out_pose_filter= pose_filter_factory(
        CommonSensorState::DualShock4,
        ds4_config->position_filter_type,
        ds4_config->orientation_filter_type,
        constants);
}

static void 
post_imu_filter_packets_for_psmove(
	const PSMoveController *psmove, 
	const PSMoveControllerInputState *psmoveState,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_controller_imu_sensor_queue *pose_filter_queue)
{
    const PSMoveControllerConfig *config = psmove->getConfig();

    PoseSensorPacket sensor_packet;

    sensor_packet.clear();

	// One magnetometer update for every two accel/gryo readings
	if (psmove->getSupportsMagnetometer())
	{
		sensor_packet.raw_imu_magnetometer = 
			{psmoveState->RawMag[0], psmoveState->RawMag[1], psmoveState->RawMag[2]};
        sensor_packet.imu_magnetometer_unit =
            Eigen::Vector3f(psmoveState->CalibratedMag[0], psmoveState->CalibratedMag[1], psmoveState->CalibratedMag[2]);
		sensor_packet.has_magnetometer_measurement= true;
	}

	if (psmove->getIsPS4Controller())
	{
		const int frame= 0;

		sensor_packet.timestamp= now;

		sensor_packet.raw_imu_accelerometer = {
			psmoveState->RawAccel[frame][0], 
			psmoveState->RawAccel[frame][1], 
			psmoveState->RawAccel[frame][2]};
		sensor_packet.imu_accelerometer_g_units =
			Eigen::Vector3f(
				psmoveState->CalibratedAccel[frame][0], 
				psmoveState->CalibratedAccel[frame][1], 
				psmoveState->CalibratedAccel[frame][2]);
		sensor_packet.has_accelerometer_measurement= true;

		sensor_packet.raw_imu_gyroscope = {
			psmoveState->RawGyro[frame][0], 
			psmoveState->RawGyro[frame][1], 
			psmoveState->RawGyro[frame][2]};
		sensor_packet.imu_gyroscope_rad_per_sec =
			Eigen::Vector3f(
				psmoveState->CalibratedGyro[frame][0], 
				psmoveState->CalibratedGyro[frame][1], 
				psmoveState->CalibratedGyro[frame][2]);
		sensor_packet.has_gyroscope_measurement= true;

		pose_filter_queue->enqueue(sensor_packet);
	}
	else
	{
		// Don't bother with the earlier frame if this is the very first IMU packet 
		// (since we have no previous timestamp to use)
		int start_frame_index= 0;
		if (duration_since_last_update == t_high_resolution_duration::zero())
		{
			start_frame_index= 1;
		}

		const t_high_resolution_timepoint prev_timestamp= now - (duration_since_last_update / 2);
		t_high_resolution_timepoint timestamps[2] = {prev_timestamp, now};

		// Each state update contains two readings (one earlier and one later) of accelerometer and gyro data
		for (int frame = start_frame_index; frame < 2; ++frame)
		{
			sensor_packet.timestamp= timestamps[frame];

			sensor_packet.raw_imu_accelerometer = {
				psmoveState->RawAccel[frame][0], 
				psmoveState->RawAccel[frame][1], 
				psmoveState->RawAccel[frame][2]};
			sensor_packet.imu_accelerometer_g_units =
				Eigen::Vector3f(
					psmoveState->CalibratedAccel[frame][0], 
					psmoveState->CalibratedAccel[frame][1], 
					psmoveState->CalibratedAccel[frame][2]);
			sensor_packet.has_accelerometer_measurement= true;

			sensor_packet.raw_imu_gyroscope = {
				psmoveState->RawGyro[frame][0], 
				psmoveState->RawGyro[frame][1], 
				psmoveState->RawGyro[frame][2]};
			sensor_packet.imu_gyroscope_rad_per_sec =
				Eigen::Vector3f(
					psmoveState->CalibratedGyro[frame][0], 
					psmoveState->CalibratedGyro[frame][1], 
					psmoveState->CalibratedGyro[frame][2]);
			sensor_packet.has_gyroscope_measurement= true;

			pose_filter_queue->enqueue(sensor_packet);
		}
	}
}

static void
post_optical_filter_packet_for_psmove(
    const PSMoveController *psmove,
	const int tracker_id,
    const t_high_resolution_timepoint now,
    const ControllerOpticalPoseEstimation *pose_estimation,
    t_controller_optical_sensor_queue *pose_sensor_queue)
{
    const PSMoveControllerConfig *config = psmove->getConfig();

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

static void post_imu_filter_packets_for_dualshock4(
	const DualShock4Controller *ds4, 
	const DualShock4ControllerInputState *ds4State,
	const t_high_resolution_timepoint now,
	const t_high_resolution_duration duration_since_last_update,
	t_controller_imu_sensor_queue *pose_filter_queue)
{
    const DualShock4ControllerConfig *config = ds4->getConfig();
    PoseSensorPacket sensor_packet;

    sensor_packet.clear();
	sensor_packet.timestamp= now;

	sensor_packet.raw_imu_accelerometer = {
		ds4State->RawAccelerometer[0], ds4State->RawAccelerometer[1], ds4State->RawAccelerometer[2]};
    sensor_packet.imu_accelerometer_g_units =
		PSVR_vector3f_to_eigen_vector3(ds4State->CalibratedAccelerometer);
	sensor_packet.has_accelerometer_measurement= true;

	sensor_packet.raw_imu_gyroscope = {
		ds4State->RawGyro[0], ds4State->RawGyro[1], ds4State->RawGyro[2]};
    sensor_packet.imu_gyroscope_rad_per_sec =
		PSVR_vector3f_to_eigen_vector3(ds4State->CalibratedGyro);
	sensor_packet.has_gyroscope_measurement= true;

    pose_filter_queue->enqueue(sensor_packet);
}

static void post_optical_filter_packet_for_dualshock4(
	const DualShock4Controller *ds4,
	const int tracker_id,
	const t_high_resolution_timepoint now,
	const ControllerOpticalPoseEstimation *pose_estimation,
	t_controller_optical_sensor_queue *pose_sensor_queue)
{
    const DualShock4ControllerConfig *config = ds4->getConfig();

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
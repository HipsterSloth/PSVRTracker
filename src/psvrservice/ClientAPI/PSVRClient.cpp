//-- includes -----
#include "PSVRClient.h"
#include "Logger.h"
#include "PSVRServiceInterface.h"
#include <algorithm>
#include <iostream>
#include <thread>
#include <memory>
#include <assert.h>

#ifdef _MSC_VER
	#pragma warning(disable:4996)  // ignore strncpy warning
#endif

//-- typedefs -----
typedef std::deque<PSVREventMessage> t_message_queue;

// -- macros -----
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_HMD_COUNT)

// -- prototypes -----
static void applyTrackerDataFrame(const TrackerDataPacket& tracker_packet, PSVRTracker *tracker);
static void applyHmdDataFrame(const HMDDataPacket& hmd_packet, PSVRHeadMountedDisplay *hmd);
static void applyMorpheusDataFrame(const HMDDataPacket& hmd_packet, PSVRMorpheus *morpheus);
static void applyVirtualHMDDataFrame(const HMDDataPacket& hmd_packet, PSVRVirtualHMD *virtualHMD);

// -- methods -----
PSVRClient::PSVRClient(class ServiceRequestHandler *request_handler)
	: m_requestManager(request_handler)
	, m_bHasTrackerListChanged(false)
	, m_bHasHMDListChanged(false)
{
}

PSVRClient::~PSVRClient()
{
}

// -- State Queries ----
bool PSVRClient::pollHasTrackerListChanged()
{ 
	bool bHasTrackerListChanged= m_bHasTrackerListChanged;

	m_bHasTrackerListChanged= false;

	return bHasTrackerListChanged; 
}

bool PSVRClient::pollHasHMDListChanged()
{
	bool bHasHMDListChanged= m_bHasHMDListChanged;

	m_bHasHMDListChanged= false;

	return bHasHMDListChanged; 
}

// -- ClientPSVRAPI System -----
bool PSVRClient::startup(PSVRLogSeverityLevel log_level)
{
    bool success = true;

    log_init(log_level);

	// Reset status flags
	m_bHasTrackerListChanged= false;
	m_bHasHMDListChanged= false;

	PSVR_LOG_INFO("PSVRClient") << "Successfully initialized PSVRClient" << std::endl;

	memset(m_trackers, 0, sizeof(PSVRTracker)*PSVRSERVICE_MAX_TRACKER_COUNT);
	for (PSVRTrackerID tracker_id= 0; tracker_id < PSVRSERVICE_MAX_TRACKER_COUNT; ++tracker_id)    
	{
		m_trackers[tracker_id].tracker_info.tracker_id= tracker_id;
		m_trackers[tracker_id].tracker_info.tracker_type= PSVRTracker_None;
	}

	memset(m_HMDs, 0, sizeof(PSVRHeadMountedDisplay)*PSVRSERVICE_MAX_HMD_COUNT);
	for (PSVRHmdID hmd_id= 0; hmd_id < PSVRSERVICE_MAX_HMD_COUNT; ++hmd_id)    
	{
		m_HMDs[hmd_id].HmdID= hmd_id;
		m_HMDs[hmd_id].HmdType= PSVRHmd_None;
	}

    return success;
}

void PSVRClient::update()
{
    // Drop an unread messages from the previous call to update
    m_message_queue.clear();

    // Publish modified device state back to the service
    publish();
}

void PSVRClient::process_messages()
{
    PSVRMessage message;
    while(poll_next_message(&message, sizeof(message)))
    {
		// Only handle events
		process_event_message(&message.event_data);
    }
}

bool PSVRClient::poll_next_message(PSVRMessage *message, size_t message_size)
{
    bool bHasMessage = false;

    if (m_message_queue.size() > 0)
    {
        const PSVRMessage &first = m_message_queue.front();

        assert(sizeof(PSVRMessage) == message_size);
        assert(message != nullptr);
        memcpy(message, &first, sizeof(PSVRMessage));

        m_message_queue.pop_front();

        // NOTE: We intentionally keep the message parameters around in the 
        // m_response_reference_cache and m_event_reference_cache since the
        // messages contain raw void pointers to the parameters, which
        // become invalid after the next call to update.

        bHasMessage = true;
    }

    return bHasMessage;
}

void PSVRClient::shutdown()
{
    // Drop an unread messages from the previous call to update
    m_message_queue.clear();

    // Drop all of the message parameters
    // NOTE: std::vector::clear() calls the destructor on each element in the vector
    // This will decrement the last ref count to the parameter data, causing them to get cleaned up.
    m_event_reference_cache.clear();
}

// -- ClientPSVRAPI Requests -----
bool PSVRClient::allocate_tracker_listener(const PSVRClientTrackerInfo &trackerInfo)
{
    bool bSuccess= false;

    if (IS_VALID_TRACKER_INDEX(trackerInfo.tracker_id))
    {
        PSVRTracker *tracker= &m_trackers[trackerInfo.tracker_id];

		if (tracker->listener_count == 0)
		{
			memset(tracker, 0, sizeof(PSVRTracker));
            tracker->tracker_info= trackerInfo;
        }

        ++tracker->listener_count;
        bSuccess= true;
    }
    
    return bSuccess;
}

void PSVRClient::free_tracker_listener(PSVRTrackerID tracker_id)
{
	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSVRTracker *tracker= &m_trackers[tracker_id];

		assert(tracker->listener_count > 0);
		--tracker->listener_count;

		if (tracker->listener_count <= 0)
		{
			memset(tracker, 0, sizeof(PSVRTracker));
			tracker->tracker_info.tracker_id= tracker_id;
			tracker->tracker_info.tracker_type= PSVRTracker_None;
		}
	}
}

PSVRTracker* PSVRClient::get_tracker_view(PSVRTrackerID tracker_id)
{
	return IS_VALID_TRACKER_INDEX(tracker_id) ? &m_trackers[tracker_id] : nullptr;
}

bool PSVRClient::open_video_stream(PSVRTrackerID tracker_id)
{
    bool bSuccess = false;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSVRTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_video_frame_buffer == nullptr)
		{
			SharedVideoFrameBuffer *shared_buffer= nullptr;
			if (g_psvr_service->get_shared_video_frame_buffer(tracker_id, &shared_buffer) == PSVRResult_Success)
			{				
				tracker->opaque_shared_video_frame_buffer= shared_buffer;
				bSuccess = true;
			}
		}
		else
		{
			// Already open
			bSuccess = true;
		}

		if (!bSuccess)
		{
			close_video_stream(tracker_id);
		}
	}

    return bSuccess;
}

void PSVRClient::close_video_stream(PSVRTrackerID tracker_id)
{
	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		PSVRTracker *tracker= &m_trackers[tracker_id];

		tracker->opaque_shared_video_frame_buffer = nullptr;
	}
}

const unsigned char *PSVRClient::get_video_frame_buffer(PSVRTrackerID tracker_id) const
{
	const unsigned char *buffer= nullptr;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSVRTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_video_frame_buffer != nullptr)
		{
			SharedVideoFrameBuffer *shared_buffer = 
				reinterpret_cast<SharedVideoFrameBuffer *>(tracker->opaque_shared_video_frame_buffer);

			buffer= shared_buffer->getBuffer();
		}
	}

	return buffer;
}
    
bool PSVRClient::allocate_hmd_listener(PSVRHmdID hmd_id)
{
    bool bSuccess= false;

    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= &m_HMDs[hmd_id];

        if (hmd->ListenerCount == 0)
        {
			memset(hmd, 0, sizeof(PSVRHeadMountedDisplay));
            hmd->HmdID= hmd_id;
            hmd->HmdType = PSVRHmd_None;
        }

        ++hmd->ListenerCount;
        bSuccess= true;
    }
    
    return bSuccess;
}

void PSVRClient::free_hmd_listener(PSVRHmdID hmd_id)
{
    if (IS_VALID_HMD_INDEX(hmd_id))
    {
        PSVRHeadMountedDisplay *hmd= &m_HMDs[hmd_id];

        assert(hmd->ListenerCount > 0);
        --hmd->ListenerCount;

        if (hmd->ListenerCount <= 0)
        {
            memset(hmd, 0, sizeof(PSVRHeadMountedDisplay));
            hmd->HmdID= hmd_id;
            hmd->HmdType= PSVRHmd_None;
        }
    }
}

PSVRHeadMountedDisplay* PSVRClient::get_hmd_view(PSVRHmdID hmd_id)
{
	return IS_VALID_HMD_INDEX(hmd_id) ? &m_HMDs[hmd_id] : nullptr;
}
   
// IDataFrameListener
void PSVRClient::handle_data_frame(const DeviceOutputDataFrame *data_frame)
{
    switch (data_frame->device_category)
    {
    case DeviceCategory_TRACKER:
        {
            const TrackerDataPacket& tracker_packet = data_frame->device.tracker_data_packet;
			const PSVRTrackerID tracker_id= tracker_packet.tracker_id();

            PSVR_LOG_TRACE("handle_data_frame")
                << "received data frame for TrackerID: "
                << tracker_id << std::endl;

			if (IS_VALID_TRACKER_INDEX(tracker_id))
			{
				PSVRTracker *tracker= get_tracker_view(tracker_id);

				applyTrackerDataFrame(tracker_packet, tracker);
			}
        } break;
    case DeviceCategory_HMD:
        {
            const v& hmd_packet = data_frame->device.hmd_data_packet;
			const PSVRHmdID hmd_id= hmd_packet.hmd_id;

            PSVR_LOG_TRACE("handle_data_frame")
                << "received data frame for HmdID: "
                << hmd_packet.hmd_id
                << ". Ignoring." << std::endl;

			if (IS_VALID_HMD_INDEX(hmd_id))
			{
				PSVRHeadMountedDisplay *hmd= get_hmd_view(hmd_id);

				applyHmdDataFrame(hmd_packet, hmd);
			}
        } break;            
    }
}

static void applyTrackerDataFrame(
	const TrackerDataPacket& tracker_packet, 
	PSVRTracker *tracker)
{
	assert(tracker_packet.tracker_id == tracker->tracker_info.tracker_id);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now =
            std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff = now - tracker->data_frame_last_received_time;

        if (diff > 0)
        {
            float seconds = static_cast<float>(diff) / 1000.f;
            float fps = 1.f / seconds;

            tracker->data_frame_average_fps = (0.9f)*tracker->data_frame_average_fps + (0.1f)*fps;
        }

        tracker->data_frame_last_received_time = now;
    }

    if (tracker_packet.sequence_num() > tracker->sequence_num)
    {
        tracker->sequence_num = tracker_packet.sequence_num;
        tracker->is_connected = tracker_packet.is_connected;
    }
}

static void applyHmdDataFrame(
	const PSVRProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet, 
	PSVRHeadMountedDisplay *hmd)
{
	// Ignore old packets
	if (hmd_packet.sequence_num() <= hmd->OutputSequenceNum)
		return;

    // Set the generic items
    hmd->is_valid = hmd_packet.is_valid;
    hmd->hmd_type = hmd_packet.hmd_type;
    hmd->output_sequence_num = hmd_packet.output_sequence_num;
    hmd->is_connected = hmd_packet.is_connected;

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now = 
            std::chrono::duration_cast< std::chrono::milliseconds >(
                std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff= now - hmd->DataFrameLastReceivedTime;

        if (diff > 0)
        {
            float seconds= static_cast<float>(diff) / 1000.f;
            float fps= 1.f / seconds;

            hmd->DataFrameAverageFPS= (0.9f)*hmd->DataFrameAverageFPS + (0.1f)*fps;
        }

        hmd->DataFrameLastReceivedTime= now;
    }

	// Don't bother updating the rest of the hmd state if it's not connected
	if (!hmd->is_connected)
		return;

    switch (hmd->hmd_type) 
	{
        case PSVRHmd_Morpheus:
			applyMorpheusDataFrame(hmd_packet, &hmd->HmdState.MorpheusState);
            break;
        case PSVRHmd_Virtual:
			applyVirtualHMDDataFrame(hmd_packet, &hmd->HmdState.VirtualHMDState);
            break;            
        default:
            break;
    }
}

static void applyMorpheusDataFrame(
	const PSVRProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet,
	PSVRMorpheus *morpheus)
{
    const auto &morpheus_data_frame = hmd_packet.morpheus_state();

	morpheus->bIsTrackingEnabled = morpheus_data_frame.istrackingenabled();
	morpheus->bIsCurrentlyTracking = morpheus_data_frame.iscurrentlytracking();
	morpheus->bIsOrientationValid = morpheus_data_frame.isorientationvalid();
	morpheus->bIsPositionValid = morpheus_data_frame.ispositionvalid();

	morpheus->Pose.Orientation.w = morpheus_data_frame.orientation().w();
	morpheus->Pose.Orientation.x = morpheus_data_frame.orientation().x();
	morpheus->Pose.Orientation.y = morpheus_data_frame.orientation().y();
	morpheus->Pose.Orientation.z = morpheus_data_frame.orientation().z();

	morpheus->Pose.Position.x = morpheus_data_frame.position_cm().x();
	morpheus->Pose.Position.y = morpheus_data_frame.position_cm().y();
	morpheus->Pose.Position.z = morpheus_data_frame.position_cm().z();

	if (morpheus_data_frame.has_raw_sensor_data())
	{
		const auto &raw_sensor_data = morpheus_data_frame.raw_sensor_data();

		morpheus->RawSensorData.Accelerometer.x = raw_sensor_data.accelerometer().i();
		morpheus->RawSensorData.Accelerometer.y = raw_sensor_data.accelerometer().j();
		morpheus->RawSensorData.Accelerometer.z = raw_sensor_data.accelerometer().k();

		morpheus->RawSensorData.Gyroscope.x = raw_sensor_data.gyroscope().i();
		morpheus->RawSensorData.Gyroscope.y = raw_sensor_data.gyroscope().j();
		morpheus->RawSensorData.Gyroscope.z = raw_sensor_data.gyroscope().k();
	}
	else
	{
		memset(&morpheus->RawSensorData, 0, sizeof(PSVRMorpheusRawSensorData));
	}

	if (morpheus_data_frame.has_calibrated_sensor_data())
	{
		const auto &calibrated_sensor_data = morpheus_data_frame.calibrated_sensor_data();

		morpheus->CalibratedSensorData.Accelerometer.x = calibrated_sensor_data.accelerometer().i();
		morpheus->CalibratedSensorData.Accelerometer.y = calibrated_sensor_data.accelerometer().j();
		morpheus->CalibratedSensorData.Accelerometer.z = calibrated_sensor_data.accelerometer().k();

		morpheus->CalibratedSensorData.Gyroscope.x = calibrated_sensor_data.gyroscope().i();
		morpheus->CalibratedSensorData.Gyroscope.y = calibrated_sensor_data.gyroscope().j();
		morpheus->CalibratedSensorData.Gyroscope.z = calibrated_sensor_data.gyroscope().k();
	}
	else
	{
		memset(&morpheus->CalibratedSensorData, 0, sizeof(PSVRMorpheusCalibratedSensorData));
	}

	if (morpheus_data_frame.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = morpheus_data_frame.raw_tracker_data();

		const PSVRProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSVRProtocol::Position &positionOnTrackerCm = raw_tracker_data.relative_position_cm();

		morpheus->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		morpheus->RawTrackerData.ScreenLocation = {locationOnTracker.x(), locationOnTracker.y()};
		morpheus->RawTrackerData.RelativePositionCm = 
			{positionOnTrackerCm.x(), positionOnTrackerCm.y(), positionOnTrackerCm.z()};
        morpheus->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_point_cloud())
		{
			const PSVRProtocol::Polygon &protocolPointCloud = raw_tracker_data.projected_point_cloud();
			PSVRTrackingProjection &projection = morpheus->RawTrackerData.TrackingProjection;

			projection.shape.pointcloud.point_count = std::min(protocolPointCloud.vertices_size(), 7);
			for (int point_index = 0; point_index < projection.shape.pointcloud.point_count; ++point_index)
			{
				const PSVRProtocol::Pixel &point= protocolPointCloud.vertices(point_index);

				projection.shape.pointcloud.points[point_index].x = point.x();
				projection.shape.pointcloud.points[point_index].y = point.y();
			}					
			projection.shape_type = PSVRTrackingProjection::PSVRShape_PointCloud;
		}
		else
		{
			PSVRTrackingProjection &projection = morpheus->RawTrackerData.TrackingProjection;

			projection.shape_type = PSVRTrackingProjection::PSVRShape_INVALID_PROJECTION;
		}
	}
	else
	{
		memset(&morpheus->RawTrackerData, 0, sizeof(PSVRRawTrackerData));
	}

	if (morpheus_data_frame.has_physics_data())
	{
		const auto &raw_physics_data = morpheus_data_frame.physics_data();

		morpheus->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
		morpheus->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
		morpheus->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
		morpheus->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

		morpheus->PhysicsData.AngularVelocityRadPerSec.x = raw_physics_data.angular_velocity_rad_per_sec().i();
		morpheus->PhysicsData.AngularVelocityRadPerSec.y = raw_physics_data.angular_velocity_rad_per_sec().j();
		morpheus->PhysicsData.AngularVelocityRadPerSec.z = raw_physics_data.angular_velocity_rad_per_sec().k();

		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.x = raw_physics_data.angular_acceleration_rad_per_sec_sqr().i();
		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.y = raw_physics_data.angular_acceleration_rad_per_sec_sqr().j();
		morpheus->PhysicsData.AngularAccelerationRadPerSecSqr.z = raw_physics_data.angular_acceleration_rad_per_sec_sqr().k();
	}
	else
	{
		memset(&morpheus->PhysicsData, 0, sizeof(PSVRPhysicsData));
	}
}

static void applyVirtualHMDDataFrame(
	const PSVRProtocol::DeviceOutputDataFrame_HMDDataPacket& hmd_packet,
	PSVRVirtualHMD *virtualHMD)
{
    const auto &virtual_hmd_data_frame = hmd_packet.virtual_hmd_state();

	virtualHMD->bIsTrackingEnabled = virtual_hmd_data_frame.istrackingenabled();
	virtualHMD->bIsCurrentlyTracking = virtual_hmd_data_frame.iscurrentlytracking();
	virtualHMD->bIsPositionValid = virtual_hmd_data_frame.ispositionvalid();

	virtualHMD->Pose.Orientation.w = 1.f;
	virtualHMD->Pose.Orientation.x = 0.f;
	virtualHMD->Pose.Orientation.y = 0.f;
	virtualHMD->Pose.Orientation.z = 0.f;

	virtualHMD->Pose.Position.x = virtual_hmd_data_frame.position_cm().x();
	virtualHMD->Pose.Position.y = virtual_hmd_data_frame.position_cm().y();
	virtualHMD->Pose.Position.z = virtual_hmd_data_frame.position_cm().z();

	if (virtual_hmd_data_frame.has_raw_tracker_data())
	{
		const auto &raw_tracker_data = virtual_hmd_data_frame.raw_tracker_data();

		const PSVRProtocol::Pixel &locationOnTracker = raw_tracker_data.screen_location();
		const PSVRProtocol::Position &positionOnTrackerCm = raw_tracker_data.relative_position_cm();

		virtualHMD->RawTrackerData.TrackerID = raw_tracker_data.tracker_id();
		virtualHMD->RawTrackerData.ScreenLocation = {locationOnTracker.x(), locationOnTracker.y()};
		virtualHMD->RawTrackerData.RelativePositionCm = 
			{positionOnTrackerCm.x(), positionOnTrackerCm.y(), positionOnTrackerCm.z()};
        virtualHMD->RawTrackerData.ValidTrackerBitmask = raw_tracker_data.valid_tracker_bitmask();

		if (raw_tracker_data.has_projected_sphere())
		{
			const PSVRProtocol::Ellipse &protocolEllipse = raw_tracker_data.projected_sphere();
			PSVRTrackingProjection &projection = virtualHMD->RawTrackerData.TrackingProjection;

			projection.shape.ellipse.center.x = protocolEllipse.center().x();
			projection.shape.ellipse.center.y = protocolEllipse.center().y();
			projection.shape.ellipse.half_x_extent = protocolEllipse.half_x_extent();
			projection.shape.ellipse.half_y_extent = protocolEllipse.half_y_extent();
			projection.shape.ellipse.angle = protocolEllipse.angle();
			projection.shape_type = PSVRTrackingProjection::PSVRShape_Ellipse;
		}
		else
		{
			PSVRTrackingProjection &projection = virtualHMD->RawTrackerData.TrackingProjection;

			projection.shape_type = PSVRTrackingProjection::PSVRShape_INVALID_PROJECTION;
		}
	}
	else
	{
		memset(&virtualHMD->RawTrackerData, 0, sizeof(PSVRRawTrackerData));
	}

	if (virtual_hmd_data_frame.has_physics_data())
	{
		const auto &raw_physics_data = virtual_hmd_data_frame.physics_data();

		virtualHMD->PhysicsData.LinearVelocityCmPerSec.x = raw_physics_data.velocity_cm_per_sec().i();
		virtualHMD->PhysicsData.LinearVelocityCmPerSec.y = raw_physics_data.velocity_cm_per_sec().j();
		virtualHMD->PhysicsData.LinearVelocityCmPerSec.z = raw_physics_data.velocity_cm_per_sec().k();

		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.x = raw_physics_data.acceleration_cm_per_sec_sqr().i();
		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.y = raw_physics_data.acceleration_cm_per_sec_sqr().j();
		virtualHMD->PhysicsData.LinearAccelerationCmPerSecSqr.z = raw_physics_data.acceleration_cm_per_sec_sqr().k();

		virtualHMD->PhysicsData.AngularVelocityRadPerSec.x = 0.f;
		virtualHMD->PhysicsData.AngularVelocityRadPerSec.y = 0.f;
		virtualHMD->PhysicsData.AngularVelocityRadPerSec.z = 0.f;

		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.x = 0.f;
		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.y = 0.f;
		virtualHMD->PhysicsData.AngularAccelerationRadPerSecSqr.z = 0.f;
	}
	else
	{
		memset(&virtualHMD->PhysicsData, 0, sizeof(PSVRPhysicsData));
	}
}

// INotificationListener
void PSVRClient::handle_notification(const PSVREventMessage &event)
{
    m_message_queue.push_back(event);
}

// Message Helpers
//-----------------
void PSVRClient::process_event_message(
	const PSVREventMessage *event_message)
{
    switch (event_message->event_type)
    {
    // Service Events
    case PSVREventMessage::PSVREvent_trackerListUpdated:
        m_bHasTrackerListChanged= true;
        break;
    case PSVREventMessage::PSVREvent_hmdListUpdated:
        m_bHasHMDListChanged= true;
        break;
    default:
        assert(0 && "unreachable");
        break;
    }
}

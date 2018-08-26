//-- includes -----
#include "PSVRClient.h"
#include "Logger.h"
#include "PSVRServiceInterface.h"
#include "ServiceRequestHandler.h"
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
#define IS_VALID_CONTROLLER_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_CONTROLLER_COUNT)
#define IS_VALID_TRACKER_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_TRACKER_COUNT)
#define IS_VALID_HMD_INDEX(x) ((x) >= 0 && (x) < PSVRSERVICE_MAX_HMD_COUNT)

// -- prototypes -----
static void applyControllerDataFrame(const ControllerDataPacket& tracker_packet, PSVRController *controller);
static void applyTrackerDataFrame(const TrackerDataPacket& tracker_packet, PSVRTracker *tracker);
static void applyHmdDataFrame(const HMDDataPacket& hmd_packet, PSVRHeadMountedDisplay *hmd);

// -- methods -----
PSVRClient::PSVRClient()
	: m_requestHandler(nullptr)
	, m_bHasControllerListChanged(false)
	, m_bHasTrackerListChanged(false)
	, m_bHasHMDListChanged(false)
{
}

PSVRClient::~PSVRClient()
{
}

// -- State Queries ----
bool PSVRClient::pollHasControllerListChanged()
{ 
	bool bHasControllerListChanged= m_bHasControllerListChanged;

	m_bHasControllerListChanged= false;

	return bHasControllerListChanged; 
}

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
bool PSVRClient::startup(
    PSVRLogSeverityLevel log_level,
    class ServiceRequestHandler * request_handler)
{
    bool success = true;

    m_requestHandler= request_handler;

	// Reset status flags
	m_bHasControllerListChanged= false;
	m_bHasTrackerListChanged= false;
	m_bHasHMDListChanged= false;

	PSVR_LOG_INFO("PSVRClient") << "Successfully initialized PSVRClient";

	memset(m_controllers, 0, sizeof(PSVRTracker)*PSVRSERVICE_MAX_CONTROLLER_COUNT);
	for (PSVRControllerID controller_id= 0; controller_id < PSVRSERVICE_MAX_CONTROLLER_COUNT; ++controller_id)    
	{
		m_controllers[controller_id].ControllerID= controller_id;
		m_controllers[controller_id].ControllerType= PSVRController_None;
	}

	memset(m_trackers, 0, sizeof(PSVRTracker)*PSVRSERVICE_MAX_TRACKER_COUNT);
	for (PSVRTrackerID controller_id= 0; controller_id < PSVRSERVICE_MAX_TRACKER_COUNT; ++controller_id)    
	{
		m_trackers[controller_id].tracker_info.tracker_id= controller_id;
		m_trackers[controller_id].tracker_info.tracker_type= PSVRTracker_None;
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
}

void PSVRClient::process_messages()
{
    PSVREventMessage message;
    while(poll_next_message(&message, sizeof(message)))
    {
		// Only handle events
		process_event_message(&message);
    }
}

bool PSVRClient::poll_next_message(PSVREventMessage *message, size_t message_size)
{
    bool bHasMessage = false;

    if (m_message_queue.size() > 0)
    {
        const PSVREventMessage &first = m_message_queue.front();

        assert(sizeof(PSVREventMessage) == message_size);
        assert(message != nullptr);
        memcpy(message, &first, sizeof(PSVREventMessage));

        m_message_queue.pop_front();

        bHasMessage = true;
    }

    return bHasMessage;
}

void PSVRClient::shutdown()
{
    // Drop an unread messages from the previous call to update
    m_message_queue.clear();
}

// -- ClientPSVRAPI Requests -----
bool PSVRClient::allocate_controller_listener(PSVRControllerID controller_id)
{
    bool bSuccess= false;

    if (IS_VALID_CONTROLLER_INDEX(controller_id))
    {
        PSVRController *controller= &m_controllers[controller_id];

		if (controller->ListenerCount == 0)
		{
			memset(controller, 0, sizeof(PSVRController));
            controller->ControllerID= controller_id;
            controller->ControllerType = PSVRController_None;
        }

        ++controller->ListenerCount;
        bSuccess= true;
    }
    
    return bSuccess;
}

void PSVRClient::free_controller_listener(PSVRControllerID controller_id)
{
	if (IS_VALID_CONTROLLER_INDEX(controller_id))
	{
		PSVRController *controller= &m_controllers[controller_id];

		assert(controller->ListenerCount > 0);
		--controller->ListenerCount;

		if (controller->ListenerCount <= 0)
		{
			memset(controller, 0, sizeof(PSVRController));
			controller->ControllerID= controller_id;
			controller->ControllerType = PSVRController_None;
		}
	}
}

PSVRController* PSVRClient::get_controller_view(PSVRControllerID controller_id)
{
	return IS_VALID_CONTROLLER_INDEX(controller_id) ? &m_controllers[controller_id] : nullptr;
}

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
			const SharedVideoFrameBuffer *shared_buffer= nullptr;
			if (m_requestHandler->get_shared_video_frame_buffer(tracker_id, &shared_buffer) == PSVRResult_Success)
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

int PSVRClient::get_video_frame_section_count(PSVRTrackerID tracker_id) const
{
    int section_count= 0;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSVRTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_video_frame_buffer != nullptr)
		{
			const SharedVideoFrameBuffer *shared_buffer = 
				reinterpret_cast<const SharedVideoFrameBuffer *>(tracker->opaque_shared_video_frame_buffer);

			section_count= shared_buffer->getSectionCount();
		}
	}

    return section_count;
}

const unsigned char *PSVRClient::get_video_frame_buffer(PSVRTrackerID tracker_id, PSVRVideoFrameSection section) const
{
	const unsigned char *buffer= nullptr;

	if (IS_VALID_TRACKER_INDEX(tracker_id))
	{
		const PSVRTracker *tracker= &m_trackers[tracker_id];

		if (tracker->opaque_shared_video_frame_buffer != nullptr)
		{
			const SharedVideoFrameBuffer *shared_buffer = 
				reinterpret_cast<const SharedVideoFrameBuffer *>(tracker->opaque_shared_video_frame_buffer);

			buffer= shared_buffer->getBuffer(section);
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
void PSVRClient::handle_data_frame(const DeviceOutputDataFrame &data_frame)
{
    switch (data_frame.device_category)
    {
    case DeviceCategory_TRACKER:
        {
            const TrackerDataPacket& tracker_packet = data_frame.device.tracker_data_packet;
			const PSVRTrackerID tracker_id= tracker_packet.tracker_id;

            PSVR_LOG_TRACE("handle_data_frame")
                << "received data frame for TrackerID: "
                << tracker_id;

			if (IS_VALID_TRACKER_INDEX(tracker_id))
			{
				PSVRTracker *tracker= get_tracker_view(tracker_id);

				applyTrackerDataFrame(tracker_packet, tracker);
			}
        } break;
    case DeviceCategory_HMD:
        {
            const HMDDataPacket& hmd_packet = data_frame.device.hmd_data_packet;
			const PSVRHmdID hmd_id= hmd_packet.hmd_id;

            PSVR_LOG_TRACE("handle_data_frame")
                << "received data frame for HmdID: "
                << hmd_packet.hmd_id
                << ". Ignoring.";

			if (IS_VALID_HMD_INDEX(hmd_id))
			{
				PSVRHeadMountedDisplay *hmd= get_hmd_view(hmd_id);

				applyHmdDataFrame(hmd_packet, hmd);
			}
        } break;            
    }
}

static void applyControllerDataFrame(
	const ControllerDataPacket& controller_packet,
	PSVRController *controller)
{
	assert(controller_packet.controller_id == controller->ControllerID);

    // Compute the data frame receive window statistics if we have received enough samples
    {
        long long now =
            std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()).count();
        long long diff = now - controller->DataFrameLastReceivedTime;

        if (diff > 0)
        {
            float seconds = static_cast<float>(diff) / 1000.f;
            float fps = 1.f / seconds;

            controller->DataFrameAverageFPS = (0.9f)*controller->DataFrameAverageFPS + (0.1f)*fps;
        }

        controller->DataFrameLastReceivedTime = now;
    }

    if (controller_packet.output_sequence_num > controller->OutputSequenceNum)
    {
        controller->OutputSequenceNum = controller_packet.output_sequence_num;
		controller->IsConnected = controller_packet.is_connected;
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

    if (tracker_packet.sequence_num > tracker->sequence_num)
    {
        tracker->sequence_num = tracker_packet.sequence_num;
        tracker->is_connected = tracker_packet.is_connected;
    }
}

static void applyHmdDataFrame(
	const HMDDataPacket& hmd_packet, 
	PSVRHeadMountedDisplay *hmd)
{
	// Ignore old packets
	if (hmd_packet.output_sequence_num <= hmd->OutputSequenceNum)
		return;

    // Set the generic items
    hmd->bValid = hmd_packet.is_valid;
    hmd->HmdType = hmd_packet.hmd_type;
    hmd->OutputSequenceNum = hmd_packet.output_sequence_num;
    hmd->IsConnected = hmd_packet.is_connected;

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
	if (!hmd->IsConnected)
		return;

    switch (hmd->HmdType) 
	{
        case PSVRHmd_Morpheus:
            hmd->HmdState.MorpheusState= hmd_packet.hmd_state.morpheus_state;
            break;
        case PSVRHmd_Virtual:
            hmd->HmdState.VirtualHMDState= hmd_packet.hmd_state.virtual_hmd_state;
            break;            
        default:
            break;
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
    case PSVREvent_trackerListUpdated:
        m_bHasTrackerListChanged= true;
        break;
    case PSVREvent_hmdListUpdated:
        m_bHasHMDListChanged= true;
        break;
    default:
        assert(0 && "unreachable");
        break;
    }
}

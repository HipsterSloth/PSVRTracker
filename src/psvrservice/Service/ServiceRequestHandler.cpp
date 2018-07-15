//-- includes -----
#include "ServiceRequestHandler.h"

#include "DeviceManager.h"
#include "DeviceEnumerator.h"
#include "MathEigen.h"
#include "HMDManager.h"
#include "Logger.h"
#include "MorpheusHMD.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "PS3EyeTracker.h"
#include "PS4CameraTracker.h"
#include "ServerDeviceView.h"
#include "ServerTrackerView.h"
#include "ServerHMDView.h"
#include "ServiceVersion.h"
#include "TrackerManager.h"
#include "Utility.h"
#include "VirtualHMD.h"

#include <cassert>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

ServiceRequestHandler *ServiceRequestHandler::m_instance= nullptr;

//-- implementation -----
ServiceRequestHandler::ServiceRequestHandler()
    : m_peristentRequestState(new PersistentRequestConnectionState)
    , m_deviceManager(nullptr)
    , m_dataFrameListener(nullptr)
    , m_notificationListener(nullptr)
{
}

ServiceRequestHandler::~ServiceRequestHandler()
{
	delete m_peristentRequestState;
}

bool ServiceRequestHandler::startup(
    class DeviceManager *device_manager,
    class IDataFrameListener *data_frame_listener, 
    class INotificationListener *notification_listener)
{
    m_deviceManager= device_manager;
    m_dataFrameListener= data_frame_listener;
    m_notificationListener= notification_listener;

	m_instance= this;
	return true;
}	
	
void ServiceRequestHandler::shutdown()
{           
	for (int tracker_id = 0; tracker_id < TrackerManager::k_max_devices; ++tracker_id)
	{
		// Restore any overridden camera settings from the config
		if (m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override)
		{
			m_deviceManager->getTrackerViewPtr(tracker_id)->loadSettings();
		}

		// Halt any shared memory streams this connection has going
		if (m_peristentRequestState->active_tracker_stream_info[tracker_id].streaming_video_data)
		{
			m_deviceManager->getTrackerViewPtr(tracker_id)->stopSharedMemoryVideoStream();
		}
	}

	// Clean up any hmd state related to this connection
	for (int hmd_id = 0; hmd_id < HMDManager::k_max_devices; ++hmd_id)
	{
		const HMDStreamInfo &streamInfo = m_peristentRequestState->active_hmd_stream_info[hmd_id];
		ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

		// Undo the ROI suppression
		if (streamInfo.disable_roi)
		{
			m_deviceManager->getHMDViewPtr(hmd_id)->popDisableROI();
		}

		// Halt any hmd tracking this connection had going on
		if (streamInfo.include_position_data)
		{
			m_deviceManager->getHMDViewPtr(hmd_id)->stopTracking();
		}
	}
		
	m_instance= nullptr;
}

void ServiceRequestHandler::publish_tracker_data_frame(
    class ServerTrackerView *tracker_view,
        ServiceRequestHandler::t_generate_tracker_data_frame_for_stream callback)
{
    int tracker_id = tracker_view->getDeviceID();

    // Notify any connections that care about the tracker update
    if (m_peristentRequestState->active_tracker_streams.test(tracker_id))
    {
        const TrackerStreamInfo &streamInfo =
            m_peristentRequestState->active_tracker_stream_info[tracker_id];

        // Fill out a data frame specific to this stream using the given callback
        DeviceOutputDataFrame data_frame;
        callback(tracker_view, &streamInfo, data_frame);

        // Send the tracker data frame over the network
        m_dataFrameListener->handle_data_frame(data_frame);
    }
}

void ServiceRequestHandler::publish_hmd_data_frame(
    class ServerHMDView *hmd_view,
    ServiceRequestHandler::t_generate_hmd_data_frame_for_stream callback)
{
    int hmd_id = hmd_view->getDeviceID();

    // Notify any connections that care about the tracker update
    if (m_peristentRequestState->active_hmd_streams.test(hmd_id))
    {
        const HMDStreamInfo &streamInfo =
            m_peristentRequestState->active_hmd_stream_info[hmd_id];

        // Fill out a data frame specific to this stream using the given callback
        DeviceOutputDataFrame data_frame;
        callback(hmd_view, &streamInfo, data_frame);

        // Send the hmd data frame over the network
        m_dataFrameListener->handle_data_frame(data_frame);
    }
}

void ServiceRequestHandler::publish_notification(const PSVREventMessage &message)
{
    m_notificationListener->handle_notification(message);
}
	
// -- tracker requests -----
PSVRResult ServiceRequestHandler::get_tracker_list(PSVRTrackerList *out_tracker_list)
{
    memset(out_tracker_list, 0, sizeof(PSVRTrackerList));
    for (int tracker_id = 0; tracker_id < m_deviceManager->getTrackerViewMaxCount(); ++tracker_id)
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            PSVRClientTrackerInfo *tracker_info = &out_tracker_list->trackers[out_tracker_list->count++];

            switch (tracker_view->getTrackerDeviceType())
            {
            case CommonSensorState::PS3EYE:
                tracker_info->tracker_type= PSVRTracker_PS3Eye;
                break;
            case CommonSensorState::PS4Camera:
                tracker_info->tracker_type= PSVRTracker_PS4Camera;
                break;					
            case CommonSensorState::WMFMonoCamera:
                tracker_info->tracker_type= PSVRTracker_GenericMonoCamera;
                break;					
            case CommonSensorState::WMFStereoCamera:
                tracker_info->tracker_type= PSVRTracker_GenericStereoCamera;
                break;					
            default:
                assert(0 && "Unhandled tracker type");
            }

            switch (tracker_view->getTrackerDriverType())
            {
            case ITrackerInterface::Libusb:
                tracker_info->tracker_driver= PSVRDriver_LIBUSB;
                break;
            case ITrackerInterface::Generic_Webcam:
                tracker_info->tracker_driver= PSVRDriver_GENERIC_WEBCAM;
                break;
            default:
                assert(0 && "Unhandled tracker type");
            }

            tracker_info->tracker_id= tracker_id;
            strncpy(tracker_info->device_path, tracker_view->getUSBDevicePath().c_str(), sizeof(tracker_info->device_path));

			// Get the constraints for each video property
			for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
			{
				tracker_view->getVideoPropertyConstraint(
					(PSVRVideoPropertyType)prop_index, 
					tracker_info->video_property_constraints[prop_index]);
			}

            // Get the intrinsic camera lens properties
			tracker_view->getCameraIntrinsics(tracker_info->tracker_intrinsics);

            // Get the tracker pose
			tracker_info->tracker_pose= tracker_view->getTrackerPose();
        }
    }

    out_tracker_list->global_forward_degrees= m_deviceManager->m_tracker_manager->getConfig().global_forward_degrees;
        
	return PSVRResult_Success;
}

PSVRResult ServiceRequestHandler::start_tracker_data_stream(
	PSVRTrackerID tracker_id)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            TrackerStreamInfo &streamInfo =
                m_peristentRequestState->active_tracker_stream_info[tracker_id];

            // The tracker manager will always publish updates regardless of who is listening.
            // All we have to do is keep track of which connections care about the updates.
            m_peristentRequestState->active_tracker_streams.set(tracker_id, true);

            // Set control flags for the stream
            streamInfo.streaming_video_data = true;

            // Increment the number of stream listeners
            tracker_view->startSharedMemoryVideoStream();

            // Return the name of the shared memory block the video frames will be written to
            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::stop_tracker_data_stream(
    PSVRTrackerID tracker_id)
{
	PSVRResult result= PSVRResult_Error;
		
    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            m_peristentRequestState->active_tracker_streams.set(tracker_id, false);
            m_peristentRequestState->active_tracker_stream_info[tracker_id].Clear();

            // Restore any overridden camera settings from the config
            if (m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override)
            {
                tracker_view->loadSettings();
            }

            // Decrement the number of stream listeners
            tracker_view->stopSharedMemoryVideoStream();

            result= PSVRResult_Success;
        }
    }
		
	return result;
}
	
PSVRResult ServiceRequestHandler::get_shared_video_frame_buffer(PSVRTrackerID tracker_id, const SharedVideoFrameBuffer **out_shared_buffer)
{
	PSVRResult result= PSVRResult_Error;

	if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
			*out_shared_buffer= tracker_view->getSharedVideoFrameBuffer();
				
			result= PSVRResult_Success;
        }
    }
		
	return result;		
}

PSVRResult ServiceRequestHandler::get_tracker_settings(PSVRTrackerID tracker_id, PSVRHmdID hmd_id, PSVRClientTrackerSettings *out_settings)
{
	PSVRResult result= PSVRResult_Error;

	if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
			ServerHMDView *hmd_view = get_hmd_view_or_null(hmd_id);

            out_settings->frame_width= static_cast<float>(tracker_view->getFrameWidth());
            out_settings->frame_height= static_cast<float>(tracker_view->getFrameHeight());
            out_settings->frame_rate= static_cast<float>(tracker_view->getFrameRate());

			for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
			{
				out_settings->video_properties[prop_index]= tracker_view->getVideoProperty((PSVRVideoPropertyType)prop_index);
			}
				
			tracker_view->gatherTrackingColorPresets(hmd_view, out_settings);
				
			result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_frame_width(
	const PSVRTrackerID tracker_id, 
	const float desired_frame_width, 
	const bool bSaveSetting,
	float *out_result_frame_width)
{
	PSVRResult result= PSVRResult_Error;

	if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            // Set the desired frame width on the tracker
            tracker_view->setFrameWidth(desired_frame_width, bSaveSetting);

            // Only save the setting if requested
            if (bSaveSetting)
            {
                tracker_view->saveSettings();
            }
            else
            {
                m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
            }

            // Return back the actual frame width that got set
            *out_result_frame_width= static_cast<float>(tracker_view->getFrameWidth());

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_frame_height(
	const PSVRTrackerID tracker_id, 
	const float desired_frame_height, 
	const bool bSaveSetting,
	float *out_result_frame_height)
{
	PSVRResult result= PSVRResult_Error;
		
    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            // Set the desired frame height on the tracker
            tracker_view->setFrameHeight(desired_frame_height, bSaveSetting);

            // Only save the setting if requested
            if (bSaveSetting)
            {
                tracker_view->saveSettings();
            }
            else
            {
                m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
            }

            // Return back the actual frame height that got set
            *out_result_frame_height= static_cast<float>(tracker_view->getFrameHeight());

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_frame_rate(
	const PSVRTrackerID tracker_id, 
	const float desired_frame_rate, 
	const bool bSaveSetting,
	float *out_result_frame_rate)
{
	PSVRResult result= PSVRResult_Error;
		
    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            // Set the desired frame rate on the tracker
            tracker_view->setFrameRate(desired_frame_rate, bSaveSetting);

            // Only save the setting if requested
            if (bSaveSetting)
            {
                tracker_view->saveSettings();
            }
            else
            {
                m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
            }

            // Return back the actual frame rate that got set
            *out_result_frame_rate= static_cast<float>(tracker_view->getFrameRate());

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_video_property(
	const PSVRTrackerID tracker_id, const PSVRVideoPropertyType property_type, int desired_value, bool save_setting,
	int *out_value)
{
	PSVRResult result= PSVRResult_Error;
		
    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            // Set the desired property value on the tracker
            tracker_view->setVideoProperty(property_type, desired_value, save_setting);

            // Only save the setting if requested
            if (save_setting)
            {
                tracker_view->saveSettings();
            }
            else
            {
                m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
            }

            // Return back the actual property value that got set
            *out_value= tracker_view->getVideoProperty(property_type);

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_color_preset(
    const PSVRTrackerID tracker_id, 
    const PSVRHmdID hmd_id, 
    const PSVRTrackingColorType tracking_color_type,
    const PSVR_HSVColorRange &desired_color_filter, 
    PSVR_HSVColorRange &out_color_filter)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            ServerHMDView *hmd_view = get_hmd_view_or_null(hmd_id);

            // Assign the color range
            tracker_view->setHMDTrackingColorPreset(hmd_view, tracking_color_type, &desired_color_filter);
            // Read back what actually got set
            tracker_view->getHMDTrackingColorPreset(hmd_view, tracking_color_type, &out_color_filter);

            result= PSVRResult_Success;
        }
    }

    return result;
}

PSVRResult ServiceRequestHandler::set_tracker_pose(
    const PSVRTrackerID tracker_id,
    const PSVRPosef *pose)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            tracker_view->setTrackerPose(pose);
            tracker_view->saveSettings();

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_intrinsics(
    const PSVRTrackerID tracker_id,
    const PSVRTrackerIntrinsics *tracker_intrinsics)
{
	PSVRResult result= PSVRResult_Error;
		
    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            tracker_view->setCameraIntrinsics(*tracker_intrinsics);
            tracker_view->saveSettings();

            result= PSVRResult_Success;
        }
    }
				
	return result;		
}

PSVRResult ServiceRequestHandler::get_tracking_space_settings(
    PSVRTrackingSpace *out_tracking_space)
{
	memset(out_tracking_space, 0, sizeof(PSVRTrackingSpace));
    out_tracking_space->global_forward_degrees= m_deviceManager->m_tracker_manager->getConfig().global_forward_degrees;
		
    return PSVRResult_Success;
}

PSVRResult ServiceRequestHandler::reload_tracker_settings(
    const PSVRTrackerID tracker_id)
{
    PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            tracker_view->loadSettings();
            result= PSVRResult_Success;
        }
    }

    return result;
}

PSVRResult ServiceRequestHandler::get_tracker_debug_flags(PSMTrackerDebugFlags *out_flags) const
{
	*out_flags= TrackerManagerConfig::debug_flags;

	return PSVRResult_Success;
}

PSVRResult ServiceRequestHandler::set_tracker_debug_flags(PSMTrackerDebugFlags flags)
{
	TrackerManagerConfig::debug_flags= flags;

	return PSVRResult_Success;
}

// -- hmd requests -----
ServerHMDView *ServiceRequestHandler::get_hmd_view_or_null(PSVRHmdID hmd_id)
{
    ServerHMDView *hmd_view = nullptr;

    if (Utility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
    {
        ServerHMDViewPtr hmd_view_ptr = m_deviceManager->getHMDViewPtr(hmd_id);

        if (hmd_view_ptr->getIsOpen())
        {
            hmd_view = hmd_view_ptr.get();
        }
    }

    return hmd_view;
}

PSVRResult ServiceRequestHandler::get_hmd_list(
    PSVRHmdList *out_hmd_list)
{
    memset(out_hmd_list, 0, sizeof(PSVRHmdList));
    for (int hmd_id = 0; hmd_id < m_deviceManager->getHMDViewMaxCount(); ++hmd_id)
    {
        ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

        if (hmd_view->getIsOpen() && out_hmd_list->count < PSVRSERVICE_MAX_HMD_COUNT)
        {
            PSVRClientHMDInfo *hmd_info = &out_hmd_list->hmds[out_hmd_list->count++];

            switch (hmd_view->getHMDDeviceType())
            {
            case CommonSensorState::Morpheus:
                {
                    const MorpheusHMD *morpheusHMD= hmd_view->castCheckedConst<MorpheusHMD>();
                    const MorpheusHMDConfig *config= morpheusHMD->getConfig();

                    hmd_info->hmd_type= PSVRHmd_Morpheus;
                    hmd_info->prediction_time= config->prediction_time;
					strncpy(hmd_info->orientation_filter, config->orientation_filter_type.c_str(), sizeof(hmd_info->orientation_filter));
					strncpy(hmd_info->position_filter, config->position_filter_type.c_str(), sizeof(hmd_info->position_filter));
                }
                break;
            case CommonSensorState::VirtualHMD:
                {
                    const VirtualHMD *virtualHMD= hmd_view->castCheckedConst<VirtualHMD>();
                    const VirtualHMDConfig *config= virtualHMD->getConfig();

                    hmd_info->hmd_type= PSVRHmd_Virtual;
                    hmd_info->prediction_time= config->prediction_time;
                    strncpy(hmd_info->orientation_filter, config->orientation_filter_type.c_str(), sizeof(hmd_info->orientation_filter));
                    strncpy(hmd_info->position_filter, config->position_filter_type.c_str(), sizeof(hmd_info->position_filter));
                }
                break;
            default:
                assert(0 && "Unhandled tracker type");
            }

            hmd_info->hmd_id= hmd_id;
            hmd_info->tracking_color_type= hmd_view->getTrackingColorID();
            strncpy(hmd_info->device_path, hmd_view->getUSBDevicePath().c_str(), sizeof(hmd_info->device_path));
        }
    }

    return PSVRResult_Success;
}
	
PSVRResult ServiceRequestHandler::get_hmd_tracking_shape(PSVRHmdID hmd_id, PSVRTrackingShape *out_shape)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
    {
        ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);
			
        if (hmd_view->getIsOpen())
        {
			hmd_view->getTrackingShape(*out_shape);
		}				
	}			
		
	return result;
}

PSVRResult ServiceRequestHandler::start_hmd_data_stream(
    const PSVRHmdID hmd_id,
    unsigned int data_stream_flags)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
    {
        ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

        if (hmd_view->getIsOpen())
        {
            HMDStreamInfo &streamInfo = m_peristentRequestState->active_hmd_stream_info[hmd_id];

            // The hmd manager will always publish updates regardless of who is listening.
            // All we have to do is keep track of which connections care about the updates.
            m_peristentRequestState->active_hmd_streams.set(hmd_id, true);

            // Set control flags for the stream
            streamInfo.Clear();
            streamInfo.include_position_data = (data_stream_flags & PSMStreamFlags_includePositionData) > 0;
            streamInfo.include_physics_data = (data_stream_flags & PSMStreamFlags_includePhysicsData) > 0;
            streamInfo.include_raw_sensor_data = (data_stream_flags & PSMStreamFlags_includeRawSensorData) > 0;
            streamInfo.include_calibrated_sensor_data = (data_stream_flags & PSMStreamFlags_includeCalibratedSensorData) > 0;
            streamInfo.include_raw_tracker_data = (data_stream_flags & PSMStreamFlags_includeRawTrackerData) > 0;
            streamInfo.disable_roi = (data_stream_flags & PSMStreamFlags_disableROI) > 0;

            PSVR_LOG_INFO("ServerRequestHandler") << "Start hmd(" << hmd_id << ") stream ("
                << "pos=" << streamInfo.include_position_data
                << ",phys=" << streamInfo.include_physics_data
                << ",raw_sens=" << streamInfo.include_raw_sensor_data
                << ",cal_sens=" << streamInfo.include_calibrated_sensor_data
                << ",trkr=" << streamInfo.include_raw_tracker_data
                << ",roi=" << streamInfo.disable_roi
                << ")";

            if (streamInfo.disable_roi)
            {
                ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

                hmd_view->pushDisableROI();
            }

            if (streamInfo.include_position_data)
            {
                ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

                hmd_view->startTracking();
            }

            // Return the name of the shared memory block the video frames will be written to
			result= PSVRResult_Success;
		}
    }

	return result;
}

PSVRResult ServiceRequestHandler::stop_hmd_data_stream(
    const PSVRHmdID hmd_id)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
    {
        ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

        if (hmd_view->getIsOpen())
        {
            const HMDStreamInfo &streamInfo = m_peristentRequestState->active_hmd_stream_info[hmd_id];

            if (streamInfo.disable_roi)
            {
                hmd_view->popDisableROI();
            }

            if (streamInfo.include_position_data)
            {
                hmd_view->stopTracking();
            }

            m_peristentRequestState->active_hmd_streams.set(hmd_id, false);
            m_peristentRequestState->active_hmd_stream_info[hmd_id].Clear();

            result= PSVRResult_Success;
        }
    }

	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_led_tracking_color(
    const PSVRHmdID hmd_id,
	const PSVRTrackingColorType new_color_id)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HmdView && 
        HmdView->getHMDDeviceType() == CommonSensorState::VirtualHMD)
    {
        const PSVRTrackingColorType oldColorID = HmdView->getTrackingColorID();

        if (new_color_id != oldColorID)
        {
            // Give up control of our existing tracking color
            if (oldColorID != PSVRTrackingColorType_INVALID)
            {
                m_deviceManager->m_tracker_manager->freeTrackingColorID(oldColorID);
            }

            // Take the color from any other controller that might have it
            if (m_deviceManager->m_tracker_manager->claimTrackingColorID(HmdView.get(), new_color_id))
            {
                // Assign the new color to ourselves
                HmdView->setTrackingColorID(new_color_id);
                result= PSVRResult_Success;
            }
            else
            {
                if (oldColorID != PSVRTrackingColorType_INVALID)
                {
                    m_deviceManager->m_tracker_manager->claimTrackingColorID(HmdView.get(), oldColorID);
                }
            }
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_accelerometer_calibration(
    const PSVRHmdID hmd_id,
	const PSVRVector3f &measured_g,
	const float raw_variance)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HMDView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HMDView && HMDView->getHMDDeviceType() == CommonSensorState::Morpheus)
    {
        MorpheusHMD *hmd = HMDView->castChecked<MorpheusHMD>();
        IPoseFilter *poseFilter = HMDView->getPoseFilterMutable();
        MorpheusHMDConfig *config = hmd->getConfigMutable();

        // Compute the bias as 1g subtracted from the measured direction of gravity
        float length = sqrtf(measured_g.x*measured_g.x + measured_g.y*measured_g.y + measured_g.z*measured_g.z);
        if (length > k_real_epsilon)
        {
            config->raw_accelerometer_bias.x = measured_g.x * (1.f - 1.f / (length*config->accelerometer_gain.x));
            config->raw_accelerometer_bias.y = measured_g.y * (1.f - 1.f / (length*config->accelerometer_gain.y));
            config->raw_accelerometer_bias.z = measured_g.z * (1.f - 1.f / (length*config->accelerometer_gain.z));
        }

        config->raw_accelerometer_variance = raw_variance;
        config->save();

        // Reset the orientation filter state the calibration changed
        poseFilter->resetState();

        result= PSVRResult_Success;
    }

	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_gyroscope_calibration(
    const PSVRHmdID hmd_id,
	const PSVRVector3f &raw_gyro_bias,
	const float raw_variance,
	const float raw_drift)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HMDView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HMDView && HMDView->getHMDDeviceType() == CommonSensorState::Morpheus)
    {
        MorpheusHMD *hmd = HMDView->castChecked<MorpheusHMD>();
        MorpheusHMDConfig *config = hmd->getConfigMutable();

		config->raw_gyro_bias= raw_gyro_bias;
        config->raw_gyro_variance = raw_variance;
        config->raw_gyro_drift = raw_drift;
        config->save();

        // Reset the orientation filter state the calibration changed
        HMDView->getPoseFilterMutable()->resetState();

        result= PSVRResult_Success;
    }

	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_orientation_filter(
    const PSVRHmdID hmd_id,
	const std::string orientation_filter)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HmdView && HmdView->getIsOpen())
    {
        if (HmdView->getHMDDeviceType() == CommonSensorState::Morpheus)
        {
            MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
            MorpheusHMDConfig *config = hmd->getConfigMutable();

            if (config->orientation_filter_type != orientation_filter)
            {
                config->orientation_filter_type = orientation_filter;
                config->save();

                HmdView->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
        else if (HmdView->getHMDDeviceType() == CommonSensorState::VirtualHMD)
        {
            VirtualHMD *hmd = HmdView->castChecked<VirtualHMD>();
            VirtualHMDConfig *config = hmd->getConfigMutable();

            if (config->orientation_filter_type != orientation_filter)
            {
                config->orientation_filter_type = orientation_filter;
                config->save();

                HmdView->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_position_filter(
    const PSVRHmdID hmd_id,
	const std::string position_filter)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HmdView && HmdView->getIsOpen())
    {
        if (HmdView->getHMDDeviceType() == CommonSensorState::Morpheus)
        {
            MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
            MorpheusHMDConfig *config = hmd->getConfigMutable();

            if (config->position_filter_type != position_filter)
            {
                config->position_filter_type = position_filter;
                config->save();

                HmdView->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
        else if (HmdView->getHMDDeviceType() == CommonSensorState::VirtualHMD)
        {
            VirtualHMD *hmd = HmdView->castChecked<VirtualHMD>();
            VirtualHMDConfig *config = hmd->getConfigMutable();

            if (config->position_filter_type != position_filter)
            {
                config->position_filter_type = position_filter;
                config->save();

                HmdView->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_prediction_time(
    const PSVRHmdID hmd_id,
	const float hmd_prediction_time)
{
	PSVRResult result= PSVRResult_Error;

    ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

    if (HmdView && HmdView->getIsOpen())
    {
        if (HmdView->getHMDDeviceType() == CommonSensorState::Morpheus)
        {
            MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
            MorpheusHMDConfig *config = hmd->getConfigMutable();

            if (config->prediction_time != hmd_prediction_time)
            {
                config->prediction_time = hmd_prediction_time;
                config->save();
            }

            result= PSVRResult_Success;
        }
        else if (HmdView->getHMDDeviceType() == CommonSensorState::VirtualHMD)
        {
            VirtualHMD *hmd = HmdView->castChecked<VirtualHMD>();
            VirtualHMDConfig *config = hmd->getConfigMutable();

            if (config->prediction_time != hmd_prediction_time)
            {
                config->prediction_time = hmd_prediction_time;
                config->save();
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_hmd_data_stream_tracker_index(
	const PSVRTrackerID tracker_id,
    const PSVRHmdID hmd_id)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()) &&
        Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);
        HMDStreamInfo &streamInfo = m_peristentRequestState->active_hmd_stream_info[hmd_id];

        PSVR_LOG_INFO("ServerRequestHandler") << "Set hmd(" << hmd_id << ") stream tracker id: " << tracker_id;

        streamInfo.selected_tracker_index= tracker_id;

        result= PSVRResult_Success;
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::get_service_version(
    char *out_version_string, 
	size_t max_version_string)
{
    // Return the protocol version
    strncpy(out_version_string, PSVR_SERVICE_VERSION_STRING, max_version_string);
		
	return PSVRResult_Success;
}

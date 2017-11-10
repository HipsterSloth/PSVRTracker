//-- includes -----
#include "ServerRequestHandler.h"

#include "DeviceManager.h"
#include "DeviceEnumerator.h"
#include "MathEigen.h"
#include "HMDManager.h"
#include "Logger.h"
#include "MorpheusHMD.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "PS3EyeTracker.h"
#include "ServerDeviceView.h"
#include "ServerTrackerView.h"
#include "ServerHMDView.h"
#include "TrackerManager.h"
#include "Utility.h"
#include "VirtualController.h"
#include "VirtualHMD.h"

#include <cassert>
#include <bitset>

//-- definitions -----
struct PersistentRequestConnectionState
{
    std::bitset<TrackerManager::k_max_devices> active_tracker_streams;
    std::bitset<HMDManager::k_max_devices> active_hmd_streams;
    TrackerStreamInfo active_tracker_stream_info[TrackerManager::k_max_devices];
    HMDStreamInfo active_hmd_stream_info[HMDManager::k_max_devices];

    PersistentRequestConnectionState()
        : active_tracker_streams()
        , active_hmd_streams()
    {
        for (int index = 0; index < TrackerManager::k_max_devices; ++index)
        {
            active_tracker_stream_info[index].Clear();
        }
        
        for (int index = 0; index < HMDManager::k_max_devices; ++index)
        {
            active_hmd_stream_info[index].Clear();
        }
    }
};

//-- implementation -----
class ServerRequestHandler
{
public:
    ServerRequestHandler(
		class DeviceManager *device_manager,
		class IDataFrameListener *data_frame_listener)
        : m_deviceManager(device_manager)
		, m_dataFrameListener(data_frame_listener)
        , m_peristentRequestState(new PersistentRequestConnectionState)
    {
    }

    virtual ~ServerRequestHandler()
    {
		delete m_peristentRequestState;
    }

	bool startup()
	{
		m_instance= this;
		return true;
	}	
	
    void shutdown()
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

    void publish_tracker_data_frame(
        class ServerTrackerView *tracker_view,
            ServerRequestHandler::t_generate_tracker_data_frame_for_stream callback)
    {
        int tracker_id = tracker_view->getDeviceID();

        // Notify any connections that care about the tracker update
        for (t_connection_state_iter iter = m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id = iter->first;
            RequestConnectionStatePtr connection_state = iter->second;

            if (connection_state->active_tracker_streams.test(tracker_id))
            {
                const TrackerStreamInfo &streamInfo =
                    connection_state->active_tracker_stream_info[tracker_id];

                // Fill out a data frame specific to this stream using the given callback
                DeviceOutputDataFrame data_frame;
                callback(tracker_view, &streamInfo, &data_frame);				

                // Send the tracker data frame over the network
                m_dataFrameListener->handle_data_frame(data_frame);
            }
        }
    }

    void publish_hmd_data_frame(
        class ServerHMDView *hmd_view,
        ServerRequestHandler::t_generate_hmd_data_frame_for_stream callback)
    {
        int hmd_id = hmd_view->getDeviceID();

        // Notify any connections that care about the tracker update
        for (t_connection_state_iter iter = m_connection_state_map.begin(); iter != m_connection_state_map.end(); ++iter)
        {
            int connection_id = iter->first;
            RequestConnectionStatePtr connection_state = iter->second;

            if (connection_state->active_hmd_streams.test(hmd_id))
            {
                const HMDStreamInfo &streamInfo =
                    connection_state->active_hmd_stream_info[hmd_id];

                // Fill out a data frame specific to this stream using the given callback
                DeviceOutputDataFrame data_frame;
                callback(hmd_view, &streamInfo, &data_frame);	

                // Send the hmd data frame over the network
                m_dataFrameListener->handle_data_frame(data_frame);
            }
        }
    }    
	
    // -- tracker requests -----
    PSMResult get_tracker_list(PSMTrackerList *out_tracker_list)
    {
        for (int tracker_id = 0; tracker_id < m_deviceManager->getTrackerViewMaxCount(); ++tracker_id)
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);

            if (tracker_view->getIsOpen())
            {
                PSMClientTrackerInfo *tracker_info = out_tracker_list->trackers[++out_tracker_list->count];

                switch (tracker_view->getTrackerDeviceType())
                {
                case CommonControllerState::PS3EYE:
                    tracker_info->tracker_type= PSMTracker_PS3Eye;
                    break;
                case CommonControllerState::PS4CAMERA:
                    tracker_info->tracker_type= PSMTracker_PS4Camera;
                    break;					
                default:
                    assert(0 && "Unhandled tracker type");
                }

                switch (tracker_view->getTrackerDriverType())
                {
                case ITrackerInterface::Libusb:
                    tracker_info->tracker_driver= PSMDriver_LIBUSB;
                    break;
                case ITrackerInterface::Generic_Webcam:
                    tracker_info->tracker_driver= PSMDriver_GENERIC_WEBCAM;
                    break;
                default:
                    assert(0 && "Unhandled tracker type");
                }

                tracker_info->tracker_id= tracker_id;
                strncpy(tracker_info->device_path, tracker_view->getUSBDevicePath().c_str(), sizeof(tracker_info->device_path));

                // Get the intrinsic camera lens properties
				tracker_view->getCameraIntrinsics(&tracker_info->tracker_intrinsics);

                // Get the tracker pose
				tracker_info->tracker_pose= tracker_view->getTrackerPose();
            }
        }

        list->set_global_forward_degrees(m_deviceManager->m_tracker_manager->getConfig().global_forward_degrees);
        
		return PSMResult_Success;
    }

    PSMResult start_tracker_data_stream(
		PSMTrackerID tracker_id)
    {
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
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
                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult stop_tracker_data_stream(
        PSMTrackerID tracker_id)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
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

                result= PSMResult_Success;
            }
        }
		
		return result;
    }
	
	PSMResult get_shared_video_frame_buffer(PSMTrackerID tracker_id, SharedVideoFrameBuffer **out_shared_buffer)
	{
		PSMResult result= PSMResult_Error;

		if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
				*out_shared_buffer= tracker_view->getSharedVideoFrameBuffer();
				
				result= PSMResult_Success;
            }
        }
		
		return result;		
	}

    PSMResult get_tracker_settings(PSMTrackerID tracker_id, PSMHmdID hmd_id, PSMClientTrackerSettings *out_settings)
    {
		PSMResult result= PSMResult_Error;

		if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
				ServerHMDView *hmd_view = get_hmd_view_or_null(hmd_id);

                settings->frame_width= static_cast<float>(tracker_view->getFrameWidth());
                settings->frame_height= static_cast<float>(tracker_view->getFrameHeight());
                settings->frame_rate= static_cast<float>(tracker_view->getFrameRate());
                settings->exposure= static_cast<float>(tracker_view->getExposure());
                settings->gain= static_cast<float>(tracker_view->getGain());
				
                tracker_view->gatherTrackerOptions(settings);
				tracker_view->gatherTrackingColorPresets(hmd_view, settings);
				
				result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_frame_width(
		const PSMTrackerID tracker_id, 
		const float desired_frame_width, 
		const bool bSaveSetting,
		float *out_result_frame_width)
    {
		PSMResult result= PSMResult_Error;

		if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
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

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_frame_height(
		const PSMTrackerID tracker_id, 
		const float desired_frame_height, 
		const bool bSaveSetting,
		float *out_result_frame_height)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
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

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_frame_rate(
		const PSMTrackerID tracker_id, 
		const float desired_frame_rate, 
		const bool bSaveSetting,
		float *out_result_frame_rate)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
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

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_exposure(
		const PSMTrackerID tracker_id, 
		const float desired_exposure, 
		const bool bSaveSetting,
		float *out_result_exposure)
    {
		PSMResult result= PSMResult_Error;

		if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
                // Set the desired exposure on the tracker
                tracker_view->setExposure(desired_exposure, bSaveSetting);

                // Only save the setting if requested
                if (bSaveSetting)
                {
                    tracker_view->saveSettings();
                }
                else
                {
                    m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
                }

                // Return back the actual exposure that got set
                *out_result_exposure= static_cast<float>(tracker_view->getExposure());

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_gain(
		const PSMTrackerID tracker_id, 
		const float desired_gain, 
		const bool bSaveSetting,
		float *out_result_gain)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
                // Set the desired gain on the tracker
                tracker_view->setGain(desired_gain, bSaveSetting);

                // Only save the setting if requested
                if (bSaveSetting)
                {
                    tracker_view->saveSettings();
                }
                else
                {
                    m_peristentRequestState->active_tracker_stream_info[tracker_id].has_temp_settings_override = true;
                }

                // Return back the actual gain that got set
                *out_result_gain= static_cast<float>(tracker_view->getGain());

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_option(
		const PSMTrackerID tracker_id,
        const std::string &option_name,
		const int desired_option_index,
		int *out_new_option_index)
    {
        PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
                if (tracker_view->setOptionIndex(option_name, desired_option_index))
                {
                    // Return back the actual option index that got set
                    tracker_view->getOptionIndex(option_name, *out_new_option_index);
                    tracker_view->saveSettings();

                    result= PSMResult_Success;
                }
            }
		}
			
		return result;
    }

    PSMResult set_tracker_color_preset(
        const PSMTrackerID tracker_id,
		const PSMHmdID hmd_id,
		const PSMTrackingColorType color_type,
		const PSMHSVColorRange *desired_range,
        PSMHSVColorRange *out_result_range)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
				ServerHMDView *hmd_view = get_hmd_view_or_null(device_id);

				// Assign the color range
				tracker_view->setHMDTrackingColorPreset(hmd_view, color_type, desired_range);
				// Read back what actually got set
				tracker_view->getHMDTrackingColorPreset(hmd_view, color_type, out_result_range);
				
                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_pose(
        const PSMTrackerID tracker_id,
        const PSMPosef *pose)
    {
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
                tracker_view->setTrackerPose(pose);
                tracker_view->saveSettings();

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_tracker_intrinsics(
        const PSMTrackerID tracker_id,
        const PSMTrackerIntrinsics *tracker_intrinsics)
    {
		PSMResult result= PSMResult_Error;
		
        if (ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
            if (tracker_view->getIsOpen())
            {
                tracker_view->setCameraIntrinsics(tracker_intrinsics);
                tracker_view->saveSettings();

                result= PSMResult_Success;
            }
        }
				
		return result;		
    }

    PSMResult get_tracking_space_settings(
        PSMTrackingSpace *out_tracking_space)
    {
		memset(out_tracking_space, 0, sizeof(PSMTrackingSpace));
        out_tracking_space->global_forward_degrees= m_deviceManager->m_tracker_manager->getConfig().global_forward_degrees;
		
        return PSMResult_Success;
    }
    // -- hmd requests -----
    inline ServerHMDView *get_hmd_view_or_null(PSMHmdID hmd_id)
    {
        ServerHMDView *hmd_view = nullptr;

        if (ServerUtility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
        {
            ServerHMDViewPtr hmd_view_ptr = m_deviceManager->getHMDViewPtr(hmd_id);

            if (hmd_view_ptr->getIsOpen())
            {
                hmd_view = hmd_view_ptr.get();
            }
        }

        return hmd_view;
    }

    PSMResult get_hmd_list(
        PSMHmdList *out_hmd_list)
    {
        for (int hmd_id = 0; hmd_id < m_deviceManager->getHMDViewMaxCount(); ++hmd_id)
        {
            ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);

            if (hmd_view->getIsOpen() && out_hmd_list->count < PSVRSERVICE_MAX_HMD_COUNT)
            {
                PSMHmdList *hmd_info = &out_hmd_list->hmds[hmd_id];

                switch (hmd_view->getHMDDeviceType())
                {
                case CommonHMDState::Morpheus:
                    {
                        const MorpheusHMD *morpheusHMD= hmd_view->castCheckedConst<MorpheusHMD>();
                        const MorpheusHMDConfig *config= morpheusHMD->getConfig();

                        hmd_info->hmd_type= PSMoveProtocol::Morpheus;
                        hmd_info->prediction_time= config->prediction_time;
						strncpy(hmd_info->orientation_filter, config->orientation_filter, sizeof(hmd_info->orientation_filter));
						strncpy(hmd_info->position_filter, config->position_filter_type, sizeof(hmd_info->position_filter));
                    }
                    break;
                case CommonHMDState::VirtualHMD:
                    {
                        const VirtualHMD *virtualHMD= hmd_view->castCheckedConst<VirtualHMD>();
                        const VirtualHMDConfig *config= virtualHMD->getConfig();

                        hmd_info->hmd_type= PSMoveProtocol::VirtualHMD;
                        hmd_info->prediction_time= config->prediction_time;
                        strncpy(hmd_info->position_filter, config->position_filter_type, sizeof(hmd_info->position_filter));
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

        return PSMResult_Success;
    }
	
	PSMResult get_hmd_tracking_shape(PSMHmdID hmd_id, PSMTrackingShape *out_shape)
	{
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
        {
            ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);
			
            if (hmd_view->getIsOpen())
            {
				hmd_view->getTrackingShape(out_shape);
			}				
		}			
		
		return result;
	}

    PSMResult start_hmd_data_stream(
        const PSMHmdID hmd_id)
    {
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
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
                streamInfo.include_position_data = request.include_position_data();
                streamInfo.include_physics_data = request.include_physics_data();
                streamInfo.include_raw_sensor_data = request.include_raw_sensor_data();
                streamInfo.include_calibrated_sensor_data = request.include_calibrated_sensor_data();
                streamInfo.include_raw_tracker_data = request.include_raw_tracker_data();
                streamInfo.disable_roi = request.disable_roi();

                SERVER_LOG_INFO("ServerRequestHandler") << "Start hmd(" << hmd_id << ") stream ("
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
				result= PSMResult_Success;
			}
        }

		return result;
    }

    PSMResult stop_hmd_data_stream(
        const PSMHmdID hmd_id)
    {
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()))
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

                result= PSMResult_Success;
            }
        }

		return result;
    }

    PSMResult set_hmd_led_tracking_color(
        const PSMHmdID hmd_id,
		const PSMTrackingColorType new_color_id)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

        if (HmdView && 
            HmdView->getHMDDeviceType() == CommonDeviceState::VirtualHMD)
        {
            const eCommonTrackingColorID oldColorID = HmdView->getTrackingColorID();

            if (newColorID != oldColorID)
            {
                // Give up control of our existing tracking color
                if (oldColorID != eCommonTrackingColorID::INVALID_COLOR)
                {
                    m_deviceManager->m_tracker_manager->freeTrackingColorID(oldColorID);
                }

                // Take the color from any other controller that might have it
                if (m_deviceManager->m_tracker_manager->claimTrackingColorID(HmdView.get(), newColorID))
                {
                    // Assign the new color to ourselves
                    HmdView->setTrackingColorID(newColorID);
                    result= PSMResult_Success;
                }
                else
                {
                    if (oldColorID != eCommonTrackingColorID::INVALID_COLOR)
                    {
                        m_deviceManager->m_tracker_manager->claimTrackingColorID(HmdView.get(), oldColorID);
                    }
                }
            }
        }
		
		return result;
    }

    PSMResult set_hmd_accelerometer_calibration(
        const PSMHmdID hmd_id,
		const PSMVector3f &measured_g,
		const float raw_variance)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HMDView = m_deviceManager->getHMDViewPtr(hmd_id);

        if (HMDView && HMDView->getHMDDeviceType() == CommonDeviceState::Morpheus)
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

            result= PSMResult_Success;
        }

		return result;
    }

    PSMResult set_hmd_gyroscope_calibration(
        const PSMHmdID hmd_id,
		const PSMVector3f &raw_gyro_bias,
		const float raw_variance,
		const float raw_drift)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HMDView = m_deviceManager->getHMDViewPtr(hmd_id);

        if (HMDView && HMDView->getHMDDeviceType() == CommonDeviceState::Morpheus)
        {
            MorpheusHMD *hmd = HMDView->castChecked<MorpheusHMD>();
            MorpheusHMDConfig *config = hmd->getConfigMutable();

			config->raw_gyro_bias= raw_gyro_bias;
            config->raw_gyro_variance = raw_variance;
            config->raw_gyro_drift = raw_drift;
            config->save();

            // Reset the orientation filter state the calibration changed
            HMDView->getPoseFilterMutable()->resetState();

            result= PSMResult_Success;
        }

		return result;
    }

    PSMResult set_hmd_orientation_filter(
        const PSMHmdID hmd_id,
		const std::string orientation_filter)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

        if (HmdView && HmdView->getIsOpen())
        {
            if (HmdView->getHMDDeviceType() == CommonDeviceState::Morpheus)
            {
                MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
                MorpheusHMDConfig *config = hmd->getConfigMutable();

                if (config->orientation_filter_type != orientation_filter)
                {
                    config->orientation_filter_type = orientation_filter;
                    config->save();

                    HmdView->resetPoseFilter();
                }

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_hmd_position_filter(
        const PSMHmdID hmd_id,
		const std::string position_filter)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);

        if (HmdView && HmdView->getIsOpen())
        {
            if (HmdView->getHMDDeviceType() == CommonDeviceState::Morpheus)
            {
                MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
                MorpheusHMDConfig *config = hmd->getConfigMutable();

                if (config->position_filter_type != position_filter)
                {
                    config->position_filter_type = position_filter;
                    config->save();

                    HmdView->resetPoseFilter();
                }

                result= PSMResult_Success;
            }
            else if (HmdView->getHMDDeviceType() == CommonDeviceState::VirtualHMD)
            {
                VirtualHMD *hmd = HmdView->castChecked<VirtualHMD>();
                VirtualHMDConfig *config = hmd->getConfigMutable();

                if (config->position_filter_type != position_filter)
                {
                    config->position_filter_type = position_filter;
                    config->save();

                    HmdView->resetPoseFilter();
                }

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_hmd_prediction_time(
        const PSMHmdID hmd_id,
		const float hmd_prediction_time)
    {
		PSMResult result= PSMResult_Error;

        ServerHMDViewPtr HmdView = m_deviceManager->getHMDViewPtr(hmd_id);
        const PSMoveProtocol::Request_RequestSetHMDPredictionTime &request =
            context.request->request_set_hmd_prediction_time();

        if (HmdView && HmdView->getIsOpen())
        {
            if (HmdView->getHMDDeviceType() == CommonDeviceState::Morpheus)
            {
                MorpheusHMD *hmd = HmdView->castChecked<MorpheusHMD>();
                MorpheusHMDConfig *config = hmd->getConfigMutable();

                if (config->prediction_time != hmd_prediction_time)
                {
                    config->prediction_time = hmd_prediction_time;
                    config->save();
                }

                result= PSMResult_Success;
            }
            else if (HmdView->getHMDDeviceType() == CommonDeviceState::VirtualHMD)
            {
                VirtualHMD *hmd = HmdView->castChecked<VirtualHMD>();
                VirtualHMDConfig *config = hmd->getConfigMutable();

                if (config->prediction_time != hmd_prediction_time)
                {
                    config->prediction_time = hmd_prediction_time;
                    config->save();
                }

                result= PSMResult_Success;
            }
        }
		
		return result;
    }

    PSMResult set_hmd_data_stream_tracker_index(
		const PSMTrackerID tracker_id,
        const PSMHmdID hmd_id)
    {
		PSMResult result= PSMResult_Error;

        if (ServerUtility::is_index_valid(hmd_id, m_deviceManager->getHMDViewMaxCount()) &&
            ServerUtility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
        {
            ServerHMDViewPtr hmd_view = m_deviceManager->getHMDViewPtr(hmd_id);
            HMDStreamInfo &streamInfo = m_peristentRequestState->active_hmd_stream_info[hmd_id];

            SERVER_LOG_INFO("ServerRequestHandler") << "Set hmd(" << hmd_id << ") stream tracker id: " << tracker_id;

            streamInfo.selected_tracker_index= tracker_id;

            result= PSMResult_Success;
        }
		
		return result;
    }

    PSMResult get_service_version(
        char *out_version_string, 
		size_t max_version_string)
    {
        // Return the protocol version
        strncpy(out_version_string, PSM_PROTOCOL_VERSION_STRING, max_version_string);
		
		return PSMResult_Success;
    }

private:
    class DeviceManager *m_deviceManager;
    class PersistentRequestConnectionState *m_peristentRequestState;
};

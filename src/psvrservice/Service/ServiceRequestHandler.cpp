//-- includes -----
#include "ServiceRequestHandler.h"

#include "DeviceManager.h"
#include "ControllerManager.h"
#include "DeviceEnumerator.h"
#include "DualShock4Controller.h"
#include "MathEigen.h"
#include "HMDManager.h"
#include "Logger.h"
#include "MorpheusHMD.h"
#include "OrientationFilter.h"
#include "PositionFilter.h"
#include "PS3EyeTracker.h"
#include "PSMoveController.h"
#include "ServerControllerView.h"
#include "ServerDeviceView.h"
#include "ServerTrackerView.h"
#include "ServerHMDView.h"
#include "ServiceVersion.h"
#include "TrackerManager.h"
#include "TrackerCapabilitiesConfig.h"
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
	for (int contorller_id = 0; contorller_id < ControllerManager::k_max_devices; ++contorller_id)
	{
		const ControllerStreamInfo &streamInfo = m_peristentRequestState->active_controller_stream_info[contorller_id];

		// Undo the ROI suppression
		if (streamInfo.disable_roi)
		{
			m_deviceManager->getControllerViewPtr(contorller_id)->popDisableROI();
		}

		// Halt any hmd tracking this connection had going on
		if (streamInfo.include_position_data)
		{
			m_deviceManager->getControllerViewPtr(contorller_id)->stopTracking();
		}
	}

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

void ServiceRequestHandler::handle_input_data_frame(DeviceInputDataFrame &data_frame)
{
    switch (data_frame.device_category)
    {
	case DeviceCategory_CONTROLLER:
        {
            handle_data_frame__controller_packet(data_frame);
        } break;
    }
}

void ServiceRequestHandler::handle_data_frame__controller_packet(
    DeviceInputDataFrame &data_frame)
{
    const ControllerInputDataPacket &controllerDataPacket = data_frame.device.controller_data_packet;
    const int controller_id = controllerDataPacket.controller_id;

    if (Utility::is_index_valid(controller_id, m_deviceManager->getControllerViewMaxCount()))
    {
        ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);
        ControllerStreamInfo &streamInfo = m_peristentRequestState->active_controller_stream_info[controller_id];

        // Don't consider this data frame if the controller isn't in a streamable connection
        // or if the sequence number is old
        if (controller_view->getIsStreamable() && 
            controllerDataPacket.input_sequence_num > streamInfo.last_data_input_sequence_number)
        {
            // Remember the last sequence number we received from this connection
            streamInfo.last_data_input_sequence_number = controllerDataPacket.input_sequence_num;

            switch (controller_view->getControllerDeviceType())
            {
            case CommonSensorState::eDeviceType::PSMove:
                {
                    const PSVRPSMoveInput &psmove_state= controllerDataPacket.controller_state.psmove_state;

                    // Update the rumble
                    const float rumbleValue= static_cast<float>(psmove_state.Rumble) / 255.f;
                    controller_view->setControllerRumble(rumbleValue, PSVRControllerRumbleChannel_All);

                    // Update the override led color
                    {
                        unsigned char r = static_cast<unsigned char>(psmove_state.LED_r);
                        unsigned char g = static_cast<unsigned char>(psmove_state.LED_g);
                        unsigned char b = static_cast<unsigned char>(psmove_state.LED_b);

                        // (0,0,0) is treated as clearing the override
                        if (r == 0 && g == 0 && b == 0)
                        {
                            if (controller_view->getIsLEDOverrideActive())
                            {
                                // Removes the over led color and restores the tracking color
                                // of the controller is currently being tracked
                                controller_view->clearLEDOverride();
                            }
                        }
                        // Otherwise we are setting the override to a new color
                        else
                        {
                            // Sets the bulb LED color to some new override color
                            // If tracking was active this likely will affect controller tracking
                            controller_view->setLEDOverride(r, g, b);
                        }

                        // Flag if the LED override is active
                        // If the stream closes and this flag is active we'll need to clear the led override
                        streamInfo.led_override_active= controller_view->getIsLEDOverrideActive();
                    }
                } break;
            case CommonSensorState::eDeviceType::DualShock4:
                {
                    const PSVRDualShock4Input &psmove_state= controllerDataPacket.controller_state.ds4_state;

                    // Update the rumble
                    const float bigRumbleValue= static_cast<float>(psmove_state.BigRumble) / 255.f;
                    const float smallRumbleValue= static_cast<float>(psmove_state.SmallRumble) / 255.f;
                    controller_view->setControllerRumble(bigRumbleValue, PSVRControllerRumbleChannel_Left);
                    controller_view->setControllerRumble(smallRumbleValue, PSVRControllerRumbleChannel_Right);

                    // Update the override led color
                    {
                        unsigned char r = static_cast<unsigned char>(psmove_state.LED_r);
                        unsigned char g = static_cast<unsigned char>(psmove_state.LED_g);
                        unsigned char b = static_cast<unsigned char>(psmove_state.LED_b);

                        // (0,0,0) is treated as clearing the override
                        if (r == 0 && g == 0 && b == 0)
                        {
                            if (controller_view->getIsLEDOverrideActive())
                            {
                                // Removes the over led color and restores the tracking color
                                // of the controller is currently being tracked
                                controller_view->clearLEDOverride();
                            }
                        }
                        // Otherwise we are setting the override to a new color
                        else
                        {
                            // Sets the bulb LED color to some new override color
                            // If tracking was active this likely will affect controller tracking
                            controller_view->setLEDOverride(r, g, b);
                        }

                        // Flag if the LED override is active
                        // If the stream closes and this flag is active we'll need to clear the led override
                        streamInfo.led_override_active= controller_view->getIsLEDOverrideActive();
                    }
                } break;
            }
        }
    }
}

void ServiceRequestHandler::publish_controller_data_frame(
	class ServerControllerView *controller_view,
	ServiceRequestHandler::t_generate_controller_data_frame_for_stream callback)
{
    int controller_id = controller_view->getDeviceID();

    // Notify any connections that care about the tracker update
    if (m_peristentRequestState->active_controller_streams.test(controller_id))
    {
        const ControllerStreamInfo &streamInfo =
            m_peristentRequestState->active_controller_stream_info[controller_id];

        // Fill out a data frame specific to this stream using the given callback
        DeviceOutputDataFrame data_frame;
		memset(&data_frame, 0, sizeof(DeviceOutputDataFrame));
        callback(controller_view, &streamInfo, data_frame);

        // Send the hmd data frame over the network
        m_dataFrameListener->handle_data_frame(data_frame);
    }
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
		memset(&data_frame, 0, sizeof(DeviceOutputDataFrame));
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
		memset(&data_frame, 0, sizeof(DeviceOutputDataFrame));
        callback(hmd_view, &streamInfo, data_frame);

        // Send the hmd data frame over the network
        m_dataFrameListener->handle_data_frame(data_frame);
    }
}

void ServiceRequestHandler::publish_notification(const PSVREventMessage &message)
{
    m_notificationListener->handle_notification(message);
}
	
// -- controller requests -----
ServerControllerView *ServiceRequestHandler::get_controller_view_or_null(PSVRControllerID controller_id)
{
    ServerControllerView *controller_view = nullptr;

    if (Utility::is_index_valid(controller_id, m_deviceManager->getControllerViewMaxCount()))
    {
        ServerControllerViewPtr controller_view_ptr = m_deviceManager->getControllerViewPtr(controller_id);

        if (controller_view_ptr->getIsOpen())
        {
            controller_view = controller_view_ptr.get();
        }
    }

    return controller_view;
}

PSVRResult ServiceRequestHandler::get_controller_list(
	const bool include_usb,
	PSVRControllerList *out_controller_list)
{
    memset(out_controller_list, 0, sizeof(PSVRControllerList));
    for (int controller_id = 0; controller_id < m_deviceManager->getControllerViewMaxCount(); ++controller_id)
    {
        ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

        if (out_controller_list->count < PSVRSERVICE_MAX_CONTROLLER_COUNT && 
			controller_view->getIsOpen() &&
			(controller_view->getIsBluetooth() || include_usb))
        {
            PSVRClientControllerInfo *controller_info = &out_controller_list-> controllers[out_controller_list->count++];
			std::string controller_hand = "";
			std::string gyro_gain_setting = "";

            switch (controller_view->getControllerDeviceType())
            {
            case CommonSensorState::PSMove:
                {
                    const PSMoveController *psmove= controller_view->castCheckedConst<PSMoveController>();
                    const PSMoveControllerConfig *config= psmove->getConfig();

                    controller_info->controller_type= PSVRController_Move;
                    controller_info->prediction_time= config->prediction_time;
					controller_info->has_magnetometer= psmove->getSupportsMagnetometer();
					strncpy(controller_info->orientation_filter, config->orientation_filter_type.c_str(), sizeof(controller_info->orientation_filter));
					strncpy(controller_info->position_filter, config->position_filter_type.c_str(), sizeof(controller_info->position_filter));
					controller_hand= config->hand;
                }
                break;
            case CommonSensorState::DualShock4:
                {
                    const DualShock4Controller *ds4= controller_view->castCheckedConst<DualShock4Controller>();
                    const DualShock4ControllerConfig *config= ds4->getConfig();

                    controller_info->controller_type= PSVRController_DualShock4;
                    controller_info->prediction_time= config->prediction_time;
					controller_info->has_magnetometer= false;
                    strncpy(controller_info->orientation_filter, config->orientation_filter_type.c_str(), sizeof(controller_info->orientation_filter));
                    strncpy(controller_info->position_filter, config->position_filter_type.c_str(), sizeof(controller_info->position_filter));
					controller_hand= config->hand;

                    float radian_gain_divisor = safe_divide_with_default(1.f, config->gyro_gain, 1.f);
                    float degree_gain_divisor = radian_gain_divisor * k_degrees_to_radians;

                    // Gyro gain mode can vary from controller to controller
                    // Sensitivity values from Pg.15 of:
                    // https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI055-DS000-08.pdf
                    if (is_nearly_equal(degree_gain_divisor, 262.4f, 1.f))
                    {
                        gyro_gain_setting = "125deg/s";
                    }
                    else if (is_nearly_equal(degree_gain_divisor, 131.2f, 1.f))
                    {
                        gyro_gain_setting = "250deg/s";
                    }
                    else if (is_nearly_equal(degree_gain_divisor, 65.6f, 1.f))
                    {
                        gyro_gain_setting = "500deg/s";
                    }
                    else if (is_nearly_equal(degree_gain_divisor, 32.8f, 1.f))
                    {
                        gyro_gain_setting = "1000deg/s";
                    }
                    else if (is_nearly_equal(degree_gain_divisor, 16.4f, 1.f))
                    {
                        gyro_gain_setting = "2000deg/s";
                    }
                    else
                    {
                        gyro_gain_setting = "custom";
                    }
                }
                break;
            default:
                assert(0 && "Unhandled tracker type");
            }

            controller_info->controller_id= controller_id;
			controller_info->is_bluetooth= controller_view->getIsBluetooth();
            controller_info->tracking_color_type= controller_view->getTrackingColorID();
            strncpy(controller_info->device_path, controller_view->getUSBDevicePath().c_str(), sizeof(controller_info->device_path));
			strncpy(controller_info->controller_serial, controller_view->getSerial().c_str(), sizeof(controller_info->controller_serial));
			strncpy(controller_info->assigned_host_serial, controller_view->getAssignedHostBluetoothAddress().c_str(), sizeof(controller_info->assigned_host_serial));
			strncpy(controller_info->gyro_gain_setting, gyro_gain_setting.c_str(), sizeof(controller_info->gyro_gain_setting));

			if (controller_hand == "Left")
				controller_info->controller_hand= PSMControllerHand_Left;
			else if (controller_hand == "Right")
				controller_info->controller_hand= PSMControllerHand_Right;
			else
				controller_info->controller_hand= PSMControllerHand_Any;
        }
    }

	strncpy(out_controller_list->host_serial, 
		m_deviceManager->getControllerManager()->getBluetoothHostAddress().c_str(), 
		sizeof(out_controller_list->host_serial));

    return PSVRResult_Success;
}

PSVRResult ServiceRequestHandler::start_controller_data_stream(PSVRControllerID controller_id, unsigned int data_stream_flags)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

    if (controller_view && controller_view->getIsOpen())
    {
        ControllerStreamInfo &streamInfo = m_peristentRequestState->active_controller_stream_info[controller_id];

        // The controller manager will always publish updates regardless of who is listening.
        // All we have to do is keep track of which connections care about the updates.
        m_peristentRequestState->active_controller_streams.set(controller_id, true);

        // Set control flags for the stream
        streamInfo.Clear();
        streamInfo.include_position_data = (data_stream_flags & PSVRStreamFlags_includePositionData) > 0;
        streamInfo.include_physics_data = (data_stream_flags & PSVRStreamFlags_includePhysicsData) > 0;
        streamInfo.include_raw_sensor_data = (data_stream_flags & PSVRStreamFlags_includeRawSensorData) > 0;
        streamInfo.include_calibrated_sensor_data = (data_stream_flags & PSVRStreamFlags_includeCalibratedSensorData) > 0;
        streamInfo.include_raw_tracker_data = (data_stream_flags & PSVRStreamFlags_includeRawTrackerData) > 0;
        streamInfo.disable_roi = (data_stream_flags & PSVRStreamFlags_disableROI) > 0;

        PSVR_LOG_INFO("ServerRequestHandler") << "Start controller(" << controller_id << ") stream ("
            << "pos=" << streamInfo.include_position_data
            << ",phys=" << streamInfo.include_physics_data
            << ",raw_sens=" << streamInfo.include_raw_sensor_data
            << ",cal_sens=" << streamInfo.include_calibrated_sensor_data
            << ",trkr=" << streamInfo.include_raw_tracker_data
            << ",roi=" << streamInfo.disable_roi
            << ")";

        if (streamInfo.disable_roi)
        {
            controller_view->pushDisableROI();
        }

        // Attach the initial state of the controller
		controller_view->markStateAsUnpublished();
		controller_view->publish();

        if (streamInfo.include_position_data)
        {
            controller_view->startTracking();
        }

        // Return the name of the shared memory block the video frames will be written to
		result= PSVRResult_Success;
	}

	return result;
}

PSVRResult ServiceRequestHandler::stop_controller_data_stream(PSVRControllerID controller_id)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

    if (controller_view && controller_view->getIsOpen())
    {
        const ControllerStreamInfo &streamInfo = m_peristentRequestState->active_controller_stream_info[controller_id];

        if (streamInfo.disable_roi)
        {
            controller_view->popDisableROI();
        }

        if (streamInfo.include_position_data)
        {
            controller_view->stopTracking();
        }

        m_peristentRequestState->active_controller_streams.set(controller_id, false);
        m_peristentRequestState->active_controller_stream_info[controller_id].Clear();

        result= PSVRResult_Success;
    }

	return result;
}

PSVRResult ServiceRequestHandler::set_led_tracking_color(PSVRControllerID controller_id, PSVRTrackingColorType new_color_id)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr ControllerView = m_deviceManager->getControllerViewPtr(controller_id);

    if (ControllerView)
    {
        const PSVRTrackingColorType oldColorID = ControllerView->getTrackingColorID();

        if (new_color_id != oldColorID)
        {
            // Give up control of our existing tracking color
            if (oldColorID != PSVRTrackingColorType_INVALID)
            {
                m_deviceManager->m_tracker_manager->freeTrackingColorID(oldColorID);
            }

            // Take the color from any other controller that might have it
            if (m_deviceManager->m_tracker_manager->claimTrackingColorID(ControllerView.get(), new_color_id))
            {
                // Assign the new color to ourselves
                ControllerView->setTrackingColorID(new_color_id);
                result= PSVRResult_Success;
            }
            else
            {
                if (oldColorID != PSVRTrackingColorType_INVALID)
                {
                    m_deviceManager->m_tracker_manager->claimTrackingColorID(ControllerView.get(), oldColorID);
                }
            }
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::reset_orientation(PSVRControllerID controller_id, const PSVRQuatf& q_pose)
{
	ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);
	PSVRResult result= PSVRResult_Error;

	if (controller_view && controller_view->getIsOpen())
	{
		result= controller_view->recenterOrientation(q_pose) ? PSVRResult_Success : PSVRResult_Error;
	}

	return result;
}

PSVRResult ServiceRequestHandler::set_controller_data_stream_tracker_index(
	PSVRControllerID controller_id, 
	PSVRTrackerID tracker_id)
{
	PSVRResult result= PSVRResult_Error;

    if (Utility::is_index_valid(controller_id, m_deviceManager->getControllerViewMaxCount()) &&
        Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);
        ControllerStreamInfo &streamInfo = m_peristentRequestState->active_controller_stream_info[controller_id];

        PSVR_LOG_INFO("ServerRequestHandler") << "Set controller(" << controller_id << ") stream tracker id: " << tracker_id;

        streamInfo.selected_tracker_index= tracker_id;

        result= PSVRResult_Success;
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_controller_hand(PSVRControllerID controller_id, PSVRControllerHand controller_hand)
{
	PSVRResult result= PSVRResult_Error;
	ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

	if (controller_view)
	{
        std::string hand;

		switch (controller_hand)
		{
		case PSMControllerHand_Left:
			hand= "Left";
			break;
		case PSMControllerHand_Right:
			hand= "Right";
			break;
		case PSMControllerHand_Any:
		default:
			hand= "Any";
			break;
		}

		switch (controller_view->getControllerDeviceType())
		{
		case CommonSensorState::PSMove:
			{
				PSMoveController *psmove= controller_view->castChecked<PSMoveController>();
				PSMoveControllerConfig config= *psmove->getConfig();

				config.hand= hand;

				psmove->setConfig(&config);
				result= PSVRResult_Success;
			} break;
		case CommonSensorState::DualShock4:
			{
				DualShock4Controller *ds4= controller_view->castChecked<DualShock4Controller>();
				DualShock4ControllerConfig config= *ds4->getConfig();

				config.hand= hand;

				ds4->setConfig(&config);
				result= PSVRResult_Success;
			} break;
		}
	}

	return result;
}

PSVRResult ServiceRequestHandler::set_controller_accelerometer_calibration(
	PSVRControllerID controller_id, 
	float noise_radius, 
	float noise_variance)
{
	PSVRResult result= PSVRResult_Error;
	ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

	if (controller_view)
	{
		switch (controller_view->getControllerDeviceType())
		{
		case CommonSensorState::PSMove:
			{
				PSMoveController *psmove= controller_view->castChecked<PSMoveController>();
				PSMoveControllerConfig config= *psmove->getConfig();

				config.accelerometer_noise_radius= noise_radius;
				config.accelerometer_variance= noise_variance;

				psmove->setConfig(&config);
				result= PSVRResult_Success;
			} break;
		case CommonSensorState::DualShock4:
			{
				DualShock4Controller *ds4= controller_view->castChecked<DualShock4Controller>();
				DualShock4ControllerConfig config= *ds4->getConfig();

				config.accelerometer_noise_radius= noise_radius;
				config.accelerometer_variance= noise_variance;

				ds4->setConfig(&config);
				result= PSVRResult_Success;
			} break;
		}
	}

	return result;
}

PSVRResult ServiceRequestHandler::set_controller_gyroscope_calibration(
	PSVRControllerID controller_id,
	float drift,
	float variance,
	const char *gain_setting)
{
	PSVRResult result= PSVRResult_Error;
	ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

	if (controller_view)
	{
		switch (controller_view->getControllerDeviceType())
		{
		case CommonSensorState::PSMove:
			{
				PSMoveController *psmove= controller_view->castChecked<PSMoveController>();
				PSMoveControllerConfig config= *psmove->getConfig();

                config.gyro_drift= drift;
                config.gyro_variance= variance;

				psmove->setConfig(&config);
				controller_view->resetPoseFilter();

				result= PSVRResult_Success;
			} break;
		case CommonSensorState::DualShock4:
			{
				DualShock4Controller *ds4= controller_view->castChecked<DualShock4Controller>();
				DualShock4ControllerConfig config= *ds4->getConfig();

				bool bChanged = false;

				if (drift > 0.f)
				{
	                config.gyro_drift= drift;
					bChanged= true;
				}

				if (variance > 0.f)
				{
					config.gyro_variance= variance;
					bChanged= true;
				}

                const std::string gain_setting_string = gain_setting;
                if (gain_setting_string.length() > 0)
                {
                    // Sensitivity values from Pg.15 of:
                    // https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI055-DS000-08.pdf
                    if (gain_setting_string == "125deg/s")
                    {
                        config.gyro_gain = 1.f / (262.4f / k_degrees_to_radians);
						bChanged= true;
                    }
                    else if (gain_setting_string == "250deg/s")
                    {
                        config.gyro_gain = 1.f / (131.2f / k_degrees_to_radians);
						bChanged= true;
                    }
                    else if (gain_setting_string == "500deg/s")
                    {
                        config.gyro_gain = 1.f / (65.6f / k_degrees_to_radians);
						bChanged= true;
                    }
                    else if (gain_setting_string == "1000deg/s")
                    {
                        config.gyro_gain = 1.f / (32.8f / k_degrees_to_radians);
						bChanged= true;
                    }
                    else if (gain_setting_string == "2000deg/s")
                    {
                        config.gyro_gain = 1.f / (16.4f / k_degrees_to_radians);
						bChanged= true;
                    }
                }

				if (bChanged)
				{
					ds4->setConfig(&config);
					controller_view->resetPoseFilter();
				}

				result= PSVRResult_Success;
			} break;
		}
	}

	return result;
}

PSVRResult ServiceRequestHandler::set_controller_orientation_filter(PSVRControllerID controller_id, const std::string orientation_filter)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

    if (controller_view && controller_view->getIsOpen())
    {
        if (controller_view->getControllerDeviceType() == CommonSensorState::PSMove)
        {
            PSMoveController *psmove = controller_view->castChecked<PSMoveController>();
            PSMoveControllerConfig config = *psmove->getConfig();

            if (config.orientation_filter_type != orientation_filter)
            {
                config.orientation_filter_type = orientation_filter;
                psmove->setConfig(&config);				
				controller_view->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
        else if (controller_view->getControllerDeviceType() == CommonSensorState::DualShock4)
        {
            DualShock4Controller *ds4 = controller_view->castChecked<DualShock4Controller>();
            DualShock4ControllerConfig config = *ds4->getConfig();

            if (config.orientation_filter_type != orientation_filter)
            {
                config.orientation_filter_type = orientation_filter;
                ds4->setConfig(&config);
				controller_view->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_controller_position_filter(PSVRControllerID controller_id, const std::string position_filter)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

    if (controller_view && controller_view->getIsOpen())
    {
        if (controller_view->getControllerDeviceType() == CommonSensorState::PSMove)
        {
            PSMoveController *psmove = controller_view->castChecked<PSMoveController>();
            PSMoveControllerConfig config = *psmove->getConfig();

            if (config.position_filter_type != position_filter)
            {
                config.position_filter_type = position_filter;
                psmove->setConfig(&config);				
				controller_view->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
        else if (controller_view->getControllerDeviceType() == CommonSensorState::DualShock4)
        {
            DualShock4Controller *ds4 = controller_view->castChecked<DualShock4Controller>();
            DualShock4ControllerConfig config = *ds4->getConfig();

            if (config.position_filter_type != position_filter)
            {
                config.position_filter_type = position_filter;
                ds4->setConfig(&config);
				controller_view->resetPoseFilter();
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_controller_prediction_time(PSVRControllerID controller_id, const float prediction_time)
{
	PSVRResult result= PSVRResult_Error;

    ServerControllerViewPtr controller_view = m_deviceManager->getControllerViewPtr(controller_id);

    if (controller_view && controller_view->getIsOpen())
    {
        if (controller_view->getControllerDeviceType() == CommonSensorState::PSMove)
        {
            PSMoveController *psmove = controller_view->castChecked<PSMoveController>();
            PSMoveControllerConfig config = *psmove->getConfig();

            if (config.prediction_time != prediction_time)
            {
                config.prediction_time = prediction_time;
                psmove->setConfig(&config);
            }

            result= PSVRResult_Success;
        }
        else if (controller_view->getControllerDeviceType() == CommonSensorState::DualShock4)
        {
            DualShock4Controller *ds4 = controller_view->castChecked<DualShock4Controller>();
            DualShock4ControllerConfig config = *ds4->getConfig();

            if (config.prediction_time != prediction_time)
            {
                config.prediction_time = prediction_time;
                ds4->setConfig(&config);
            }

            result= PSVRResult_Success;
        }
    }
		
	return result;
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
            case ITrackerInterface::Winusb:
                tracker_info->tracker_driver= PSVRDriver_WINUSB;
                break;
            case ITrackerInterface::WindowsMediaFramework:
                tracker_info->tracker_driver= PSVRDriver_WINDOWSMEDIAFRAMEWORK;
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

			// Get the list of available tracker modes
			std::vector<std::string> mode_names;
			if (tracker_view->getAvailableTrackerModes(mode_names))
			{
				tracker_info->mode_count= 0;
				for (const std::string &mode_name : mode_names)
				{
					strncpy(tracker_info->mode_list[tracker_info->mode_count], mode_name.c_str(), MAX_PSVR_TRACKER_MODE_NAME_LENGTH);
					++tracker_info->mode_count;
				}
			}
			else
			{
				tracker_info->mode_count= 0;
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

PSVRResult ServiceRequestHandler::get_tracker_mode(const PSVRTrackerID tracker_id, std::string &out_mode)
{
	PSVRResult result= PSVRResult_Error;

	if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
            out_mode= tracker_view->getTrackerMode()->modeName;
            result= PSVRResult_Success;
        }
    }
		
	return result;
}

PSVRResult ServiceRequestHandler::set_tracker_mode(const PSVRTrackerID tracker_id, const std::string &new_mode)
{
	PSVRResult result= PSVRResult_Error;

	if (Utility::is_index_valid(tracker_id, m_deviceManager->getTrackerViewMaxCount()))
    {
        ServerTrackerViewPtr tracker_view = m_deviceManager->getTrackerViewPtr(tracker_id);
        if (tracker_view->getIsOpen())
        {
			if (tracker_view->setTrackerMode(new_mode))
			{
	            result= PSVRResult_Success;
			}
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

PSVRResult ServiceRequestHandler::set_tracker_controller_color_preset(
    const PSVRTrackerID tracker_id, 
    const PSVRControllerID controller_id, 
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
            ServerControllerView *controller_view = get_controller_view_or_null(controller_id);

            // Assign the color range
            tracker_view->setControllerTrackingColorPreset(controller_view, tracking_color_type, &desired_color_filter);
            // Read back what actually got set
            tracker_view->getControllerTrackingColorPreset(controller_view, tracking_color_type, &out_color_filter);

            result= PSVRResult_Success;
        }
    }

    return result;
}

PSVRResult ServiceRequestHandler::set_tracker_hmd_color_preset(
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

PSVRResult ServiceRequestHandler::get_tracker_debug_flags(PSVRTrackerDebugFlags *out_flags) const
{
	*out_flags= TrackerManagerConfig::debug_flags;

	return PSVRResult_Success;
}

PSVRResult ServiceRequestHandler::set_tracker_debug_flags(PSVRTrackerDebugFlags flags)
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
            streamInfo.include_position_data = (data_stream_flags & PSVRStreamFlags_includePositionData) > 0;
            streamInfo.include_physics_data = (data_stream_flags & PSVRStreamFlags_includePhysicsData) > 0;
            streamInfo.include_raw_sensor_data = (data_stream_flags & PSVRStreamFlags_includeRawSensorData) > 0;
            streamInfo.include_calibrated_sensor_data = (data_stream_flags & PSVRStreamFlags_includeCalibratedSensorData) > 0;
            streamInfo.include_raw_tracker_data = (data_stream_flags & PSVRStreamFlags_includeRawTrackerData) > 0;
            streamInfo.disable_roi = (data_stream_flags & PSVRStreamFlags_disableROI) > 0;

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

            // Attach the initial state of the HMD
			hmd_view->markStateAsUnpublished();
			hmd_view->publish();

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

//-- includes -----
#include "TrackerManager.h"
#include "TrackerDeviceEnumerator.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "Logger.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"
#include "TrackerCapabilitiesConfig.h"
#include "MathUtility.h"
#include "TrackerUSBDeviceEnumerator.h"
#include "WMFCameraEnumerator.h"
#include "USBDeviceManager.h"
#include "Utility.h"

#include <fstream>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

//-- constants -----

//-- prototypes --
static void uploadFirmwareToAllPS4Cameras(const std::string &firmware_path);

//-- Tracker Manager Config -----
const int TrackerManagerConfig::CONFIG_VERSION = 2;
PSVRTrackerDebugFlags TrackerManagerConfig::debug_flags= PSVRTrackerDebugFlags_none;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
{
    //optical_tracking_timeout= 100;
    tracker_sleep_ms = 1;
    use_bgr_to_hsv_lookup_table = true;
    min_valid_projection_area= 16;
    disable_roi = false;
    global_forward_degrees = 0.f;
};

const configuru::Config
TrackerManagerConfig::writeToJSON()
{
    configuru::Config pt{
        {"version", TrackerManagerConfig::CONFIG_VERSION},
        {"use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table},
        {"tracker_sleep_ms", tracker_sleep_ms},
        {"min_valid_projection_area", min_valid_projection_area},	
        {"disable_roi", disable_roi},
        {"global_forward_degrees", global_forward_degrees},
		{"debug_show_tracking_model", (TrackerManagerConfig::debug_flags & PSVRTrackerDebugFlags_trackingModel) > 0}
    };

    return pt;
}

void
TrackerManagerConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == TrackerManagerConfig::CONFIG_VERSION)
    {
        //optical_tracking_timeout= pt.get_or<int>("optical_tracking_timeout", optical_tracking_timeout);
        use_bgr_to_hsv_lookup_table = pt.get_or<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
        tracker_sleep_ms = pt.get_or<int>("tracker_sleep_ms", tracker_sleep_ms);
        min_valid_projection_area = pt.get_or<float>("min_valid_projection_area", min_valid_projection_area);	
        disable_roi = pt.get_or<bool>("disable_roi", disable_roi);
        global_forward_degrees= pt.get_or<float>("global_forward_degrees", global_forward_degrees);

		unsigned int debug_flags= PSVRTrackerDebugFlags_none;
		if (pt.get_or<bool>("debug_show_tracking_model", false))
		{
			debug_flags|= PSVRTrackerDebugFlags_trackingModel;
		}
		TrackerManagerConfig::debug_flags= (PSVRTrackerDebugFlags)debug_flags;
    }
    else
    {
        PSVR_LOG_WARNING("TrackerManagerConfig") <<
            "Config version " << version << " does not match expected version " <<
            TrackerManagerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

PSVRVector3f 
TrackerManagerConfig::get_global_forward_axis() const
{
    return {cosf(global_forward_degrees*k_degrees_to_radians), 0.f, sinf(global_forward_degrees*k_degrees_to_radians)};
}

PSVRVector3f 
TrackerManagerConfig::get_global_backward_axis() const
{
    return {-cosf(global_forward_degrees*k_degrees_to_radians), 0.f, -sinf(global_forward_degrees*k_degrees_to_radians)};
}

PSVRVector3f
TrackerManagerConfig::get_global_right_axis() const
{
    return {-sinf(global_forward_degrees*k_degrees_to_radians), 0.f, cosf(global_forward_degrees*k_degrees_to_radians)};
}

PSVRVector3f
TrackerManagerConfig::get_global_left_axis() const
{
    return {sinf(global_forward_degrees*k_degrees_to_radians), 0.f, -cosf(global_forward_degrees*k_degrees_to_radians)};
}

PSVRVector3f 
TrackerManagerConfig::get_global_up_axis() const
{
    return {0.f, 1.f, 0.f};
}

PSVRVector3f 
TrackerManagerConfig::get_global_down_axis() const
{
    return {0.f, -1.f, 0.f};
}

//-- Tracker Manager -----
TrackerManager::TrackerManager()
    : DeviceTypeManager(10000, 13)
	, m_supportedTrackers(new TrackerCapabilitiesSet)
    , m_tracker_list_dirty(false)
{
	// Share the supported tracker list with the tracker enumerators
	TrackerUSBDeviceEnumerator::s_supportedTrackers= m_supportedTrackers;
	WMFCameraEnumerator::s_supportedTrackers= m_supportedTrackers;
}

bool 
TrackerManager::startup()
{
    bool bSuccess = DeviceTypeManager::startup();

    if (bSuccess)
    {
        // Load any config from disk
        cfg.load();

        // Save back out the config in case there were updated defaults
        cfg.save();

		// Fetch the config files for all the trackers we support
		m_supportedTrackers->reloadSupportedTrackerCapabilities();

        // The PS4 camera needs firmware uploaded first before it will show up as a connected device
        uploadFirmwareToAllPS4Cameras(
			Utility::get_resource_directory()+std::string("\\firmware\\ps4camera_firmware.bin"));

        // Refresh the tracker list
        mark_tracker_list_dirty();

        // Put all of the available tracking colors in the queue
        for (int color_index = 0; color_index < PSVRTrackingColorType_MaxColorTypes; ++color_index)
        {
            m_available_color_ids.push_back(static_cast<PSVRTrackingColorType>(color_index));
        }
    }

    return bSuccess;
}

void 
TrackerManager::pollUpdatedVideoFrames()
{
    for (int tracker_id = 0; tracker_id < k_max_devices; ++tracker_id)
    {
        ServerTrackerViewPtr tracker_view = getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            tracker_view->pollUpdatedVideoFrame();
        }
    }
}

void
TrackerManager::closeAllTrackers()
{
    for (int tracker_id = 0; tracker_id < k_max_devices; ++tracker_id)
    {
        ServerTrackerViewPtr tracker_view = getTrackerViewPtr(tracker_id);

        if (tracker_view->getIsOpen())
        {
            tracker_view->close();
        }
    }

    // Refresh the tracker list once we're allowed to
    mark_tracker_list_dirty();

    // Tell any clients that the tracker list changed
    send_device_list_changed_notification();
}

bool
TrackerManager::can_update_connected_devices()
{
    return m_tracker_list_dirty && DeviceTypeManager::can_update_connected_devices();
}

void 
TrackerManager::mark_tracker_list_dirty()
{
    m_tracker_list_dirty= true;
}

DeviceEnumerator *
TrackerManager::allocate_device_enumerator()
{
    return new TrackerDeviceEnumerator(TrackerDeviceEnumerator::CommunicationType_ALL);
}

void
TrackerManager::free_device_enumerator(DeviceEnumerator *enumerator)
{
    delete static_cast<TrackerDeviceEnumerator *>(enumerator);

    // Tracker list is no longer dirty after we have iterated through the list of cameras
    m_tracker_list_dirty = false;
}

ServerDeviceView *
TrackerManager::allocate_device_view(int device_id)
{
    return new ServerTrackerView(device_id);
}

ServerTrackerViewPtr
TrackerManager::getTrackerViewPtr(int device_id) const
{
    assert(m_deviceViews != nullptr);

    return std::static_pointer_cast<ServerTrackerView>(m_deviceViews[device_id]);
}

int TrackerManager::getListUpdatedResponseType()
{
    return PSVREvent_trackerListUpdated;
}

PSVRTrackingColorType 
TrackerManager::allocateTrackingColorID()
{
    assert(m_available_color_ids.size() > 0);
    PSVRTrackingColorType tracking_color = m_available_color_ids.front();

    m_available_color_ids.pop_front();

    return tracking_color;
}

bool 
TrackerManager::claimTrackingColorID(const ServerControllerView *claiming_controller_view, PSVRTrackingColorType color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen())
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                PSVRTrackingColorType newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

bool 
TrackerManager::claimTrackingColorID(const ServerHMDView *claiming_hmd_view, PSVRTrackingColorType color_id)
{
    bool bColorWasInUse = false;
    bool bSuccess= true;

    // If any other controller has this tracking color, make them pick a new color (if possible)
    HMDManager *hmdManager= DeviceManager::getInstance()->m_hmd_manager;
    for (int device_id = 0; device_id < hmdManager->getMaxDevices(); ++device_id)
    {
        ServerHMDViewPtr hmd_view = hmdManager->getHMDViewPtr(device_id);

        if (hmd_view->getIsOpen() && hmd_view.get() != claiming_hmd_view)
        {
            if (hmd_view->getTrackingColorID() == color_id)
            {
                PSVRTrackingColorType newTrackingColor= allocateTrackingColorID();

                if (!hmd_view->setTrackingColorID(newTrackingColor))
                {
                    freeTrackingColorID(newTrackingColor);
                    bSuccess= false;
                }

                bColorWasInUse = true;
                break;
            }
        }
    }

    // If the color was not in use, remove it from the color queue
    if (!bColorWasInUse)
    {
        for (auto iter = m_available_color_ids.begin(); iter != m_available_color_ids.end(); ++iter)
        {
            if (*iter == color_id)
            {
                m_available_color_ids.erase(iter);
                break;
            }
        }
    }

    return bSuccess;
}

void 
TrackerManager::freeTrackingColorID(PSVRTrackingColorType color_id)
{
    assert(std::find(m_available_color_ids.begin(), m_available_color_ids.end(), color_id) == m_available_color_ids.end());
    m_available_color_ids.push_back(color_id);
}

#define FIRMWARE_UPLOAD_CHUNK_SIZE      512
#define FIRMWARE_UPLOAD_DEVICE_PID      0x0580
#define FIRMWARE_UPLOAD_DEVICE_VID      0x05a9

static void uploadFirmwareToAllPS4Cameras(const std::string &firmware_path)
{
    USBDeviceEnumerator* enumerator= usb_device_enumerator_allocate();

    while (enumerator != nullptr && usb_device_enumerator_is_valid(enumerator))
    {
        USBDeviceFilter filter;
        if (usb_device_enumerator_get_filter(enumerator, filter) &&
            filter.product_id == FIRMWARE_UPLOAD_DEVICE_PID &&
            filter.vendor_id == FIRMWARE_UPLOAD_DEVICE_VID)
        {
            t_usb_device_handle dev_handle= 
                usb_device_open(
                    enumerator, 
                    0, // interface
                    1, // configuration
                    true); // reset device

            if (dev_handle != k_invalid_usb_device_handle)
            {
                PSVR_LOG_INFO("PS4CameraTracker::uploadFirmware") << "Uploading firmware to ov580 camera...";

                uint8_t chunk[FIRMWARE_UPLOAD_CHUNK_SIZE];
                std::ifstream firmware(firmware_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate);
        
                if (firmware.is_open()) 
                {
                    uint32_t length = (uint32_t)firmware.tellg();
                    firmware.seekg(0, std::ios::beg);

                    uint16_t index = 0x14;
                    uint16_t value = 0;

                    for (uint32_t pos = 0; pos < length; pos += FIRMWARE_UPLOAD_CHUNK_SIZE)
                    {
                        uint16_t size = (FIRMWARE_UPLOAD_CHUNK_SIZE > (length - pos)) ? (length - pos) : FIRMWARE_UPLOAD_CHUNK_SIZE;

                        firmware.read((char*)chunk, size);

                        USBTransferRequest request(_USBRequestType_ControlTransfer);
                        request.payload.control_transfer.usb_device_handle= dev_handle;
                        request.payload.control_transfer.bmRequestType= 0x40;
                        request.payload.control_transfer.bRequest= 0x0;
                        request.payload.control_transfer.wValue= value;
                        request.payload.control_transfer.wIndex= index;
                        request.payload.control_transfer.wLength= size;
                        request.payload.control_transfer.timeout= 1000;
                        assert(size <= sizeof(request.payload.control_transfer.data)); 
                        memcpy(request.payload.control_transfer.data, chunk, size);
                        usb_device_submit_transfer_request_blocking(request);

                        if (((uint32_t)value + size) > 0xFFFF)
                        {
                            index += 1;
                        }

                        value += size;
                    }
                    firmware.close();

                    USBTransferRequest request(_USBRequestType_ControlTransfer);
                    request.payload.control_transfer.usb_device_handle= dev_handle;
                    request.payload.control_transfer.bmRequestType= 0x40;
                    request.payload.control_transfer.bRequest= 0x0;
                    request.payload.control_transfer.wValue= 0x2200;
                    request.payload.control_transfer.wIndex= 0x8018;
                    request.payload.control_transfer.wLength= 1;
                    request.payload.control_transfer.timeout= 1000;
                    request.payload.control_transfer.data[0]= 0x5b;
                    usb_device_submit_transfer_request_blocking(request);
            
                    PSVR_LOG_INFO("PS4CameraTracker::uploadFirmware") << "Firmware uploaded...";
                }
                else 
                {
                    PSVR_LOG_INFO("PS4CameraTracker::uploadFirmware") << "Unable to open firmware.bin!";
                }

                usb_device_close(dev_handle);
            }
        }

        usb_device_enumerator_next(enumerator);
    }

    if (enumerator != nullptr)
    {
        usb_device_enumerator_free(enumerator);
    }
}
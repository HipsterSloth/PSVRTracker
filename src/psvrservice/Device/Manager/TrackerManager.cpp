//-- includes -----
#include "TrackerManager.h"
#include "TrackerDeviceEnumerator.h"
#include "VirtualStereoCameraEnumerator.h"
#include "DeviceManager.h"
#include "HMDManager.h"
#include "Logger.h"
#include "PS4CameraTracker.h"
#include "ServerHMDView.h"
#include "ServerTrackerView.h"
#include "ServerDeviceView.h"
#include "MathUtility.h"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

//-- constants -----

//-- Tracker Manager Config -----
const int TrackerManagerConfig::CONFIG_VERSION = 2;

TrackerManagerConfig::TrackerManagerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
{
    virtual_stereo_tracker_count= 0;
    //optical_tracking_timeout= 100;
    tracker_sleep_ms = 1;
    use_bgr_to_hsv_lookup_table = true;
    min_valid_projection_area= 16;
    disable_roi = false;
    default_tracker_profile.frame_width = 640;
    default_tracker_profile.frame_rate = 40;
    default_tracker_profile.exposure = 32;
    default_tracker_profile.gain = 32;
    strncpy(
        default_tracker_profile.color_preset_table.table_name, 
        "default_tracker_profile", 
        sizeof(default_tracker_profile.color_preset_table.table_name));
    global_forward_degrees = 0.f;
    for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
    {
        default_tracker_profile.color_preset_table.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const configuru::Config
TrackerManagerConfig::writeToJSON()
{
    configuru::Config pt{
        {"version", TrackerManagerConfig::CONFIG_VERSION},
        //{"optical_tracking_timeout", optical_tracking_timeout},
        {"use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table},
        {"tracker_sleep_ms", tracker_sleep_ms},
        {"min_valid_projection_area", min_valid_projection_area},	
        {"disable_roi", disable_roi},
        {"default_tracker_profile.frame_width", default_tracker_profile.frame_width},
        {"default_tracker_profile.frame_rate", default_tracker_profile.frame_rate},
        {"default_tracker_profile.exposure", default_tracker_profile.exposure},
        {"default_tracker_profile.gain", default_tracker_profile.gain},
        {"global_forward_degrees", global_forward_degrees},
        {"virtual_stereo_tracker_count", virtual_stereo_tracker_count}
    };

    writeColorPropertyPresetTable(&default_tracker_profile.color_preset_table, pt);

    return pt;
}

void
TrackerManagerConfig::readFromJSON(const configuru::Config &pt)
{
    version = pt.get_or<int>("version", 0);

    if (version == TrackerManagerConfig::CONFIG_VERSION)
    {
        virtual_stereo_tracker_count = pt.get_or<int>("virtual_stereo_tracker_count", virtual_stereo_tracker_count);
        //optical_tracking_timeout= pt.get_or<int>("optical_tracking_timeout", optical_tracking_timeout);
        use_bgr_to_hsv_lookup_table = pt.get_or<bool>("use_bgr_to_hsv_lookup_table", use_bgr_to_hsv_lookup_table);
        tracker_sleep_ms = pt.get_or<int>("tracker_sleep_ms", tracker_sleep_ms);
        min_valid_projection_area = pt.get_or<float>("min_valid_projection_area", min_valid_projection_area);	
        disable_roi = pt.get_or<bool>("disable_roi", disable_roi);
        default_tracker_profile.frame_width = pt.get_or<float>("default_tracker_profile.frame_width", 640);
        default_tracker_profile.frame_rate = pt.get_or<float>("default_tracker_profile.frame_rate", 40);
        default_tracker_profile.exposure = pt.get_or<float>("default_tracker_profile.exposure", 32);
        default_tracker_profile.gain = pt.get_or<float>("default_tracker_profile.gain", 32);

        global_forward_degrees= pt.get_or<float>("global_forward_degrees", global_forward_degrees);

        readColorPropertyPresetTable(pt, &default_tracker_profile.color_preset_table);
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
    , m_tracker_list_dirty(false)
{
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

        // Copy the virtual stereo camera count into the Virtual Stereo Camera Enumerator's static variable.
        // This breaks the dependency between the Tracker Manager and the enumerator.
        VirtualStereoCameraEnumerator::virtual_stereo_camera_count= cfg.virtual_stereo_tracker_count;

        // The PS4 camera needs firmware uploaded first before it will show up as a connected device
        PS4CameraTracker::uploadFirmwareToAllPS4Cameras("resources/firmware.bin");

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
    return 
		cfg.virtual_stereo_tracker_count > 0 
		? new TrackerDeviceEnumerator(TrackerDeviceEnumerator::CommunicationType_VIRTUAL_STEREO)
		: new TrackerDeviceEnumerator(TrackerDeviceEnumerator::CommunicationType_NON_VIRTUAL);
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

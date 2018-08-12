// -- includes -----
#include "PS3EyeTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PS3EyeVideo.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerManager.h"
#include "TrackerCapabilitiesConfig.h"
#include "TrackerUSBDeviceEnumerator.h"
#include "opencv2/opencv.hpp"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- constants -----
#define PS3EYE_STATE_BUFFER_MAX 16

static const char *OPTION_FOV_SETTING = "FOV Setting";
static const char *OPTION_FOV_RED_DOT = "Red Dot";
static const char *OPTION_FOV_BLUE_DOT = "Blue Dot";

// -- PS3EYE Tracker
PS3EyeTracker::PS3EyeTracker()
    : m_cfg()
    , m_deviceIdentifier()
    , m_videoDevice(nullptr)
    , m_DriverType(PS3EyeTracker::Winusb)
{
}

PS3EyeTracker::~PS3EyeTracker()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~PS3EyeTracker") << "Tracker deleted without calling close() first!";
    }
}

// PSVRTracker
bool PS3EyeTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_USB, CommonSensorState::PS3EYE);
    bool success = false;

    // Skip over everything that isn't a PS3EYE
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonSensorState::PS3EYE)
    {
        enumerator.next();
    }

    if (enumerator.is_valid())
    {
        success = open(&enumerator);
    }

    return success;
}

// -- IDeviceInterface
bool PS3EyeTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonSensorState::PS3EYE)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == m_deviceIdentifier);
    }

    return matches;
}

bool PS3EyeTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();
	const int camera_index = tracker_enumerator->get_camera_index();

    bool bSuccess = true;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("PS3EyeTracker::open") << "WMFMonoTracker(" << cur_dev_path << ") already open. Ignoring request.";
    }
    else
    {
		const TrackerUSBDeviceEnumerator *usb_tracker_enumerator= tracker_enumerator->get_usb_tracker_enumerator();
		const char *unique_id= usb_tracker_enumerator->get_path();

        PSVR_LOG_INFO("PS3EyeTracker::open") << "Opening WMFMonoTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

		// Remember the path to this camera
        m_deviceIdentifier = cur_dev_path;

		//###hipstersloth $TODO Get the driver type from the usb enumerator
		//m_DriverType= usb_tracker_enumerator->get_usb_device_enumerator()->get_usb_driver_type();

		// Build a config file name from the unique id
		char config_name[256];
        Utility::format_string(config_name, sizeof(config_name), "WMFMonoCamera_%s", unique_id);

        // Load the config file for the tracker
        m_cfg = PS3EyeTrackerConfig(config_name);
        m_cfg.load();

		// Fetch the camera capabilities
		m_capabilities= usb_tracker_enumerator->getTrackerCapabilities();

		// If no mode is specified, then default to the first mode
		if (m_cfg.current_mode == "")
		{
			m_cfg.current_mode= m_capabilities->supportedModes[0].modeName;
		}

		// Find the camera mode by name
		m_currentMode= m_capabilities->findCameraMode(m_cfg.current_mode);
		if (m_currentMode != nullptr)
		{
			// Copy the tracker intrinsics over from the capabilities
			m_cfg.trackerIntrinsics= m_currentMode->intrinsics.intrinsics.mono;

			// Attempt to find a compatible video mode
			ePS3EyeVideoMode desiredVideoMode= 
				PS3EyeVideoDevice::findBestVideoMode(
					(unsigned int)m_currentMode->bufferPixelWidth,
					(unsigned int)m_currentMode->bufferPixelHeight,
					(unsigned int)m_currentMode->frameRate);

			if (desiredVideoMode != PS3EyeVideoMode_INVALID)
			{
				m_videoDevice = new PS3EyeVideoDevice(usb_tracker_enumerator->get_usb_device_enumerator());

				if (m_videoDevice->open(
						desiredVideoMode,
						m_cfg, 
						m_listener))
				{
					bSuccess = true;
				}
			}
		}

		// Save the config back out again in case defaults changed
        m_cfg.save();
    }
    
    if (!bSuccess)
    {
        close();
    }

    return bSuccess;
}

bool PS3EyeTracker::getIsOpen() const
{
    return m_videoDevice != nullptr && m_videoDevice->isStreaming();
}

void PS3EyeTracker::close()
{
    if (m_videoDevice != nullptr)
    {
        delete m_videoDevice;
        m_videoDevice = nullptr;
    }
}

CommonSensorState::eDeviceType PS3EyeTracker::getDeviceType() const
{
    return CommonSensorState::PS3EYE;
}

ITrackerInterface::eDriverType PS3EyeTracker::getDriverType() const
{
    return m_DriverType;
}

std::string PS3EyeTracker::getUSBDevicePath() const
{
    return m_deviceIdentifier;
}

bool PS3EyeTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    if (out_width != nullptr)
    {
        int width = (int)m_currentMode->intrinsics.intrinsics.mono.pixel_width;

        if (out_stride != nullptr)
        {
            // Assume 3 bytes per pixel?
            *out_stride = 3 * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        int height = (int)m_currentMode->intrinsics.intrinsics.mono.pixel_height;

        *out_height = height;
    }

    return bSuccess;
}

void PS3EyeTracker::loadSettings()
{
	const PSVRVideoPropertyConstraint *constraints= m_videoDevice->getVideoPropertyConstraints();

    m_cfg.load();

	for (int prop_index = 0; prop_index < PSVRVideoProperty_COUNT; ++prop_index)
	{
		const PSVRVideoPropertyType prop_type = (PSVRVideoPropertyType)prop_index;
		const PSVRVideoPropertyConstraint &constraint= constraints[prop_index];

		if (constraint.is_supported)
		{
			int currentValue= getVideoProperty(prop_type);
			int desiredValue= m_cfg.video_properties[prop_index];

			if (desiredValue != currentValue)
			{
				bool bUpdateConfig= false;

				if (desiredValue < constraint.min_value || 
					desiredValue > constraint.max_value)
				{
					desiredValue= constraint.default_value;
					bUpdateConfig= true;
				}

				setVideoProperty(prop_type, desiredValue, bUpdateConfig);
			}
		}
	}
}

void PS3EyeTracker::saveSettings()
{
    m_cfg.save();
}

bool PS3EyeTracker::getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const
{
	if (m_capabilities)
	{
		m_capabilities->getAvailableTrackerModes(out_mode_names);
		return true;
	}

	return false;
}

const TrackerModeConfig *PS3EyeTracker::getTrackerMode() const
{
	return m_currentMode;
}

bool PS3EyeTracker::setTrackerMode(const std::string mode_name)
{
	const TrackerModeConfig *new_mode= m_capabilities->findCameraMode(mode_name);

	if (new_mode != nullptr && new_mode != m_currentMode)
	{		
		ePS3EyeVideoMode desiredVideoMode= 
			PS3EyeVideoDevice::findBestVideoMode(
				(unsigned int)m_currentMode->bufferPixelWidth,
				(unsigned int)m_currentMode->bufferPixelHeight,
				(unsigned int)m_currentMode->frameRate);

		m_cfg.trackerIntrinsics= new_mode->intrinsics.intrinsics.mono;
		m_currentMode= new_mode;

		if (desiredVideoMode != PS3EyeVideoMode_INVALID)
		{
			m_videoDevice->open(desiredVideoMode, m_cfg, m_listener);
		}

		m_cfg.save();

		return true;
	}

	return false;
}

double PS3EyeTracker::getFrameWidth() const
{
	return (double)m_currentMode->intrinsics.intrinsics.stereo.pixel_width;
}

double PS3EyeTracker::getFrameHeight() const
{
	return (double)m_currentMode->intrinsics.intrinsics.stereo.pixel_height;
}

double PS3EyeTracker::getFrameRate() const
{
	return (double)m_currentMode->frameRate;
}

bool PS3EyeTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	return m_videoDevice->getVideoPropertyConstraint(property_type, outConstraint);
}

void PS3EyeTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
{
	m_videoDevice->setVideoProperty(property_type, desired_value);

	if (bUpdateConfig)
	{
		m_cfg.video_properties[property_type] = desired_value;
	}
}

int PS3EyeTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	return m_videoDevice->getVideoProperty(property_type);
}

void PS3EyeTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_MONO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.mono= m_cfg.trackerIntrinsics;
}

void PS3EyeTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_MONO_TRACKER_INTRINSICS);
    m_cfg.trackerIntrinsics = tracker_intrinsics.intrinsics.mono;
}

PSVRPosef PS3EyeTracker::getTrackerPose() const
{
    return m_cfg.pose;
}

void PS3EyeTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    m_cfg.pose = *pose;
    m_cfg.save();
}

void PS3EyeTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(m_cfg.trackerIntrinsics.hfov);
    outVFOV = static_cast<float>(m_cfg.trackerIntrinsics.vfov);
}

void PS3EyeTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(m_cfg.trackerIntrinsics.znear);
    outZFar = static_cast<float>(m_cfg.trackerIntrinsics.zfar);
}

void PS3EyeTracker::gatherTrackingColorPresets(
	const std::string &table_name, 
    PSVRClientTrackerSettings* settings) const
{
    settings->color_range_table= *m_cfg.getColorRangeTable(table_name);
}

void PS3EyeTracker::setTrackingColorPreset(
	const std::string &table_name, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
//    cfg.ColorPresets[color] = *preset; // from generic_camera conflict
	PSVR_HSVColorRangeTable *table= m_cfg.getOrAddColorRangeTable(table_name);

    table->color_presets[color] = *preset;
    m_cfg.save();
}

void PS3EyeTracker::getTrackingColorPreset(
	const std::string &table_name, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
	const PSVR_HSVColorRangeTable *table= m_cfg.getColorRangeTable(table_name);

    *out_preset = table->color_presets[color];
}

void PS3EyeTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}
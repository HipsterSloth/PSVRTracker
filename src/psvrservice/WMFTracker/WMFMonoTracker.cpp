// -- includes -----
#include "WMFMonoTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PSEyeVideoCapture.h"
#include "ServerTrackerView.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerCapabilitiesConfig.h"
#include "WMFCameraEnumerator.h"
#include "TrackerManager.h"
#include "WorkerThread.h"
#include "WMFVideo.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- constants -----
#define VIRTUAL_STEREO_STATE_BUFFER_MAX		16

// -- private definitions -----
// -- public methods
// -- WMF Stereo Tracker Config
const int WMFMonoTrackerConfig::CONFIG_VERSION = 1;

WMFMonoTrackerConfig::WMFMonoTrackerConfig(const std::string &fnamebase)
    : WMFCommonTrackerConfig(fnamebase)
{
	tracker_intrinsics.pixel_width= 640;
	tracker_intrinsics.pixel_height= 480;
    tracker_intrinsics.hfov= 60.0; // degrees
    tracker_intrinsics.vfov= 45.0; // degrees
    tracker_intrinsics.znear= 10.0; // cm
    tracker_intrinsics.zfar= 200.0; // cm
    tracker_intrinsics.camera_matrix= {{ 
        554.2563, 0, 320.0, // f_x, 0, c_x
        0, 554.2563, 240.0, // 0, f_y, c_y
        0, 0, 1}};
    tracker_intrinsics.distortion_coefficients= {
        -0.10771770030260086, 0.1213262677192688, 0.04875476285815239, // K1, K2, K3
        0.00091733073350042105, 0.00010589254816295579};  // P1, P2
};

const configuru::Config 
WMFMonoTrackerConfig::writeToJSON()
{
	configuru::Config pt = WMFCommonTrackerConfig::writeToJSON();

    pt["is_valid"]= is_valid;
    pt["version"]= WMFMonoTrackerConfig::CONFIG_VERSION;
    pt["max_poll_failure_count"]= max_poll_failure_count;

	writeMonoTrackerIntrinsics(pt, tracker_intrinsics);

    return pt;
}

void 
WMFMonoTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == WMFMonoTrackerConfig::CONFIG_VERSION)
    {
		WMFCommonTrackerConfig::readFromJSON(pt);

		readMonoTrackerIntrinsics(pt, tracker_intrinsics);
    }
    else
    {
        PSVR_LOG_WARNING("WMFMonoTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            WMFMonoTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- WMFMonoTracker
WMFMonoTracker::WMFMonoTracker()
    : m_cfg()
	, m_videoDevice(nullptr)
    , m_DriverType(WMFMonoTracker::Generic_Webcam)
{
}

WMFMonoTracker::~WMFMonoTracker()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~WMFMonoTracker") << "Tracker deleted without calling close() first!";
    }
}

bool WMFMonoTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_WMF, CommonSensorState::WMFMonoCamera);
    bool success = false;

    // Skip over everything that isn't a WMF camera
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonSensorState::WMFMonoCamera)
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
bool WMFMonoTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonSensorState::WMFMonoCamera)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == m_device_identifier);
    }

    return matches;
}

bool WMFMonoTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();
	const int camera_index = tracker_enumerator->get_camera_index();

    bool bSuccess = true;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("WMFMonoTracker::open") << "WMFMonoTracker(" << cur_dev_path << ") already open. Ignoring request.";
    }
    else
    {
		const WMFCameraEnumerator *wmf_enumerator= 
			tracker_enumerator->get_windows_media_foundation_camera_enumerator();
		const char *unique_id= wmf_enumerator->get_unique_identifier();

        PSVR_LOG_INFO("WMFMonoTracker::open") << "Opening WMFMonoTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

		// Remember the path to this camera
        m_device_identifier = cur_dev_path;

		// Build a config file name from the unique id
		char config_name[256];
        Utility::format_string(config_name, sizeof(config_name), "WMFMonoCamera_%s", unique_id);

        // Load the config file for the tracker
        m_cfg = WMFMonoTrackerConfig(config_name);
        m_cfg.load();

		// Fetch the camera capabilities
		m_capabilities= wmf_enumerator->getTrackerCapabilities();

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
			m_cfg.tracker_intrinsics= m_currentMode->intrinsics.intrinsics.mono;

			// Attempt to find a compatible WMF video format
			std::string mfvideoformat= std::string("MFVideoFormat_")+m_currentMode->bufferFormat;
			int desiredFormatIndex= 
				wmf_enumerator->get_device_info()->findBestDeviceFormatIndex(
					(unsigned int)m_currentMode->bufferPixelWidth,
					(unsigned int)m_currentMode->bufferPixelHeight,
					(unsigned int)m_currentMode->frameRate,
					mfvideoformat.c_str());

			if (desiredFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
			{
				m_videoDevice = 
					new WMFVideoDevice(
						wmf_enumerator->get_device_index(), *wmf_enumerator->get_device_info());

				if (m_videoDevice->open(desiredFormatIndex, m_cfg, m_listener))
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

bool WMFMonoTracker::getIsOpen() const
{
    return m_videoDevice != nullptr && m_videoDevice->getIsOpen();
}


void WMFMonoTracker::close()
{
	if (m_videoDevice != nullptr)
	{
		delete m_videoDevice;
		m_videoDevice= nullptr;
	}
}

CommonSensorState::eDeviceType WMFMonoTracker::getDeviceType() const
{
    return CommonSensorState::WMFMonoCamera;
}

ITrackerInterface::eDriverType WMFMonoTracker::getDriverType() const
{
    return m_DriverType;
}

std::string WMFMonoTracker::getUSBDevicePath() const
{
    return m_device_identifier;
}

bool WMFMonoTracker::getVideoFrameDimensions(
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

bool WMFMonoTracker::getIsFrameMirrored() const
{ 
	return m_currentMode->isFrameMirrored; 
}

bool WMFMonoTracker::getIsBufferMirrored() const
{ 
	return false; // Only ever true for stereo video feeds
}

void WMFMonoTracker::loadSettings()
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

void WMFMonoTracker::saveSettings()
{
    m_cfg.save();
}

bool WMFMonoTracker::getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const
{
	if (m_capabilities)
	{
		m_capabilities->getAvailableTrackerModes(out_mode_names);
		return true;
	}

	return false;
}

const TrackerModeConfig *WMFMonoTracker::getTrackerMode() const
{
	return m_currentMode;
}

bool WMFMonoTracker::setTrackerMode(const std::string mode_name)
{
	const TrackerModeConfig *new_mode= m_capabilities->findCameraMode(mode_name);

	if (new_mode != nullptr && new_mode != m_currentMode)
	{		
		const WMFDeviceFormatInfo *deviceInfo= m_videoDevice->getCurrentDeviceFormat();
		std::string mfvideoformat= std::string("MFVideoFormat_")+new_mode->bufferFormat;
		int desiredFormatIndex= m_videoDevice->m_deviceInfo.findBestDeviceFormatIndex(
			(unsigned int)new_mode->bufferPixelWidth,
			(unsigned int)new_mode->bufferPixelHeight,
			(unsigned int)new_mode->frameRate,
			mfvideoformat.c_str());

		m_cfg.tracker_intrinsics= new_mode->intrinsics.intrinsics.mono;
		m_currentMode= new_mode;

		if (desiredFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
		{
			m_videoDevice->open(desiredFormatIndex, m_cfg, m_listener);
		}

		m_cfg.save();

		return true;
	}

	return false;
}

double WMFMonoTracker::getFrameWidth() const
{
	return (double)m_videoDevice->getCurrentDeviceFormat()->width / 2.0;
}

double WMFMonoTracker::getFrameHeight() const
{
	return (double)m_videoDevice->getCurrentDeviceFormat()->height;
}

double WMFMonoTracker::getFrameRate() const
{
	const WMFDeviceFormatInfo *deviceFormat= m_videoDevice->getCurrentDeviceFormat();
	double frameRate= (double)deviceFormat->frame_rate_numerator / (double)deviceFormat->frame_rate_denominator;

	return frameRate;
}

bool WMFMonoTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	
	return m_videoDevice->getVideoPropertyConstraint(property_type, outConstraint);
}

void WMFMonoTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
{
	m_videoDevice->setVideoProperty(property_type, desired_value);

	if (bUpdateConfig)
	{
		m_cfg.video_properties[property_type] = desired_value;
	}
}

int WMFMonoTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	return m_videoDevice->getVideoProperty(property_type);
}

void WMFMonoTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_MONO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.mono= m_cfg.tracker_intrinsics;
}

void WMFMonoTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_MONO_TRACKER_INTRINSICS);
    m_cfg.tracker_intrinsics= tracker_intrinsics.intrinsics.mono;
}

PSVRPosef WMFMonoTracker::getTrackerPose() const
{
    return m_cfg.pose;
}

void WMFMonoTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    m_cfg.pose = *pose;
    m_cfg.save();
}

void WMFMonoTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(m_cfg.tracker_intrinsics.hfov);
    outVFOV = static_cast<float>(m_cfg.tracker_intrinsics.vfov);
}

void WMFMonoTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(m_cfg.tracker_intrinsics.znear);
    outZFar = static_cast<float>(m_cfg.tracker_intrinsics.zfar);
}

void WMFMonoTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSVRClientTrackerSettings* settings) const
{
	settings->color_range_table = *m_cfg.getColorRangeTable(controller_serial);
}

void WMFMonoTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
	PSVR_HSVColorRangeTable *table= m_cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    m_cfg.save();
}

void WMFMonoTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
	const PSVR_HSVColorRangeTable *table= m_cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

void WMFMonoTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}
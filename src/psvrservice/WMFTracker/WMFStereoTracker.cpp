// -- includes -----
#include "WMFStereoTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PSEyeVideoCapture.h"
#include "ServerTrackerView.h"
#include "TrackerDeviceEnumerator.h"
#include "WMFCameraEnumerator.h"
#include "TrackerManager.h"
#include "WorkerThread.h"
#include "WMFVideo.h"

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- public methods
// -- WMF Stereo Tracker Config
const int WMFStereoTrackerConfig::CONFIG_VERSION = 1;

WMFStereoTrackerConfig::WMFStereoTrackerConfig(const std::string &fnamebase)
    : WMFCommonTrackerConfig(fnamebase)
{
	tracker_intrinsics.pixel_width= 640;
	tracker_intrinsics.pixel_height= 480;
    tracker_intrinsics.hfov= 60.0; // degrees
    tracker_intrinsics.vfov= 45.0; // degrees
    tracker_intrinsics.znear= 10.0; // cm
    tracker_intrinsics.zfar= 200.0; // cm
    tracker_intrinsics.left_camera_matrix= {{ 
        554.2563, 0, 320.0, // f_x, 0, c_x
        0, 554.2563, 240.0, // 0, f_y, c_y
        0, 0, 1}};
    tracker_intrinsics.right_camera_matrix= {{
        554.2563, 0, 320.0,  // f_x, 0, c_x
        0, 554.2563, 240.0,  // 0, f_y, c_y
        0, 0, 1}};
    tracker_intrinsics.left_distortion_coefficients= {
        -0.10771770030260086, 0.1213262677192688, 0.04875476285815239, // K1, K2, K3
        0.00091733073350042105, 0.00010589254816295579};  // P1, P2
    tracker_intrinsics.right_distortion_coefficients= {
        -0.10771770030260086, 0.1213262677192688, 0.04875476285815239, // K1, K2, K3
        0.00091733073350042105, 0.00010589254816295579};  // P1, P2
    tracker_intrinsics.left_rectification_rotation= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    tracker_intrinsics.right_rectification_rotation= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    tracker_intrinsics.left_rectification_projection= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,}};
    tracker_intrinsics.right_rectification_projection= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,}};
    tracker_intrinsics.rotation_between_cameras= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    tracker_intrinsics.translation_between_cameras= {0.0, 0.0, 0.0};
    tracker_intrinsics.essential_matrix= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    tracker_intrinsics.fundamental_matrix= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    tracker_intrinsics.reprojection_matrix= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0}};
};

const configuru::Config 
WMFStereoTrackerConfig::writeToJSON()
{
    configuru::Config pt= WMFCommonTrackerConfig::writeToJSON();

    pt["version"]= WMFStereoTrackerConfig::CONFIG_VERSION;
    pt["frame_width"]= tracker_intrinsics.pixel_width;
    pt["frame_height"]= tracker_intrinsics.pixel_height;
    pt["hfov"]= tracker_intrinsics.hfov;
    pt["vfov"]= tracker_intrinsics.vfov;
    pt["zNear"]= tracker_intrinsics.znear;
    pt["zFar"]= tracker_intrinsics.zfar;

    writeMatrix3d(pt, "left_camera_matrix", tracker_intrinsics.left_camera_matrix);
    writeMatrix3d(pt, "right_camera_matrix", tracker_intrinsics.right_camera_matrix);

    writeDistortionCoefficients(pt, "left_distortion_cofficients", &tracker_intrinsics.left_distortion_coefficients);
    writeDistortionCoefficients(pt, "right_distortion_cofficients", &tracker_intrinsics.right_distortion_coefficients);

    writeMatrix3d(pt, "left_rectification_rotation", tracker_intrinsics.left_rectification_rotation);
    writeMatrix3d(pt, "right_rectification_rotation", tracker_intrinsics.right_rectification_rotation);

    writeMatrix34d(pt, "left_rectification_projection", tracker_intrinsics.left_rectification_projection);
    writeMatrix34d(pt, "right_rectification_projection", tracker_intrinsics.right_rectification_projection);

    writeMatrix3d(pt, "rotation_between_cameras", tracker_intrinsics.rotation_between_cameras);
    writeVector3d(pt, "translation_between_cameras", tracker_intrinsics.translation_between_cameras);
    writeMatrix3d(pt, "essential_matrix", tracker_intrinsics.essential_matrix);
    writeMatrix3d(pt, "fundamental_matrix", tracker_intrinsics.fundamental_matrix);
    writeMatrix4d(pt, "reprojection_matrix", tracker_intrinsics.reprojection_matrix);

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

    return pt;
}

void 
WMFStereoTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == WMFStereoTrackerConfig::CONFIG_VERSION)
    {
		WMFCommonTrackerConfig::readFromJSON(pt);

		tracker_intrinsics.pixel_width = pt.get_or<float>("frame_width", 640.f);
		tracker_intrinsics.pixel_height = pt.get_or<float>("frame_height", 480.f);
        tracker_intrinsics.hfov = pt.get_or<float>("hfov", 60.f);
        tracker_intrinsics.vfov = pt.get_or<float>("vfov", 45.f);
        tracker_intrinsics.znear = pt.get_or<float>("zNear", 10.f);
        tracker_intrinsics.zfar = pt.get_or<float>("zFar", 200.f);

        readMatrix3d(pt, "left_camera_matrix", tracker_intrinsics.left_camera_matrix);
        readMatrix3d(pt, "right_camera_matrix", tracker_intrinsics.right_camera_matrix);

        readDistortionCoefficients(pt, "left_distortion_cofficients", 
            &tracker_intrinsics.left_distortion_coefficients, 
            &tracker_intrinsics.left_distortion_coefficients);
        readDistortionCoefficients(pt, "right_distortion_cofficients", 
            &tracker_intrinsics.right_distortion_coefficients, 
            &tracker_intrinsics.right_distortion_coefficients);

        readMatrix3d(pt, "left_rectification_rotation", tracker_intrinsics.left_rectification_rotation);
        readMatrix3d(pt, "right_rectification_rotation", tracker_intrinsics.right_rectification_rotation);

        readMatrix34d(pt, "left_rectification_projection", tracker_intrinsics.left_rectification_projection);
        readMatrix34d(pt, "right_rectification_projection", tracker_intrinsics.right_rectification_projection);

        readMatrix3d(pt, "rotation_between_cameras", tracker_intrinsics.rotation_between_cameras);
        readVector3d(pt, "translation_between_cameras", tracker_intrinsics.translation_between_cameras);
        readMatrix3d(pt, "essential_matrix", tracker_intrinsics.essential_matrix);
        readMatrix3d(pt, "fundamental_matrix", tracker_intrinsics.fundamental_matrix);
        readMatrix4d(pt, "reprojection_matrix", tracker_intrinsics.reprojection_matrix);
    }
    else
    {
        PSVR_LOG_WARNING("WMFStereoTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            WMFStereoTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

// -- WMFStereoTracker
WMFStereoTracker::WMFStereoTracker()
    : m_cfg()
	, m_videoDevice(nullptr)
    , m_DriverType(WMFStereoTracker::Generic_Webcam)
{
}

WMFStereoTracker::~WMFStereoTracker()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~WMFStereoTracker") << "Tracker deleted without calling close() first!";
    }
}

bool WMFStereoTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_WMF, CommonSensorState::WMFStereoCamera);
    bool success = false;

    // Skip over everything that isn't a WMF camera
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonSensorState::WMFStereoCamera)
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
bool WMFStereoTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonSensorState::WMFStereoCamera)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == m_device_identifier);
    }

    return matches;
}

bool WMFStereoTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();
	const int camera_index = tracker_enumerator->get_camera_index();

    bool bSuccess = true;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("WMFStereoTracker::open") << "WMFStereoTracker(" << cur_dev_path << ") already open. Ignoring request.";
    }
    else
    {
		const WMFCameraEnumerator *wmf_enumerator= 
			tracker_enumerator->get_windows_media_foundation_camera_enumerator();
		const char *unique_id= wmf_enumerator->get_unique_identifier();

        PSVR_LOG_INFO("WMFStereoTracker::open") << "Opening WMFStereoTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

		// Remember the path to this camera
        m_device_identifier = cur_dev_path;

		// Build a config file name from the unique id
		char config_name[256];
        Utility::format_string(config_name, sizeof(config_name), "WMFStereoCamera_%s", unique_id);

        // Load the config file for the tracker
        m_cfg = WMFStereoTrackerConfig(config_name);
        m_cfg.load();

		int desiredFormatIndex= 
			wmf_enumerator->get_device_info()->findBestDeviceFormatIndex(
				2*(unsigned int)m_cfg.tracker_intrinsics.pixel_width,
				(unsigned int)m_cfg.tracker_intrinsics.pixel_height,
				(unsigned int)m_cfg.frame_rate,
				CAMERA_BUFFER_FORMAT_MJPG);

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

		// Save the config back out again in case defaults changed
        m_cfg.save();
    }
    
    if (!bSuccess)
    {
        close();
    }

    return bSuccess;
}

bool WMFStereoTracker::getIsOpen() const
{
    return m_videoDevice != nullptr && m_videoDevice->getIsOpen();
}


void WMFStereoTracker::close()
{
	if (m_videoDevice != nullptr)
	{
		delete m_videoDevice;
		m_videoDevice= nullptr;
	}
}

CommonSensorState::eDeviceType WMFStereoTracker::getDeviceType() const
{
    return CommonSensorState::WMFStereoCamera;
}

ITrackerInterface::eDriverType WMFStereoTracker::getDriverType() const
{
    return m_DriverType;
}

std::string WMFStereoTracker::getUSBDevicePath() const
{
    return m_device_identifier;
}

bool WMFStereoTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    if (out_width != nullptr)
    {
        int width = m_videoDevice->getCurrentDeviceFormat()->width / 2;

        if (out_stride != nullptr)
        {
            // Assume 3 bytes per pixel?
            *out_stride = 3 * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        int height = m_videoDevice->getCurrentDeviceFormat()->height;

        *out_height = height;
    }

    return bSuccess;
}

void WMFStereoTracker::loadSettings()
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

void WMFStereoTracker::saveSettings()
{
    m_cfg.save();
}

void WMFStereoTracker::setFrameWidth(double value, bool bUpdateConfig)
{
	const WMFDeviceFormatInfo *deviceInfo= m_videoDevice->getCurrentDeviceFormat();
	int desiredFormatIndex= m_videoDevice->m_deviceInfo.findBestDeviceFormatIndex(
		2*(unsigned int)value,
		UNSPECIFIED_CAMERA_HEIGHT, 
		(unsigned int)getFrameRate(),
		CAMERA_BUFFER_FORMAT_MJPG);

	if (desiredFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
	{
		m_videoDevice->open(desiredFormatIndex, m_cfg, m_listener);
	}

	if (bUpdateConfig)
	{
		m_cfg.tracker_intrinsics.pixel_width = static_cast<float>(value);
	}

	m_cfg.save();
}

double WMFStereoTracker::getFrameWidth() const
{
	return (double)m_videoDevice->getCurrentDeviceFormat()->width / 2.0;
}

void WMFStereoTracker::setFrameHeight(double value, bool bUpdateConfig)
{
	const WMFDeviceFormatInfo *deviceInfo= m_videoDevice->getCurrentDeviceFormat();
	int desiredFormatIndex= m_videoDevice->m_deviceInfo.findBestDeviceFormatIndex(
		UNSPECIFIED_CAMERA_WIDTH, 
		(unsigned int)value,
		(unsigned int)getFrameRate(),
		CAMERA_BUFFER_FORMAT_MJPG);

	if (desiredFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
	{
		m_videoDevice->open(desiredFormatIndex, m_cfg, m_listener);
	}

	if (bUpdateConfig)
	{
		m_cfg.tracker_intrinsics.pixel_height = static_cast<float>(value);
	}

	m_cfg.save();
}

double WMFStereoTracker::getFrameHeight() const
{
	return (double)m_videoDevice->getCurrentDeviceFormat()->height;
}

void WMFStereoTracker::setFrameRate(double value, bool bUpdateConfig)
{
    if (getFrameRate() != value)
    {
		const WMFDeviceFormatInfo *deviceInfo= m_videoDevice->getCurrentDeviceFormat();
		int desiredFormatIndex= m_videoDevice->m_deviceInfo.findBestDeviceFormatIndex(
			deviceInfo->width, 
			deviceInfo->height,
			(unsigned int)value,
			CAMERA_BUFFER_FORMAT_MJPG);

		if (desiredFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
		{
			m_videoDevice->open(desiredFormatIndex, m_cfg, m_listener);
		}

	    if (bUpdateConfig)
	    {
		    m_cfg.frame_rate = value;
	    }

		m_cfg.save();
    }
}

double WMFStereoTracker::getFrameRate() const
{
	const WMFDeviceFormatInfo *deviceFormat= m_videoDevice->getCurrentDeviceFormat();
	double frameRate= (double)deviceFormat->frame_rate_numerator / (double)deviceFormat->frame_rate_denominator;

	return frameRate;
}

bool WMFStereoTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	
	return m_videoDevice->getVideoPropertyConstraint(property_type, outConstraint);
}

void WMFStereoTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
{
	m_videoDevice->setVideoProperty(property_type, desired_value);

	if (bUpdateConfig)
	{
		m_cfg.video_properties[property_type] = desired_value;
	}
}

int WMFStereoTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	return m_videoDevice->getVideoProperty(property_type);
}

void WMFStereoTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_STEREO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.stereo= m_cfg.tracker_intrinsics;
}

void WMFStereoTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS);
    m_cfg.tracker_intrinsics= tracker_intrinsics.intrinsics.stereo;
}

PSVRPosef WMFStereoTracker::getTrackerPose() const
{
    return m_cfg.pose;
}

void WMFStereoTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    m_cfg.pose = *pose;
    m_cfg.save();
}

void WMFStereoTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(m_cfg.tracker_intrinsics.hfov);
    outVFOV = static_cast<float>(m_cfg.tracker_intrinsics.vfov);
}

void WMFStereoTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(m_cfg.tracker_intrinsics.znear);
    outZFar = static_cast<float>(m_cfg.tracker_intrinsics.zfar);
}

void WMFStereoTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSVRClientTrackerSettings* settings) const
{
	settings->color_range_table = *m_cfg.getColorRangeTable(controller_serial);
}

void WMFStereoTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
	PSVR_HSVColorRangeTable *table= m_cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    m_cfg.save();
}

void WMFStereoTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
	const PSVR_HSVColorRangeTable *table= m_cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

void WMFStereoTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}
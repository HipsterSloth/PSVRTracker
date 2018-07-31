// -- includes -----
#include "PS3EyeTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PSEyeVideoCapture.h"
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

// -- private definitions -----
class PSEyeCaptureData
{
public:
    PSEyeCaptureData()
        : frame()
    {

    }

    cv::Mat frame;
};

// -- public methods
// -- PS3EYE Controller Config
const int PS3EyeTrackerConfig::CONFIG_VERSION = 7;
const int PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION= 1;

PS3EyeTrackerConfig::PS3EyeTrackerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
	, current_mode("640x480(60FPS)")
    , exposure(32)
    , gain(32)
    , fovSetting(BlueDot)
{
	trackerIntrinsics.pixel_width= 640.f;
	trackerIntrinsics.pixel_height= 480.f;
    trackerIntrinsics.camera_matrix= {
        554.2563, 0.0, 320.0, 
        0.0, 554.2563, 240.0,
        0.0, 0.0, 1.0}; // pixels
    trackerIntrinsics.hfov= 60.0; // degrees
    trackerIntrinsics.vfov= 45.0; // degrees
    trackerIntrinsics.znear= 10.0; // cm
    trackerIntrinsics.zfar= 200.0; // cm
    trackerIntrinsics.distortion_coefficients.k1= -0.10771770030260086;
    trackerIntrinsics.distortion_coefficients.k2= 0.1213262677192688;
    trackerIntrinsics.distortion_coefficients.k3= 0.04875476285815239;
    trackerIntrinsics.distortion_coefficients.p1= 0.00091733073350042105;
    trackerIntrinsics.distortion_coefficients.p2= 0.00010589254816295579;
	
    pose = *k_PSVR_pose_identity;
    memset(&SharedColorPresets, 0, sizeof(PSVR_HSVColorRangeTable));
};

const configuru::Config
PS3EyeTrackerConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"version", PS3EyeTrackerConfig::CONFIG_VERSION},
        {"lens_calibration_version", PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION},
        {"max_poll_failure_count", max_poll_failure_count},
        {"current_mode", current_mode},
        {"exposure", exposure},
        {"gain", gain},
        {"hfov", trackerIntrinsics.hfov},
        {"vfov", trackerIntrinsics.vfov},
        {"zNear", trackerIntrinsics.znear},
        {"zFar", trackerIntrinsics.zfar},
        {"fovSetting", static_cast<int>(fovSetting)},
        {"pose.orientation.w", pose.Orientation.w},
        {"pose.orientation.x", pose.Orientation.x},
        {"pose.orientation.y", pose.Orientation.y},
        {"pose.orientation.z", pose.Orientation.z},
        {"pose.position.x", pose.Position.x},
        {"pose.position.y", pose.Position.y},
        {"pose.position.z", pose.Position.z}
    };

    writeMatrix3d(pt, "camera_matrix", trackerIntrinsics.camera_matrix);
    writeDistortionCoefficients(pt, "distortion", &trackerIntrinsics.distortion_coefficients);

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

    return pt;
}

void
PS3EyeTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == PS3EyeTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get_or<bool>("is_valid", false);
        max_poll_failure_count = pt.get_or<long>("max_poll_failure_count", 100);
		current_mode= pt.get_or<std::string>("current_mode", current_mode);
        exposure = (int)pt.get_or<float>("exposure", 32);
		gain = (int)pt.get_or<float>("gain", 32);

		int lens_calibration_version = pt.get_or<int>("lens_calibration_version", 0);
		if (lens_calibration_version == PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION)
		{
            trackerIntrinsics.hfov = pt.get_or<float>("hfov", 60.0f);
            trackerIntrinsics.vfov = pt.get_or<float>("vfov", 45.0f);
            trackerIntrinsics.znear = pt.get_or<float>("zNear", 10.0f);
            trackerIntrinsics.zfar = pt.get_or<float>("zFar", 200.0f);
		    trackerIntrinsics.pixel_width = pt.get_or<float>("frame_width", 640.f);
		    trackerIntrinsics.pixel_height = pt.get_or<float>("frame_height", 480.f);

            readMatrix3d(pt, "camera_matrix", trackerIntrinsics.camera_matrix);
            readDistortionCoefficients(pt, "distortion", 
                &trackerIntrinsics.distortion_coefficients, 
                &trackerIntrinsics.distortion_coefficients);
		}
		else
		{
			PSVR_LOG_WARNING("PS3EyeTrackerConfig") <<
				"Config version " << lens_calibration_version << " does not match expected version " <<
				PS3EyeTrackerConfig::LENS_CALIBRATION_VERSION << ", Using defaults.";
		}

        fovSetting = 
            static_cast<PS3EyeTrackerConfig::eFOVSetting>(
                pt.get_or<int>("fovSetting", PS3EyeTrackerConfig::eFOVSetting::BlueDot));

        pose.Orientation.w = pt.get_or<float>("pose.orientation.w", 1.0);
        pose.Orientation.x = pt.get_or<float>("pose.orientation.x", 0.0);
        pose.Orientation.y = pt.get_or<float>("pose.orientation.y", 0.0);
        pose.Orientation.z = pt.get_or<float>("pose.orientation.z", 0.0);
        pose.Position.x = pt.get_or<float>("pose.position.x", 0.0);
        pose.Position.y = pt.get_or<float>("pose.position.y", 0.0);
        pose.Position.z = pt.get_or<float>("pose.position.z", 0.0);

		// Read the default preset table
		readColorPropertyPresetTable(pt, &SharedColorPresets);

		// Read all of the controller preset tables
		const std::string controller_prefix("controller_");
		const std::string hmd_prefix("hmd_");
		for(auto iter : pt.as_object())
		{
			const std::string &entry_name= iter.key();
			
			if (entry_name.compare(0, controller_prefix.length(), controller_prefix) == 0 ||
				entry_name.compare(0, hmd_prefix.length(), hmd_prefix) == 0)
			{
				PSVR_HSVColorRangeTable table;

				strncpy(table.table_name, entry_name.c_str(), sizeof(table.table_name));
				for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
				{
					table.color_presets[preset_index] = k_default_color_presets[preset_index];
				}

				readColorPropertyPresetTable(pt, &table);

				DeviceColorPresets.push_back(table);
			}
		}
    }
    else
    {
        PSVR_LOG_WARNING("PS3EyeTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            PS3EyeTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const PSVR_HSVColorRangeTable *
PS3EyeTrackerConfig::getColorRangeTable(const std::string &table_name) const
{
	const PSVR_HSVColorRangeTable *table= &SharedColorPresets;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}
	}

	return table;
}

inline PSVR_HSVColorRangeTable *
PS3EyeTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
{
	PSVR_HSVColorRangeTable *table= nullptr;	

	if (table_name.length() > 0)
	{
		for (auto &entry : DeviceColorPresets)
		{
			if (entry.table_name == table_name)
			{
				table= &entry;
			}
		}

		if (table == nullptr)
		{
			PSVR_HSVColorRangeTable Table;

			strncpy(Table.table_name, table_name.c_str(), sizeof(Table.table_name));
			for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
			{
				Table.color_presets[preset_index] = k_default_color_presets[preset_index];
			}

			DeviceColorPresets.push_back(Table);
			table= &DeviceColorPresets[DeviceColorPresets.size() - 1];
		}
	}
	else
	{
		table= &SharedColorPresets;
	}

	return table;
}

// -- PS3EYE Tracker
PS3EyeTracker::PS3EyeTracker()
    : cfg()
    , USBDevicePath()
    , VideoCapture(nullptr)
    , CaptureData(nullptr)
    , DriverType(PS3EyeTracker::Libusb)
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

        matches = (enumerator_path == USBDevicePath);
    }

    return matches;
}

bool PS3EyeTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();

    bool bSuccess = false;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("PS3EyeTracker::open") << "PS3EyeTracker(" << cur_dev_path << ") already open. Ignoring request.";
        bSuccess = true;
    }
    else
    {
        const int camera_index = tracker_enumerator->get_camera_index();

        PSVR_LOG_INFO("PS3EyeTracker::open") << "Opening PS3EyeTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

        VideoCapture = new PSEyeVideoCapture(camera_index);

        if (VideoCapture->isOpened())
        {
            CaptureData = new PSEyeCaptureData;
            USBDevicePath = enumerator->get_path();
            bSuccess = true;
        }
        else
        {
            PSVR_LOG_ERROR("PS3EyeTracker::open") << "Failed to open PS3EyeTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

            close();
        }
    }
    
    if (bSuccess)
    {
		const TrackerUSBDeviceEnumerator *tracker_usb_enumerator = tracker_enumerator->get_usb_tracker_enumerator();

        std::string identifier = VideoCapture->getUniqueIndentifier();
        std::string config_name = "PS3EyeTrackerConfig_";
        config_name.append(identifier);

        cfg = PS3EyeTrackerConfig(config_name);

		// Load the ps3eye config
        cfg.load();

		// If no mode is specified, then default to the first mode
		if (cfg.current_mode == "")
		{
			cfg.current_mode= m_capabilities->supportedModes[0].modeName;
		}

		// Apply the mode settings (frame width, height, fps, ...)
		m_capabilities= tracker_usb_enumerator->getTrackerCapabilities();
		setTrackerMode(cfg.current_mode);

		// Save the config back out again in case defaults changed
		cfg.save();

		VideoCapture->set(cv::CAP_PROP_EXPOSURE, (double)cfg.exposure);
		VideoCapture->set(cv::CAP_PROP_GAIN, (double)cfg.gain);
    }

    return bSuccess;
}

bool PS3EyeTracker::getIsOpen() const
{
    return VideoCapture != nullptr;
}

void PS3EyeTracker::close()
{
    if (CaptureData != nullptr)
    {
        delete CaptureData;
        CaptureData = nullptr;
    }

    if (VideoCapture != nullptr)
    {
        delete VideoCapture;
        VideoCapture = nullptr;
    }
}

CommonSensorState::eDeviceType PS3EyeTracker::getDeviceType() const
{
    return CommonSensorState::PS3EYE;
}

ITrackerInterface::eDriverType PS3EyeTracker::getDriverType() const
{
    //###bwalker $TODO Get the driver type from VideoCapture
    return DriverType;
}

std::string PS3EyeTracker::getUSBDevicePath() const
{
    return USBDevicePath;
}

bool PS3EyeTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    if (out_width != nullptr)
    {
        int width = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH));

        if (out_stride != nullptr)
        {
            int format = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FORMAT));
            int bytes_per_pixel;

            if (format != -1)
            {
                switch (format)
                {
                case cv::CAP_MODE_BGR:
                case cv::CAP_MODE_RGB:
                    bytes_per_pixel = 3;
                    break;
                case cv::CAP_MODE_YUYV:
                    bytes_per_pixel = 2;
                    break;
                case cv::CAP_MODE_GRAY:
                    bytes_per_pixel = 1;
                    break;
                default:
                    assert(false && "Unknown video format?");
                    break;
                }
            }
            else
            {
                // Assume RGB?
                PSVR_LOG_ERROR("PS3EyeTracker::getVideoFrameDimensions") << "Unknown video format for camera" << USBDevicePath << ")";
                bytes_per_pixel = 3;
            }

            *out_stride = bytes_per_pixel * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        int height = static_cast<int>(VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT));

        *out_height = height;
    }

    return bSuccess;
}

void PS3EyeTracker::loadSettings()
{
	const double currentFrameWidth = VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
	const double currentFrameRate = VideoCapture->get(cv::CAP_PROP_FPS);
    const int currentExposure= (int)VideoCapture->get(cv::CAP_PROP_EXPOSURE);
    const int currentGain= (int)VideoCapture->get(cv::CAP_PROP_GAIN);

    cfg.load();

    if (currentExposure != cfg.exposure)
    {
        VideoCapture->set(cv::CAP_PROP_EXPOSURE, cfg.exposure);
    }

    if (currentGain != cfg.gain)
    {
        VideoCapture->set(cv::CAP_PROP_GAIN, (double)cfg.gain);
    }

	setTrackerMode(cfg.current_mode);
}

void PS3EyeTracker::saveSettings()
{
    cfg.save();
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
		if (new_mode->intrinsics.intrinsics.mono.pixel_width != getFrameWidth())
		{
			VideoCapture->set(
				cv::CAP_PROP_FRAME_WIDTH,
				(double)new_mode->intrinsics.intrinsics.mono.pixel_width);
		}

		if (new_mode->intrinsics.intrinsics.mono.pixel_height != getFrameHeight())
		{
			VideoCapture->set(
				cv::CAP_PROP_FRAME_HEIGHT,
				(double)new_mode->intrinsics.intrinsics.mono.pixel_height);
		}

		if (new_mode->frameRate != getFrameRate())
		{
			VideoCapture->set(cv::CAP_PROP_FPS, (double)new_mode->frameRate);
		}

		cfg.trackerIntrinsics= new_mode->intrinsics.intrinsics.mono;
		m_currentMode= new_mode;

		return true;
	}

	return false;
}

double PS3EyeTracker::getFrameWidth() const
{
	return VideoCapture->get(cv::CAP_PROP_FRAME_WIDTH);
}

double PS3EyeTracker::getFrameHeight() const
{
	return VideoCapture->get(cv::CAP_PROP_FRAME_HEIGHT);
}

double PS3EyeTracker::getFrameRate() const
{
	return VideoCapture->get(cv::CAP_PROP_FPS);
}

bool PS3EyeTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	memset(&outConstraint, 0, sizeof(PSVRVideoPropertyConstraint));

	switch (property_type)
	{
	case PSVRVideoProperty_Exposure:
	case PSVRVideoProperty_Gain:
		{
			outConstraint.default_value= 32;
			outConstraint.is_automatic= false;
			outConstraint.is_supported= true;
			outConstraint.min_value= 0;
			outConstraint.max_value= 255;
			outConstraint.stepping_delta= 8;
		} break;
	}

	return outConstraint.is_supported;
}

void PS3EyeTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
{
	switch (property_type)
	{
	case PSVRVideoProperty_Exposure:
		{
			VideoCapture->set(cv::CAP_PROP_EXPOSURE, (double)desired_value);

			if (bUpdateConfig)
			{
				cfg.exposure = desired_value;
			}
		} break;
	case PSVRVideoProperty_Gain:
		{
			VideoCapture->set(cv::CAP_PROP_GAIN, (double)desired_value);

			if (bUpdateConfig)
			{
				cfg.gain = desired_value;
			}
		} break;
	}
}

int PS3EyeTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	int value= 0;

	switch (property_type)
	{
	case PSVRVideoProperty_Exposure:
		value= (int)VideoCapture->get(cv::CAP_PROP_EXPOSURE);
		break;
	case PSVRVideoProperty_Gain:
		value= (int)VideoCapture->get(cv::CAP_PROP_GAIN);
		break;
	}

	return value;
}

void PS3EyeTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_MONO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.mono= cfg.trackerIntrinsics;
}

void PS3EyeTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_MONO_TRACKER_INTRINSICS);
    cfg.trackerIntrinsics = tracker_intrinsics.intrinsics.mono;
}

PSVRPosef PS3EyeTracker::getTrackerPose() const
{
    return cfg.pose;
}

void PS3EyeTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    cfg.pose = *pose;
    cfg.save();
}

void PS3EyeTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(cfg.trackerIntrinsics.hfov);
    outVFOV = static_cast<float>(cfg.trackerIntrinsics.vfov);
}

void PS3EyeTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(cfg.trackerIntrinsics.znear);
    outZFar = static_cast<float>(cfg.trackerIntrinsics.zfar);
}

void PS3EyeTracker::gatherTrackingColorPresets(
	const std::string &table_name, 
    PSVRClientTrackerSettings* settings) const
{
    settings->color_range_table= *cfg.getColorRangeTable(table_name);
}

void PS3EyeTracker::setTrackingColorPreset(
	const std::string &table_name, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
//    cfg.ColorPresets[color] = *preset; // from generic_camera conflict
	PSVR_HSVColorRangeTable *table= cfg.getOrAddColorRangeTable(table_name);

    table->color_presets[color] = *preset;
    cfg.save();
}

void PS3EyeTracker::getTrackingColorPreset(
	const std::string &table_name, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
	const PSVR_HSVColorRangeTable *table= cfg.getColorRangeTable(table_name);

    *out_preset = table->color_presets[color];
}

void PS3EyeTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}
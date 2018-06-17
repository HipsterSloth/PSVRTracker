// -- includes -----
#include "VirtualStereoTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PSEyeVideoCapture.h"
#include "ServerTrackerView.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerUSBDeviceEnumerator.h"
#include "TrackerManager.h"
#include "opencv2/opencv.hpp"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- constants -----
#define VIRTUAL_STEREO_STATE_BUFFER_MAX 16

// -- private definitions -----
class VirtualStereoCaptureData
{
public:
    VirtualStereoCaptureData()
        : frame()
    {

    }

    cv::Mat frame;
};

// -- public methods
// -- Virtual Stereo Tracker Config
const int VirtualStereoTrackerConfig::CONFIG_VERSION = 1;

VirtualStereoTrackerConfig::VirtualStereoTrackerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
	, frame_rate(60)
    , exposure(32)
    , gain(32)
    , left_camera_usb_path("USB\\VID_1415&PID_2000\\bA_pB.C")
    , right_camera_usb_path("USB\\VID_1415&PID_2000\\bX_pY.Z")
{
    pose= *k_PSVR_pose_identity;

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

	SharedColorPresets.table_name[0]= 0;
    for (int preset_index = 0; preset_index < PSVRTrackingColorType_MaxColorTypes; ++preset_index)
    {
        SharedColorPresets.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const configuru::Config 
VirtualStereoTrackerConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"version", VirtualStereoTrackerConfig::CONFIG_VERSION},
        {"max_poll_failure_count", max_poll_failure_count},
        {"frame_width", tracker_intrinsics.pixel_width},
        {"frame_height", tracker_intrinsics.pixel_height},
        {"frame_rate", frame_rate},
        {"exposure", exposure},
        {"gain", gain},
        {"hfov", tracker_intrinsics.hfov},
        {"vfov", tracker_intrinsics.vfov},
        {"zNear", tracker_intrinsics.znear},
        {"zFar", tracker_intrinsics.zfar},
        {"pose.orientation.w", pose.Orientation.w},
        {"pose.orientation.x", pose.Orientation.x},
        {"pose.orientation.y", pose.Orientation.y},
        {"pose.orientation.z", pose.Orientation.z},
        {"pose.position.x", pose.Position.x},
        {"pose.position.y", pose.Position.y},
        {"pose.position.z", pose.Position.z},
        {"left_camera_usb_path", left_camera_usb_path},
        {"right_camera_usb_path", right_camera_usb_path}
    };

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
VirtualStereoTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == VirtualStereoTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get_or<bool>("is_valid", false);
        max_poll_failure_count = pt.get_or<long>("max_poll_failure_count", 100);
		frame_rate = pt.get_or<double>("frame_rate", 60);
        exposure = (int)pt.get_or<float>("exposure", 32);
		gain = (int)pt.get_or<float>("gain", 32);

        left_camera_usb_path= pt.get<std::string>("left_camera_usb_path");
        right_camera_usb_path= pt.get<std::string>("right_camera_usb_path");

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
        for (auto& iter : pt.as_object())
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
        PSVR_LOG_WARNING("VirtualStereoTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            VirtualStereoTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const PSVR_HSVColorRangeTable *
VirtualStereoTrackerConfig::getColorRangeTable(const std::string &table_name) const
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
VirtualStereoTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
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
VirtualStereoTracker::VirtualStereoTracker()
    : cfg()
    , LeftTracker(nullptr)
    , RightTracker(nullptr)
    , CaptureData(nullptr)
    , DriverType(VirtualStereoTracker::Libusb)
{
}

VirtualStereoTracker::~VirtualStereoTracker()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~VirtualStereoTracker") << "Tracker deleted without calling close() first!";
    }
}

// PSMoveTracker
bool VirtualStereoTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_VIRTUAL_STEREO, CommonSensorState::VirtualStereoCamera);
    bool success = false;

    // Skip over everything that isn't a PS3EYE
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonSensorState::VirtualStereoCamera)
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
bool VirtualStereoTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonSensorState::VirtualStereoCamera)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == device_identifier);
    }

    return matches;
}

bool VirtualStereoTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();

    bool bSuccess = true;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("VirtualStereoTracker::open") << "VirtualStereoTracker(" << cur_dev_path << ") already open. Ignoring request.";
    }
    else
    {
        const int camera_index = tracker_enumerator->get_camera_index();

        PSVR_LOG_INFO("VirtualStereoTracker::open") << "Opening VirtualStereoTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

        device_identifier = cur_dev_path;

        // Load the config file for the tracker
        cfg = VirtualStereoTrackerConfig(device_identifier);
        cfg.load();

        // Save the config back out again in case defaults changed
        cfg.save();

        if (cfg.left_camera_usb_path.length() == 0)
        {
            PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                "VirtualStereoTracker(" << cur_dev_path << 
                ", camera_index=" << camera_index << 
                ") has empty left camera usb path";
            bSuccess= false;
        }
        else if (cfg.right_camera_usb_path.length() == 0)
        {
            PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                "VirtualStereoTracker(" << cur_dev_path << 
                ", camera_index=" << camera_index << 
                ") has empty right camera usb path";
            bSuccess= false;
        }
        else if (cfg.right_camera_usb_path == cfg.left_camera_usb_path)
        {
            PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                "VirtualStereoTracker(" << cur_dev_path << 
                ", camera_index=" << camera_index << 
                ") has non-unique left and right cameras.";
            bSuccess= false;
        }

        if (bSuccess)
        {
            TrackerDeviceEnumerator left_usb_dev_enumerator(cfg.left_camera_usb_path);

            if (left_usb_dev_enumerator.is_valid())
            {
                LeftTracker = ServerTrackerView::allocate_tracker_interface(&left_usb_dev_enumerator);

                if (LeftTracker->open(&left_usb_dev_enumerator))
                {
                    PSVR_LOG_INFO("VirtualStereoTracker::open") << 
                        "VirtualStereoTracker(" << cur_dev_path << 
                        ", camera_index=" << camera_index << 
                        ") open left camera at usb path: " << cfg.left_camera_usb_path;
                }
                else
                {
                    PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                        "VirtualStereoTracker(" << cur_dev_path << 
                        ", camera_index=" << camera_index << 
                        ") failed to open left camera at usb path: " << cfg.left_camera_usb_path;
                    bSuccess= false;
                }
            }
            else
            {
                PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                    "VirtualStereoTracker(" << cur_dev_path << 
                    ", camera_index=" << camera_index << 
                    ") has invalid left camera usb path: " << cfg.left_camera_usb_path;
                bSuccess= false;
            }
        }

        if (bSuccess)
        {
            TrackerDeviceEnumerator right_usb_dev_enumerator(cfg.right_camera_usb_path);

            if (right_usb_dev_enumerator.is_valid())
            {
                RightTracker = ServerTrackerView::allocate_tracker_interface(&right_usb_dev_enumerator);

                if (RightTracker->open(&right_usb_dev_enumerator))
                {
                    PSVR_LOG_INFO("VirtualStereoTracker::open") << 
                        "VirtualStereoTracker(" << cur_dev_path << 
                        ", camera_index=" << camera_index << 
                        ") open right camera at usb path: " << cfg.right_camera_usb_path;
                }
                else
                {
                    PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                        "VirtualStereoTracker(" << cur_dev_path << 
                        ", camera_index=" << camera_index << 
                        ") failed to open right camera at usb path: " << cfg.right_camera_usb_path;
                    bSuccess= false;
                }
            }
            else
            {
                PSVR_LOG_WARNING("VirtualStereoTracker::open") << 
                    "VirtualStereoTracker(" << cur_dev_path << 
                    ", camera_index=" << camera_index << 
                    ") has invalid right camera usb path: " << cfg.right_camera_usb_path;
                bSuccess= false;
            }
        }
    }
    
    if (bSuccess)
    {
        CaptureData = new VirtualStereoCaptureData;

        LeftTracker->setFrameWidth(cfg.tracker_intrinsics.pixel_width, false);
		LeftTracker->setVideoProperty(PSVRVideoProperty_Exposure, cfg.exposure, false);
		LeftTracker->setVideoProperty(PSVRVideoProperty_Gain, cfg.gain, false);
        LeftTracker->setFrameRate(cfg.frame_rate, false);
        RightTracker->setFrameWidth(cfg.tracker_intrinsics.pixel_width, false);
		RightTracker->setVideoProperty(PSVRVideoProperty_Exposure, cfg.exposure, false);
		RightTracker->setVideoProperty(PSVRVideoProperty_Gain, cfg.gain, false);
        RightTracker->setFrameRate(cfg.frame_rate, false);
    }
    else
    {
        close();
    }

    return bSuccess;
}

bool VirtualStereoTracker::getIsOpen() const
{
    return LeftTracker != nullptr && LeftTracker->getIsOpen() && 
            RightTracker != nullptr && RightTracker->getIsOpen();
}

void VirtualStereoTracker::close()
{
    if (CaptureData != nullptr)
    {
        delete CaptureData;
        CaptureData = nullptr;
    }

    if (LeftTracker != nullptr)
    {
        delete LeftTracker;
        LeftTracker = nullptr;
    }

    if (RightTracker != nullptr)
    {
        delete RightTracker;
        RightTracker = nullptr;
    }
}

CommonSensorState::eDeviceType VirtualStereoTracker::getDeviceType() const
{
    return CommonSensorState::VirtualStereoCamera;
}

ITrackerInterface::eDriverType VirtualStereoTracker::getDriverType() const
{
    //###bwalker $TODO Get the driver type from VideoCapture
    return DriverType;
}

std::string VirtualStereoTracker::getUSBDevicePath() const
{
    return device_identifier;
}

bool VirtualStereoTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
    return LeftTracker->getVideoFrameDimensions(out_width, out_height, out_stride);
}

bool VirtualStereoTracker::getIsVideoMirrored() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
    return LeftTracker->getIsVideoMirrored();
}

void VirtualStereoTracker::loadSettings()
{
    double leftFrameWidth= LeftTracker->getFrameWidth();
    double leftFrameFPS= LeftTracker->getFrameRate();
    int leftFrameExposure= LeftTracker->getVideoProperty(PSVRVideoProperty_Exposure);
    int leftFrameGain= LeftTracker->getVideoProperty(PSVRVideoProperty_Gain);

    double rightFrameWidth= RightTracker->getFrameWidth();
    double rightFrameFPS= RightTracker->getFrameRate();
    int rightFrameExposure= RightTracker->getVideoProperty(PSVRVideoProperty_Exposure);
    int rightFrameGain= RightTracker->getVideoProperty(PSVRVideoProperty_Gain);

    cfg.load();

    if (getIsOpen())
    {
	    if (leftFrameWidth != cfg.tracker_intrinsics.pixel_width)
	    {
		    LeftTracker->setFrameWidth(cfg.tracker_intrinsics.pixel_width, false);
	    }
	    if (rightFrameWidth != cfg.tracker_intrinsics.pixel_width)
	    {
		    RightTracker->setFrameWidth(cfg.tracker_intrinsics.pixel_width, false);
	    }

	    if (leftFrameExposure != cfg.exposure)
	    {
		    LeftTracker->setVideoProperty(PSVRVideoProperty_Exposure, cfg.exposure, false);
	    }
	    if (rightFrameExposure != cfg.exposure)
	    {
		    RightTracker->setVideoProperty(PSVRVideoProperty_Exposure, cfg.exposure, false);
	    }

	    if (leftFrameGain != cfg.gain)
	    {
		    LeftTracker->setVideoProperty(PSVRVideoProperty_Gain, cfg.gain, false);
	    }
	    if (rightFrameGain != cfg.gain)
	    {
		    RightTracker->setVideoProperty(PSVRVideoProperty_Gain, cfg.gain, false);
	    }

	    if (leftFrameFPS != cfg.frame_rate)
	    {
            LeftTracker->setFrameRate(cfg.frame_rate, false);
	    }
	    if (rightFrameFPS != cfg.frame_rate)
	    {
            RightTracker->setFrameRate(cfg.frame_rate, false);
	    }
    }
}

void VirtualStereoTracker::saveSettings()
{
    cfg.save();
}

void VirtualStereoTracker::setFrameWidth(double value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
	    LeftTracker->setFrameWidth(value, false);
        RightTracker->setFrameWidth(value, false);
    }

	if (bUpdateConfig)
	{
		cfg.tracker_intrinsics.pixel_width = static_cast<float>(value);
	}
}

double VirtualStereoTracker::getFrameWidth() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
    return LeftTracker->getFrameWidth();
}

void VirtualStereoTracker::setFrameHeight(double value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
	    LeftTracker->setFrameHeight(value, false);
        RightTracker->setFrameHeight(value, false);
    }

	if (bUpdateConfig)
	{
		cfg.tracker_intrinsics.pixel_height = static_cast<float>(value);
	}
}

double VirtualStereoTracker::getFrameHeight() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
    return LeftTracker->getFrameHeight();
}

void VirtualStereoTracker::setFrameRate(double value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
        LeftTracker->setFrameRate(value, false);
        RightTracker->setFrameRate(value, false);
    }

	if (bUpdateConfig)
	{
		cfg.frame_rate = value;
	}
}

double VirtualStereoTracker::getFrameRate() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
	return LeftTracker->getFrameRate();
}

bool VirtualStereoTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
{
	//ASSUMPTION: Left and right trackers should have same video frame property constraints
	return LeftTracker->getVideoPropertyConstraint(property_type, outConstraint);
}

void VirtualStereoTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
        LeftTracker->setVideoProperty(property_type, desired_value, false);
        RightTracker->setVideoProperty(property_type, desired_value, false);
    }

	if (bUpdateConfig)
	{
		switch (property_type)
		{
		case PSVRVideoProperty_Exposure:
			cfg.exposure = desired_value;
			break;
		case PSVRVideoProperty_Gain:
			cfg.gain = desired_value;
			break;
		}
	}
}

int VirtualStereoTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
{
	//ASSUMPTION: Left and right trackers should have same video frame properties
	return LeftTracker->getVideoProperty(property_type);
}

void VirtualStereoTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_STEREO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.stereo= cfg.tracker_intrinsics;
}

void VirtualStereoTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS);
    cfg.tracker_intrinsics= tracker_intrinsics.intrinsics.stereo;
}

PSVRPosef VirtualStereoTracker::getTrackerPose() const
{
    return cfg.pose;
}

void VirtualStereoTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    cfg.pose = *pose;
    cfg.save();
}

void VirtualStereoTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(cfg.tracker_intrinsics.hfov);
    outVFOV = static_cast<float>(cfg.tracker_intrinsics.vfov);
}

void VirtualStereoTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(cfg.tracker_intrinsics.znear);
    outZFar = static_cast<float>(cfg.tracker_intrinsics.zfar);
}

void VirtualStereoTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSVRClientTrackerSettings* settings) const
{
	settings->color_range_table = *cfg.getColorRangeTable(controller_serial);
}

void VirtualStereoTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
	PSVR_HSVColorRangeTable *table= cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    cfg.save();
}

void VirtualStereoTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
	const PSVR_HSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

void VirtualStereoTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}
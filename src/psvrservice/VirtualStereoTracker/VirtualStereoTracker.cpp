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
	, frame_rate(40)
    , exposure(32)
    , gain(32)
    , left_camera_usb_path("USB\\\\VID_1415&PID_2000\\\\bA_pB.C")
    , right_camera_usb_path("USB\\\\VID_1415&PID_2000\\\\bX_pY.Z")
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
    for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
    {
        SharedColorPresets.color_presets[preset_index] = k_default_color_presets[preset_index];
    }
};

const boost::property_tree::ptree
VirtualStereoTrackerConfig::config2ptree()
{
    boost::property_tree::ptree pt;

    pt.put("is_valid", is_valid);
    pt.put("version", VirtualStereoTrackerConfig::CONFIG_VERSION);
    pt.put("max_poll_failure_count", max_poll_failure_count);
	pt.put("frame_width", tracker_intrinsics.pixel_width);
	pt.put("frame_height", tracker_intrinsics.pixel_height);
	pt.put("frame_rate", frame_rate);
    pt.put("exposure", exposure);
	pt.put("gain", gain);
    pt.put("hfov", tracker_intrinsics.hfov);
    pt.put("vfov", tracker_intrinsics.vfov);
    pt.put("zNear", tracker_intrinsics.znear);
    pt.put("zFar", tracker_intrinsics.zfar);

    pt.put("pose.orientation.w", pose.Orientation.w);
    pt.put("pose.orientation.x", pose.Orientation.x);
    pt.put("pose.orientation.y", pose.Orientation.y);
    pt.put("pose.orientation.z", pose.Orientation.z);
    pt.put("pose.position.x", pose.PositionCm.x);
    pt.put("pose.position.y", pose.PositionCm.y);
    pt.put("pose.position.z", pose.PositionCm.z);

    pt.put("left_camera_usb_path", left_camera_usb_path);
    pt.put("right_camera_usb_path", right_camera_usb_path);

    writeArray(pt, "left_camera_matrix", tracker_intrinsics.left_camera_matrix);
    writeArray(pt, "right_camera_matrix", tracker_intrinsics.right_camera_matrix);

    writeDistortionCoefficients(pt, "left_distortion_cofficients", &tracker_intrinsics.left_distortion_coefficients);
    writeDistortionCoefficients(pt, "right_distortion_cofficients", &tracker_intrinsics.right_distortion_coefficients);

    writeArray(pt, "left_rectification_rotation", tracker_intrinsics.left_rectification_rotation);
    writeArray(pt, "right_rectification_rotation", tracker_intrinsics.right_rectification_rotation);

    writeArray(pt, "left_rectification_projection", tracker_intrinsics.left_rectification_projection);
    writeArray(pt, "right_rectification_projection", tracker_intrinsics.right_rectification_projection);

    writeArray(pt, "rotation_between_cameras", tracker_intrinsics.rotation_between_cameras);
    writeArray(pt, "translation_between_cameras", tracker_intrinsics.translation_between_cameras);
    writeArray(pt, "essential_matrix", tracker_intrinsics.essential_matrix);
    writeArray(pt, "fundamental_matrix", tracker_intrinsics.fundamental_matrix);
    writeArray(pt, "reprojection_matrix", tracker_intrinsics.reprojection_matrix);

	writeColorPropertyPresetTable(&SharedColorPresets, pt);

	for (auto &controller_preset_table : DeviceColorPresets)
	{
		writeColorPropertyPresetTable(&controller_preset_table, pt);
	}

    return pt;
}

void
VirtualStereoTrackerConfig::ptree2config(const boost::property_tree::ptree &pt)
{
    int config_version = pt.get<int>("version", 0);
    if (config_version == VirtualStereoTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get<bool>("is_valid", false);
        max_poll_failure_count = pt.get<long>("max_poll_failure_count", 100);
		frame_rate = pt.get<double>("frame_rate", 40);
        exposure = pt.get<double>("exposure", 32);
		gain = pt.get<double>("gain", 32);

        left_camera_usb_path= pt.get<std::string>("left_camera_usb_path");
        right_camera_usb_path= pt.get<std::string>("right_camera_usb_path");

		tracker_intrinsics.pixel_width = pt.get<float>("frame_width", 640.f);
		tracker_intrinsics.pixel_height = pt.get<float>("frame_height", 480.f);
        tracker_intrinsics.hfov = pt.get<float>("hfov", 60.f);
        tracker_intrinsics.vfov = pt.get<float>("vfov", 45.f);
        tracker_intrinsics.znear = pt.get<float>("zNear", 10.f);
        tracker_intrinsics.zfar = pt.get<float>("zFar", 200.f);

        tracker_intrinsics.left_camera_matrix= readArray<double,3*3>(pt, "left_camera_matrix");
        tracker_intrinsics.right_camera_matrix= readArray<double,3*3>(pt, "right_camera_matrix");

        readDistortionCoefficients(pt, "left_distortion_cofficients", 
            &tracker_intrinsics.left_distortion_coefficients, 
            &tracker_intrinsics.left_distortion_coefficients);
        readDistortionCoefficients(pt, "right_distortion_cofficients", 
            &tracker_intrinsics.right_distortion_coefficients, 
            &tracker_intrinsics.right_distortion_coefficients);

        tracker_intrinsics.left_rectification_rotation= readArray<double,3*3>(pt, "left_rectification_rotation");
        tracker_intrinsics.right_rectification_rotation= readArray<double,3*3>(pt, "right_rectification_rotation");

        tracker_intrinsics.left_rectification_projection= readArray<double,3*4>(pt, "left_rectification_projection");
        tracker_intrinsics.right_rectification_projection= readArray<double,3*4>(pt, "right_rectification_projection");

        tracker_intrinsics.rotation_between_cameras= readArray<double,3*3>(pt, "rotation_between_cameras");
        tracker_intrinsics.translation_between_cameras= readArray<double,3>(pt, "translation_between_cameras");
        tracker_intrinsics.essential_matrix= readArray<double,3*3>(pt, "essential_matrix");
        tracker_intrinsics.fundamental_matrix= readArray<double,3*3>(pt, "fundamental_matrix");
        tracker_intrinsics.reprojection_matrix= readArray<double,4*4>(pt, "reprojection_matrix");

        pose.Orientation.w = pt.get<float>("pose.orientation.w", 1.0);
        pose.Orientation.x = pt.get<float>("pose.orientation.x", 0.0);
        pose.Orientation.y = pt.get<float>("pose.orientation.y", 0.0);
        pose.Orientation.z = pt.get<float>("pose.orientation.z", 0.0);
        pose.PositionCm.x = pt.get<float>("pose.position.x", 0.0);
        pose.PositionCm.y = pt.get<float>("pose.position.y", 0.0);
        pose.PositionCm.z = pt.get<float>("pose.position.z", 0.0);

		// Read the default preset table
		readColorPropertyPresetTable(pt, &SharedColorPresets);

		// Read all of the controller preset tables
		const std::string controller_prefix("controller_");
		const std::string hmd_prefix("hmd_");
		for(auto iter = pt.begin(); iter != pt.end(); iter++)
		{
			const std::string &entry_name= iter->first;
			
			if (entry_name.compare(0, controller_prefix.length(), controller_prefix) == 0 ||
				entry_name.compare(0, hmd_prefix.length(), hmd_prefix) == 0)
			{
				CommonHSVColorRangeTable table;

				table.table_name= entry_name;
				for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
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

const CommonHSVColorRangeTable *
VirtualStereoTrackerConfig::getColorRangeTable(const std::string &table_name) const
{
	const CommonHSVColorRangeTable *table= &SharedColorPresets;	

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

inline CommonHSVColorRangeTable *
VirtualStereoTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
{
	CommonHSVColorRangeTable *table= nullptr;	

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
			CommonHSVColorRangeTable Table;

			Table.table_name= table_name;
			for (int preset_index = 0; preset_index < eCommonTrackingColorID::MAX_TRACKING_COLOR_TYPES; ++preset_index)
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
    , NextPollSequenceNumber(0)
    , TrackerStates()
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
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_VIRTUAL_STEREO, CommonControllerState::VirtualStereoCamera);
    bool success = false;

    // Skip over everything that isn't a PS3EYE
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonDeviceState::VirtualStereoCamera)
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

    if (pEnum->get_device_type() == CommonControllerState::VirtualStereoCamera)
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
        LeftTracker->setExposure(cfg.exposure, false);
        LeftTracker->setGain(cfg.gain, false);
        LeftTracker->setFrameRate(cfg.frame_rate, false);
        RightTracker->setFrameWidth(cfg.tracker_intrinsics.pixel_height, false);
        RightTracker->setExposure(cfg.exposure, false);
        RightTracker->setGain(cfg.gain, false);
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

bool VirtualStereoTracker::getIsReadyToPoll() const
{
    return getIsOpen();
}

IDeviceInterface::ePollResult VirtualStereoTracker::poll()
{
    IDeviceInterface::ePollResult result = IDeviceInterface::_PollResultFailure;

    if (getIsOpen())
    {        
        if (!LeftTracker->poll() || !RightTracker->poll())
        {
            // Device still in valid state
            result = IControllerInterface::_PollResultSuccessNoData;
        }
        else
        {
            // New data available. Keep iterating.
            result = IControllerInterface::_PollResultSuccessNewData;
        }

        {
            VirtualStereoTrackerState newState;

            // TODO: Process the frame and extract the blobs

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = NextPollSequenceNumber;
            ++NextPollSequenceNumber;

            // Make room for new entry if at the max queue size
            //###bwalker $TODO Make this a fixed size circular buffer
            if (TrackerStates.size() >= VIRTUAL_STEREO_STATE_BUFFER_MAX)
            {
                TrackerStates.erase(TrackerStates.begin(), TrackerStates.begin() + TrackerStates.size() - VIRTUAL_STEREO_STATE_BUFFER_MAX);
            }

            TrackerStates.push_back(newState);
        }
    }

    return result;
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

long VirtualStereoTracker::getMaxPollFailureCount() const
{
    return cfg.max_poll_failure_count;
}

CommonDeviceState::eDeviceType VirtualStereoTracker::getDeviceType() const
{
    return CommonDeviceState::VirtualStereoCamera;
}

const CommonDeviceState *VirtualStereoTracker::getState(int lookBack) const
{
    const int queueSize = static_cast<int>(TrackerStates.size());
    const CommonDeviceState * result =
        (lookBack < queueSize) ? &TrackerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
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

const unsigned char *VirtualStereoTracker::getVideoFrameBuffer(ITrackerInterface::eTrackerVideoSection section) const
{
    const unsigned char *result = nullptr;

    if (getIsOpen())
    {
        switch (section)
        {
        case ITrackerInterface::LeftSection:
            result= LeftTracker->getVideoFrameBuffer(ITrackerInterface::PrimarySection);
            break;
        case ITrackerInterface::RightSection:
            result= RightTracker->getVideoFrameBuffer(ITrackerInterface::PrimarySection);
            break;
        }
    }

    return result;
}

void VirtualStereoTracker::loadSettings()
{
    double leftFrameWidth= LeftTracker->getFrameWidth();
    double leftFrameFPS= LeftTracker->getFrameRate();
    double leftFrameExposure= LeftTracker->getExposure();
    double leftFrameGain= LeftTracker->getGain();

    double rightFrameWidth= RightTracker->getFrameWidth();
    double rightFrameFPS= RightTracker->getFrameRate();
    double rightFrameExposure= RightTracker->getExposure();
    double rightFrameGain= RightTracker->getGain();

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
		    LeftTracker->setExposure(cfg.exposure, false);
	    }
	    if (rightFrameExposure != cfg.exposure)
	    {
		    RightTracker->setExposure(cfg.exposure, false);
	    }

	    if (leftFrameGain != cfg.gain)
	    {
		    LeftTracker->setGain(cfg.gain, false);
	    }
	    if (rightFrameGain != cfg.gain)
	    {
		    RightTracker->setGain(cfg.gain, false);
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

void VirtualStereoTracker::setExposure(double value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
        LeftTracker->setExposure(value, false);
        RightTracker->setExposure(value, false);
    }

	if (bUpdateConfig)
	{
		cfg.exposure = value;
	}
}

double VirtualStereoTracker::getExposure() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
    return LeftTracker->getExposure();
}

void VirtualStereoTracker::setGain(double value, bool bUpdateConfig)
{
    if (getIsOpen())
    {
        LeftTracker->setGain(value, false);
        RightTracker->setGain(value, false);
    }

	if (bUpdateConfig)
	{
		cfg.gain = value;
	}
}

double VirtualStereoTracker::getGain() const
{
    //ASSUMPTION: Left and right trackers should have same video frame properties
	return cfg.gain;
}

void VirtualStereoTracker::getCameraIntrinsics(
    CommonTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= CommonTrackerIntrinsics::STEREO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.stereo_intrinsics= cfg.tracker_intrinsics;
}

void VirtualStereoTracker::setCameraIntrinsics(
    const CommonTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == CommonTrackerIntrinsics::STEREO_TRACKER_INTRINSICS);
    cfg.tracker_intrinsics= tracker_intrinsics.stereo_intrinsics;
}

CommonDevicePose VirtualStereoTracker::getTrackerPose() const
{
    return cfg.pose;
}

void VirtualStereoTracker::setTrackerPose(
    const struct CommonDevicePose *pose)
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

void VirtualStereoTracker::gatherTrackerOptions(
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
}

bool VirtualStereoTracker::setOptionIndex(
    const std::string &option_name,
    int option_index)
{
    return false;
}

bool VirtualStereoTracker::getOptionIndex(
    const std::string &option_name, 
    int &out_option_index) const
{
    return false;
}

void VirtualStereoTracker::gatherTrackingColorPresets(
	const std::string &controller_serial, 
    PSMoveProtocol::Response_ResultTrackerSettings* settings) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    for (int list_index = 0; list_index < MAX_TRACKING_COLOR_TYPES; ++list_index)
    {
        const CommonHSVColorRange &hsvRange = table->color_presets[list_index];
        const eCommonTrackingColorID colorType = static_cast<eCommonTrackingColorID>(list_index);

        PSMoveProtocol::TrackingColorPreset *colorPreset= settings->add_color_presets();
        colorPreset->set_color_type(static_cast<PSMoveProtocol::TrackingColorType>(colorType));
        colorPreset->set_hue_center(hsvRange.hue_range.center);
        colorPreset->set_hue_range(hsvRange.hue_range.range);
        colorPreset->set_saturation_center(hsvRange.saturation_range.center);
        colorPreset->set_saturation_range(hsvRange.saturation_range.range);
        colorPreset->set_value_center(hsvRange.value_range.center);
        colorPreset->set_value_range(hsvRange.value_range.range);
    }
}

void VirtualStereoTracker::setTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    const CommonHSVColorRange *preset)
{
	CommonHSVColorRangeTable *table= cfg.getOrAddColorRangeTable(controller_serial);

    table->color_presets[color] = *preset;
    cfg.save();
}

void VirtualStereoTracker::getTrackingColorPreset(
	const std::string &controller_serial, 
    eCommonTrackingColorID color, 
    CommonHSVColorRange *out_preset) const
{
	const CommonHSVColorRangeTable *table= cfg.getColorRangeTable(controller_serial);

    *out_preset = table->color_presets[color];
}

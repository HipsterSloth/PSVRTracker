// -- includes -----
#include "PS4CameraTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "TrackerDeviceEnumerator.h"
#include "TrackerManager.h"
#include "USBDeviceManager.h"
#include "opencv2/opencv.hpp"

#include "ps4eye.h"

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- constants -----
#define FIRMWARE_UPLOAD_CHUNK_SIZE      512
#define FIRMWARE_UPLOAD_DEVICE_PID      0x0580
#define FIRMWARE_UPLOAD_DEVICE_VID      0x05a9

#define PS4CAMERA_STATE_BUFFER_MAX      16
#define DEFAULT_SECTION_WIDTH           1280
#define DEFAULT_SECTION_HEIGHT          800
#define DEFAULT_FRAME_RATE              60

// -- private definitions -----
class PS4CameraCaptureData
{
public:
    PS4CameraCaptureData()
    {

    }

    void processRawFrame()
    {
        assert(m_raw.channels() == 2);
        cv::cvtColor(m_raw, m_frame, cv::COLOR_YUV2BGR_YUY2);

        int height = m_frame.rows - MARGIN_BOTTOM;
        int width = (height * 16) / 10;
        assert(width == 1280 || width == 640 || width == 320);

        int x = MARGIN_LEFT;
        int y = 0;
        for (int i = 0; i < LEVELS; i++)
        {
            for (int j = 0; j < EYES; j++)
            {
                // Calculate the region of this level in the frame.
                cv::Rect level(x, y, width >> i, height << i);

                if (i > 0)
                {
                    // Clone the part of the frame that contains the pixels for this level.
                    // This is necessary because the rows are cut to pieces and interleaved,
                    // so we can't simply use differing strides for each level.
                    m_levels[i][j] = m_frame(level).clone();
                }
                else
                {
                    // The first level doesn't need to be cloned, since the rows aren't interleaved.
                    m_levels[i][j] = m_frame(level);
                }

                // The copy has merged the rows in memory again, so now we can simply
                // manipulate the stride and set the actual size.
                m_levels[i][j].step[0] <<= i;
                m_levels[i][j].cols = width;
                m_levels[i][j].rows = height;

                // Move to the next image
                x += level.width;
            }

            // Calculate the header margin
            y |= 1 << i;

            // Just like mip-maps every level is half the size of the previous level.
            width >>= 1;
            height >>= 1;
        }
    }

    cv::Mat& getRawMutable()
    { 
        return m_raw; 
    }

    cv::Mat getImage(PSVRVideoFrameSection section= PSVRVideoFrameSection_Left, int level = 0)
    {
        if (level >= LEVELS)
            return cv::Mat();

        // Don't worry, this is not a deep-copy
        return m_levels[level][section];
    }

    inline int getImageWidth(PSVRVideoFrameSection section = PSVRVideoFrameSection_Left, int level = 0) const
    {
        return  (level < LEVELS) ? m_levels[level][section].cols : 0;
    }

    inline int getImageHeight(PSVRVideoFrameSection section = PSVRVideoFrameSection_Right, int level = 0) const
    {
        return  (level < LEVELS) ? m_levels[level][section].rows : 0;
    }

    inline cv::VideoCaptureModes getImageFormat() const
    {
        return cv::CAP_MODE_BGR;
    }

private:
    static const int MARGIN_LEFT = 48;
    static const int MARGIN_BOTTOM = 8;
    static const int LEVELS = 4;
    static const int EYES = 2;

    cv::Mat m_raw;
    cv::Mat m_frame;
    cv::Mat m_levels[LEVELS][EYES];
};

// -- public methods
// -- PS3EYE Controller Config
const int PS4CameraTrackerConfig::CONFIG_VERSION = 1;
const int PS4CameraTrackerConfig::LENS_CALIBRATION_VERSION= 1;

PS4CameraTrackerConfig::PS4CameraTrackerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
    , frame_rate(DEFAULT_FRAME_RATE)
    , exposure(32)
    , gain(32)
{
    trackerIntrinsics.pixel_width= DEFAULT_SECTION_WIDTH;
    trackerIntrinsics.pixel_height= DEFAULT_SECTION_HEIGHT;
    trackerIntrinsics.hfov= 60.0; // degrees
    trackerIntrinsics.vfov= 45.0; // degrees
    trackerIntrinsics.znear= 10.0; // cm
    trackerIntrinsics.zfar= 200.0; // cm
    trackerIntrinsics.left_camera_matrix= {{ 
        554.2563, 0, 320.0, // f_x, 0, c_x
        0, 554.2563, 240.0, // 0, f_y, c_y
        0, 0, 1}};
    trackerIntrinsics.right_camera_matrix= {{
        554.2563, 0, 320.0,  // f_x, 0, c_x
        0, 554.2563, 240.0,  // 0, f_y, c_y
        0, 0, 1}};
    trackerIntrinsics.left_distortion_coefficients= {
        -0.10771770030260086, 0.1213262677192688, 0.04875476285815239, // K1, K2, K3
        0.00091733073350042105, 0.00010589254816295579};  // P1, P2
    trackerIntrinsics.right_distortion_coefficients= {
        -0.10771770030260086, 0.1213262677192688, 0.04875476285815239, // K1, K2, K3
        0.00091733073350042105, 0.00010589254816295579};  // P1, P2
    trackerIntrinsics.left_rectification_rotation= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    trackerIntrinsics.right_rectification_rotation= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    trackerIntrinsics.left_rectification_projection= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,}};
    trackerIntrinsics.right_rectification_projection= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,}};
    trackerIntrinsics.rotation_between_cameras= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    trackerIntrinsics.translation_between_cameras= {0.0, 0.0, 0.0};
    trackerIntrinsics.essential_matrix= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    trackerIntrinsics.fundamental_matrix= {{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0}};
    trackerIntrinsics.reprojection_matrix= {{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0}};
    
    pose = *k_PSVR_pose_identity;
    memset(&SharedColorPresets, 0, sizeof(PSVR_HSVColorRangeTable));
};

const configuru::Config
PS4CameraTrackerConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"version", PS4CameraTrackerConfig::CONFIG_VERSION},
        {"lens_calibration_version", PS4CameraTrackerConfig::LENS_CALIBRATION_VERSION},
        {"max_poll_failure_count", max_poll_failure_count},
        {"frame_rate", frame_rate},
        {"exposure", exposure},
        {"gain", gain},
        {"hfov", trackerIntrinsics.hfov},
        {"vfov", trackerIntrinsics.vfov},
        {"zNear", trackerIntrinsics.znear},
        {"zFar", trackerIntrinsics.zfar},
        {"frame_width", trackerIntrinsics.pixel_width},
        {"frame_height", trackerIntrinsics.pixel_height},
        {"pose.orientation.w", pose.Orientation.w},
        {"pose.orientation.x", pose.Orientation.x},
        {"pose.orientation.y", pose.Orientation.y},
        {"pose.orientation.z", pose.Orientation.z},
        {"pose.position.x", pose.Position.x},
        {"pose.position.y", pose.Position.y},
        {"pose.position.z", pose.Position.z}
    };

    writeMatrix3d(pt, "left_camera_matrix", trackerIntrinsics.left_camera_matrix);
    writeMatrix3d(pt, "right_camera_matrix", trackerIntrinsics.right_camera_matrix);

    writeDistortionCoefficients(pt, "left_distortion_cofficients", &trackerIntrinsics.left_distortion_coefficients);
    writeDistortionCoefficients(pt, "right_distortion_cofficients", &trackerIntrinsics.right_distortion_coefficients);

    writeMatrix3d(pt, "left_rectification_rotation", trackerIntrinsics.left_rectification_rotation);
    writeMatrix3d(pt, "right_rectification_rotation", trackerIntrinsics.right_rectification_rotation);

    writeMatrix34d(pt, "left_rectification_projection", trackerIntrinsics.left_rectification_projection);
    writeMatrix34d(pt, "right_rectification_projection", trackerIntrinsics.right_rectification_projection);

    writeMatrix3d(pt, "rotation_between_cameras", trackerIntrinsics.rotation_between_cameras);
    writeVector3d(pt, "translation_between_cameras", trackerIntrinsics.translation_between_cameras);
    writeMatrix3d(pt, "essential_matrix", trackerIntrinsics.essential_matrix);
    writeMatrix3d(pt, "fundamental_matrix", trackerIntrinsics.fundamental_matrix);
    writeMatrix4d(pt, "reprojection_matrix", trackerIntrinsics.reprojection_matrix);

    writeColorPropertyPresetTable(&SharedColorPresets, pt);

    for (auto &controller_preset_table : DeviceColorPresets)
    {
        writeColorPropertyPresetTable(&controller_preset_table, pt);
    }

    return pt;
}

void
PS4CameraTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == PS4CameraTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get_or<bool>("is_valid", false);
        max_poll_failure_count = pt.get_or<long>("max_poll_failure_count", 100);
        frame_rate = pt.get_or<double>("frame_rate", DEFAULT_FRAME_RATE);
        exposure = (int)pt.get_or<float>("exposure", 32);
		gain = (int)pt.get_or<float>("gain", 32);

        int lens_calibration_version = pt.get_or<int>("lens_calibration_version", 0);
        if (lens_calibration_version == PS4CameraTrackerConfig::LENS_CALIBRATION_VERSION)
        {
            trackerIntrinsics.hfov = pt.get_or<float>("hfov", 60.0f);
            trackerIntrinsics.vfov = pt.get_or<float>("vfov", 45.0f);
            trackerIntrinsics.znear = pt.get_or<float>("zNear", 10.0f);
            trackerIntrinsics.zfar = pt.get_or<float>("zFar", 200.0f);

            //###HipsterSloth $TODO For now we stick with the default frame size
            //trackerIntrinsics.pixel_width = pt.get_or<float>("frame_width", DEFAULT_SECTION_WIDTH);
            //trackerIntrinsics.pixel_height = pt.get_or<float>("frame_height", DEFAULT_SECTION_HEIGHT);
            trackerIntrinsics.pixel_width = DEFAULT_SECTION_WIDTH;
            trackerIntrinsics.pixel_height = DEFAULT_SECTION_HEIGHT;

            readMatrix3d(pt, "left_camera_matrix", trackerIntrinsics.left_camera_matrix);
            readMatrix3d(pt, "right_camera_matrix", trackerIntrinsics.right_camera_matrix);

            readDistortionCoefficients(pt, "left_distortion_cofficients", 
                &trackerIntrinsics.left_distortion_coefficients, 
                &trackerIntrinsics.left_distortion_coefficients);
            readDistortionCoefficients(pt, "right_distortion_cofficients", 
                &trackerIntrinsics.right_distortion_coefficients, 
                &trackerIntrinsics.right_distortion_coefficients);

            readMatrix3d(pt, "left_rectification_rotation", trackerIntrinsics.left_rectification_rotation);
            readMatrix3d(pt, "right_rectification_rotation", trackerIntrinsics.right_rectification_rotation);

            readMatrix34d(pt, "left_rectification_projection", trackerIntrinsics.left_rectification_projection);
            readMatrix34d(pt, "right_rectification_projection", trackerIntrinsics.right_rectification_projection);

            readMatrix3d(pt, "rotation_between_cameras", trackerIntrinsics.rotation_between_cameras);
            readVector3d(pt, "translation_between_cameras", trackerIntrinsics.translation_between_cameras);
            readMatrix3d(pt, "essential_matrix", trackerIntrinsics.essential_matrix);
            readMatrix3d(pt, "fundamental_matrix", trackerIntrinsics.fundamental_matrix);
            readMatrix4d(pt, "reprojection_matrix", trackerIntrinsics.reprojection_matrix);
        }
        else
        {
            PSVR_LOG_WARNING("PS4CameraTrackerConfig") <<
                "Config version " << lens_calibration_version << " does not match expected version " <<
                PS4CameraTrackerConfig::LENS_CALIBRATION_VERSION << ", Using defaults.";
        }

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
        PSVR_LOG_WARNING("PS4CameraTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            PS4CameraTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const PSVR_HSVColorRangeTable *
PS4CameraTrackerConfig::getColorRangeTable(const std::string &table_name) const
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
PS4CameraTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
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
PS4CameraTracker::PS4CameraTracker()
    : cfg()
    , USBDevicePath()
    , VideoCapture(nullptr)
    , CaptureData(nullptr)
    , NextPollSequenceNumber(0)
{
}

PS4CameraTracker::~PS4CameraTracker()
{
    if (getIsOpen())
    {
        PSVR_LOG_ERROR("~PS4CameraTracker") << "Tracker deleted without calling close() first!";
    }
}

// PSVRTracker
bool PS4CameraTracker::open() // Opens the first HID device for the tracker
{
    TrackerDeviceEnumerator enumerator(TrackerDeviceEnumerator::CommunicationType_USB, CommonSensorState::PS4Camera);
    bool success = false;

    // Skip over everything that isn't a PS3EYE
    while (enumerator.is_valid() && enumerator.get_device_type() != CommonSensorState::PS4Camera)
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
bool PS4CameraTracker::matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const
{
    // Down-cast the enumerator so we can use the correct get_path.
    const TrackerDeviceEnumerator *pEnum = static_cast<const TrackerDeviceEnumerator *>(enumerator);

    bool matches = false;

    if (pEnum->get_device_type() == CommonSensorState::PS4Camera)
    {
        std::string enumerator_path = pEnum->get_path();

        matches = (enumerator_path == USBDevicePath);
    }

    return matches;
}

bool PS4CameraTracker::open(const DeviceEnumerator *enumerator)
{
    const TrackerDeviceEnumerator *tracker_enumerator = static_cast<const TrackerDeviceEnumerator *>(enumerator);
    const char *cur_dev_path = tracker_enumerator->get_path();
    const int camera_index = tracker_enumerator->get_camera_index();

    bool bSuccess = false;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("PS4CameraTracker::open") << "PS4CameraTracker(" << cur_dev_path << ") already open. Ignoring request.";
        bSuccess = true;
    }
    else
    {
        PSVR_LOG_INFO("PS4CameraTracker::open") << "Opening PS4CameraTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

        VideoCapture = new cv::VideoCapture(camera_index + cv::CAP_MSMF);

        if (VideoCapture->isOpened())
        {
            CaptureData = new PS4CameraCaptureData;
            USBDevicePath = enumerator->get_path();
            bSuccess = true;
        }
        else
        {
            PSVR_LOG_ERROR("PS4CameraTracker::open") << "Failed to open PS4CameraTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

            close();
        }
    }
    
    if (bSuccess)
    {
        char config_name[256];
        Utility::format_string(config_name, sizeof(config_name), "PS4CameraTrackerConfig_MSMF_%d", camera_index);

        cfg = PS4CameraTrackerConfig(config_name);

        // Load the ps3eye config
        cfg.load();
        // Save the config back out again in case defaults changed
        cfg.save();

        //TODO: Frame width? Frame height? FPS?
        VideoCapture->set(cv::CAP_PROP_EXPOSURE, (double)cfg.exposure);
        VideoCapture->set(cv::CAP_PROP_GAIN, (double)cfg.gain);
    }

    // Process one frame to retrieve to initialize the capture data
    if (VideoCapture->grab() &&
        VideoCapture->retrieve(CaptureData->getRawMutable()))
    {
        CaptureData->processRawFrame();
    }
    else
    {
        PSVR_LOG_ERROR("PS4CameraTracker::open") << "Failed to grab initial frame for PS4CameraTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";
        close();
        bSuccess= false;
    }

    return bSuccess;
}

bool PS4CameraTracker::getIsOpen() const
{
    return VideoCapture != nullptr;
}

void PS4CameraTracker::close()
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

CommonSensorState::eDeviceType PS4CameraTracker::getDeviceType() const
{
    return CommonSensorState::PS4Camera;
}

ITrackerInterface::eDriverType PS4CameraTracker::getDriverType() const
{
    return ITrackerInterface::Libusb;
}

std::string PS4CameraTracker::getUSBDevicePath() const
{
    return USBDevicePath;
}

//###HipsterSloth $TODO Support section and level inputs
bool PS4CameraTracker::getVideoFrameDimensions(
    int *out_width,
    int *out_height,
    int *out_stride) const
{
    bool bSuccess = true;

    if (out_width != nullptr)
    {
        int width = CaptureData->getImageWidth();

        if (out_stride != nullptr)
        {
            int format = static_cast<int>(CaptureData->getImageFormat());
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
                // Assume BGR?
                PSVR_LOG_ERROR("PS4CameraTracker::getVideoFrameDimensions") << "Unknown video format for camera" << USBDevicePath << ")";
                bytes_per_pixel = 3;
            }

            *out_stride = bytes_per_pixel * width;
        }

        *out_width = width;
    }

    if (out_height != nullptr)
    {
        int height = static_cast<int>(CaptureData->getImageHeight());

        *out_height = height;
    }

    return bSuccess;
}

void PS4CameraTracker::loadSettings()
{
    const double currentFrameWidth = CaptureData->getImageWidth();
    const double currentFrameRate = VideoCapture->get(cv::CAP_PROP_FPS);
    const int currentExposure= VideoCapture->get(cv::CAP_PROP_EXPOSURE);
    const int currentGain= VideoCapture->get(cv::CAP_PROP_GAIN);

    cfg.load();

    //###HipsterSloth $TODO For now we stick with the default frame size
    //if (currentFrameWidth != cfg.trackerIntrinsics.pixel_width)
    //{
    //	VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, cfg.trackerIntrinsics.pixel_width);
    //}

    if (currentExposure != cfg.exposure)
    {
        VideoCapture->set(cv::CAP_PROP_EXPOSURE, (double)cfg.exposure);
    }

    if (currentGain != cfg.gain)
    {
        VideoCapture->set(cv::CAP_PROP_GAIN, (double)cfg.gain);
    }

    if (currentFrameRate != cfg.frame_rate)
    {
        VideoCapture->set(cv::CAP_PROP_FPS, cfg.frame_rate);
    }
}

void PS4CameraTracker::saveSettings()
{
    cfg.save();
}

void PS4CameraTracker::setFrameWidth(double value, bool bUpdateConfig)
{
    //###HipsterSloth $TODO For now we stick with the default frame size
    //if (getFrameWidth() != value)
    //{
       // VideoCapture->set(cv::CAP_PROP_FRAME_WIDTH, value);

       // if (bUpdateConfig)
       // {
          //  cfg.trackerIntrinsics.pixel_width = static_cast<float>(value);
       // }
    //}
}

double PS4CameraTracker::getFrameWidth() const
{
    return CaptureData->getImageWidth();
}

void PS4CameraTracker::setFrameHeight(double value, bool bUpdateConfig)
{
    //###HipsterSloth $TODO For now we stick with the default frame size
    //if (getFrameHeight() != value)
    //{
       // VideoCapture->set(cv::CAP_PROP_FRAME_HEIGHT, value);

       // if (bUpdateConfig)
       // {
          //  cfg.trackerIntrinsics.pixel_height = static_cast<float>(value);
       // }
    //}
}

double PS4CameraTracker::getFrameHeight() const
{
    return CaptureData->getImageHeight();
}

void PS4CameraTracker::setFrameRate(double value, bool bUpdateConfig)
{
    if (getFrameRate() != value)
    {
        VideoCapture->set(cv::CAP_PROP_FPS, value);

        if (bUpdateConfig)
        {
            cfg.frame_rate = value;
        }
    }
}

double PS4CameraTracker::getFrameRate() const
{
    return VideoCapture->get(cv::CAP_PROP_FPS);
}

bool PS4CameraTracker::getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const
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

void PS4CameraTracker::setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool bUpdateConfig)
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

int PS4CameraTracker::getVideoProperty(const PSVRVideoPropertyType property_type) const
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

void PS4CameraTracker::getCameraIntrinsics(
    PSVRTrackerIntrinsics &out_tracker_intrinsics) const
{
    out_tracker_intrinsics.intrinsics_type= PSVR_STEREO_TRACKER_INTRINSICS;
    out_tracker_intrinsics.intrinsics.stereo= cfg.trackerIntrinsics;
}

void PS4CameraTracker::setCameraIntrinsics(
    const PSVRTrackerIntrinsics &tracker_intrinsics)
{
    assert(tracker_intrinsics.intrinsics_type == PSVR_STEREO_TRACKER_INTRINSICS);
    cfg.trackerIntrinsics = tracker_intrinsics.intrinsics.stereo;
}

PSVRPosef PS4CameraTracker::getTrackerPose() const
{
    return cfg.pose;
}

void PS4CameraTracker::setTrackerPose(
    const PSVRPosef *pose)
{
    cfg.pose = *pose;
    cfg.save();
}

void PS4CameraTracker::getFOV(float &outHFOV, float &outVFOV) const
{
    outHFOV = static_cast<float>(cfg.trackerIntrinsics.hfov);
    outVFOV = static_cast<float>(cfg.trackerIntrinsics.vfov);
}

void PS4CameraTracker::getZRange(float &outZNear, float &outZFar) const
{
    outZNear = static_cast<float>(cfg.trackerIntrinsics.znear);
    outZFar = static_cast<float>(cfg.trackerIntrinsics.zfar);
}

void PS4CameraTracker::gatherTrackingColorPresets(
    const std::string &table_name, 
    PSVRClientTrackerSettings* settings) const
{
    settings->color_range_table= *cfg.getColorRangeTable(table_name);
}

void PS4CameraTracker::setTrackingColorPreset(
    const std::string &table_name, 
    PSVRTrackingColorType color, 
    const PSVR_HSVColorRange *preset)
{
//    cfg.ColorPresets[color] = *preset; // from generic_camera conflict
    PSVR_HSVColorRangeTable *table= cfg.getOrAddColorRangeTable(table_name);

    table->color_presets[color] = *preset;
    cfg.save();
}

void PS4CameraTracker::getTrackingColorPreset(
    const std::string &table_name, 
    PSVRTrackingColorType color, 
    PSVR_HSVColorRange *out_preset) const
{
    const PSVR_HSVColorRangeTable *table= cfg.getColorRangeTable(table_name);

    *out_preset = table->color_presets[color];
}

void PS4CameraTracker::setTrackerListener(ITrackerListener *listener)
{
	m_listener= listener;
}

void PS4CameraTracker::uploadFirmwareToAllPS4Cameras(const std::string &firmware_path)
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
                    uint32_t length = firmware.tellg();
                    firmware.seekg(0, std::ios::beg);

                    uint16_t index = 0x14;
                    uint16_t value = 0;

                    for (uint32_t pos = 0; pos < length; pos += FIRMWARE_UPLOAD_CHUNK_SIZE)
                    {
                        uint16_t size = (FIRMWARE_UPLOAD_CHUNK_SIZE > (length - pos)) ? (length - pos) : FIRMWARE_UPLOAD_CHUNK_SIZE;

                        firmware.read((char*)chunk, size);

                        USBTransferRequest request;
                        request.payload.control_transfer.usb_device_handle= dev_handle;
                        request.payload.control_transfer.bmRequestType= 0x40;
                        request.payload.control_transfer.bRequest= 0x0;
                        request.payload.control_transfer.wValue= value;
                        request.payload.control_transfer.wIndex= index;
                        request.payload.control_transfer.wLength= size;
                        request.payload.control_transfer.timeout= 1000;
                        assert(size <= sizeof(request.payload.control_transfer.data)); 
                        memcpy(request.payload.control_transfer.data, chunk, size);
                        request.request_type= _USBRequestType_ControlTransfer;
                        usb_device_submit_transfer_request_blocking(request);

                        if (((uint32_t)value + size) > 0xFFFF)
                        {
                            index += 1;
                        }

                        value += size;
                    }
                    firmware.close();

                    USBTransferRequest request;
                    request.payload.control_transfer.usb_device_handle= dev_handle;
                    request.payload.control_transfer.bmRequestType= 0x40;
                    request.payload.control_transfer.bRequest= 0x0;
                    request.payload.control_transfer.wValue= 0x2200;
                    request.payload.control_transfer.wIndex= 0x8018;
                    request.payload.control_transfer.wLength= 1;
                    request.payload.control_transfer.timeout= 1000;
                    request.payload.control_transfer.data[0]= 0x5b;
                    request.request_type= _USBRequestType_ControlTransfer;
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
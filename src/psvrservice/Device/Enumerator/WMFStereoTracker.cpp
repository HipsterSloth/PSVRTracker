// -- includes -----
#include "WMFStereoTracker.h"
#include "Logger.h"
#include "Utility.h"
#include "PSEyeVideoCapture.h"
#include "ServerTrackerView.h"
#include "TrackerDeviceEnumerator.h"
#include "WMFCameraEnumerator.h"
#include "TrackerManager.h"

#include <Mfidl.h>
#include <Mfapi.h>
#include <Mferror.h>
#include <Strmif.h>
#include <Shlwapi.h>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#ifdef _MSC_VER
    #pragma warning (disable: 4996) // 'This function or variable may be unsafe': strncpy
#endif

// -- constants -----
#define VIRTUAL_STEREO_STATE_BUFFER_MAX		16

// -- private definitions -----
class WMFStereoCaptureData
{
public:
    WMFStereoCaptureData(const WMFDeviceFormatInfo &deviceFormat)
        : section_width(deviceFormat.width / 2)
		, section_height(deviceFormat.height)
		, left_section(nullptr)
		, right_section(nullptr)
		, has_new_frame(false)
    {
		left_section = new cv::Mat(section_height, section_width,  CV_8UC3);
		right_section = new cv::Mat(section_height, section_width, CV_8UC3);
    }

	~WMFStereoCaptureData()
	{
		delete left_section;
		delete right_section;
	}

	void writeFrame(unsigned char *pSampleBuffer)
	{
		const cv::Mat videoBufferMat(section_height, 2*section_width, CV_8UC3, pSampleBuffer);
		const cv::Rect left_bounds(0, 0, section_width, section_height);
		const cv::Rect right_bounds(section_width, 0, section_width, section_height);

		videoBufferMat(left_bounds).copyTo(*left_section);
		videoBufferMat(right_bounds).copyTo(*right_section);

		has_new_frame= true;
	}

	int section_width;
	int section_height;
	cv::Mat *left_section;
	cv::Mat *right_section;
	bool has_new_frame;
};

class WMFVideoFrameProcessor : public IMFSampleGrabberSinkCallback
{
public:
	WMFVideoFrameProcessor(const WMFDeviceFormatInfo &deviceFormat);
	~WMFVideoFrameProcessor();

	HRESULT init(IMFMediaSource *pSource);
	void dispose();
	
	HRESULT start();
	void stop();
	bool pollNextEvent();

	inline const WMFStereoCaptureData *getStereoCaptureDataConst() const { return m_captureData; }
	inline WMFStereoCaptureData *getStereoCaptureData() { return m_captureData; }
	inline bool getIsRunning() const { return m_bIsRunning; }

protected:
	HRESULT CreateTopology(IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo);
	HRESULT AddSourceNode(
		IMFTopology *pTopology,           
		IMFMediaSource *pSource,          
		IMFPresentationDescriptor *pPD,   
		IMFStreamDescriptor *pSD,         
		IMFTopologyNode **ppNode);
	HRESULT AddOutputNode(
		IMFTopology *pTopology,     
		IMFActivate *pActivate,     
		DWORD dwId,                 
		IMFTopologyNode **ppNode);
	
	// IUnknown methods
	STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
    STDMETHODIMP_(ULONG) AddRef();
    STDMETHODIMP_(ULONG) Release();

    // IMFClockStateSink methods
	STDMETHODIMP OnClockStart(MFTIME hnsSystemTime, LONGLONG llClockStartOffset) { return S_OK; }
    STDMETHODIMP OnClockStop(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockPause(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockRestart(MFTIME hnsSystemTime) { return S_OK; }
    STDMETHODIMP OnClockSetRate(MFTIME hnsSystemTime, float flRate) { return S_OK; }

    // IMFSampleGrabberSinkCallback methods
    STDMETHODIMP OnSetPresentationClock(IMFPresentationClock* pClock)  { return S_OK; }
    STDMETHODIMP OnShutdown() { return S_OK; }
    STDMETHODIMP OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
        LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
        DWORD dwSampleSize);

private:
	long m_referenceCount;

	unsigned int m_deviceIndex;
	
	IMFMediaSession *m_pSession;
	IMFTopology *m_pTopology;
	
	WMFStereoCaptureData *m_captureData;
	bool m_bIsRunning;
};

class WMFVideoDevice
{
public:
    WMFVideoDevice(const int device_index, const WMFDeviceInfo &device_info);
	~WMFVideoDevice();

	bool open(int desiredFormatIndex);
	void close();

	inline const WMFVideoFrameProcessor *getVideoFrameProcessorConst() const { return m_videoFrameProcessor; }
	inline WMFVideoFrameProcessor *getVideoFrameProcessor() { return m_videoFrameProcessor; }
	const WMFDeviceFormatInfo *getCurrentDeviceFormat() const;
	bool getIsOpen() const;

	/*
	  See https://msdn.microsoft.com/en-us/library/windows/desktop/dd407328(v=vs.85).aspx
	  VideoProcAmp_Brightness		[-10k, 10k]
	  VideoProcAmp_Contrast			[0, 10k]
	  VideoProcAmp_Hue				[-180k, 180k]  
	  VideoProcAmp_Saturation		[0, 10k]
	  VideoProcAmp_Sharpness		[0, 100]
	  VideoProcAmp_Gamma			[1, 500]
	  VideoProcAmp_ColorEnable		0=off, 1=on
	  VideoProcAmp_WhiteBalance		device dependent
	  VideoProcAmp_BacklightCompensation		0=off, 1=on
	  VideoProcAmp_Gain				device dependent
	*/
	bool setProcAmpProperty(VideoProcAmpProperty propId, double unitValue, bool bAuto);
	double getProcAmpProperty(VideoProcAmpProperty propId, bool *bIsAuto = nullptr) const;

	inline double getGain() const;
	inline bool setGain(double value);

	/*
		https://msdn.microsoft.com/en-us/library/windows/desktop/dd318253(v=vs.85).aspx
		CameraControl_Pan			[-180, 180]
		CameraControl_Tilt			[-180, 180]
		CameraControl_Roll			[-180, 180]
		CameraControl_Zoom			[10, 600]
		CameraControl_Exposure		1/2^n seconds (example n=-3 is 1/8th seconds)
		CameraControl_Iris			units of f_stop*10
		CameraControl_Focus			 optimally focused target, in millimeters
	*/
	bool setCameraControlProperty(CameraControlProperty propId, double unitValue, bool bAuto);
	double getCameraControlProperty(CameraControlProperty propId, bool *bIsAuto = nullptr) const;

	double getExposure() const;
	bool setExposure(double value);

public:
	int m_deviceIndex;
	WMFDeviceInfo m_deviceInfo;
	int m_deviceFormatIndex;
	IMFMediaSource *m_mediaSource;
	WMFVideoFrameProcessor *m_videoFrameProcessor;
};

// -- public methods
// -- WMF Stereo Tracker Config
const int WMFStereoTrackerConfig::CONFIG_VERSION = 1;

WMFStereoTrackerConfig::WMFStereoTrackerConfig(const std::string &fnamebase)
    : PSVRConfig(fnamebase)
    , is_valid(false)
    , max_poll_failure_count(100)
	, frame_rate(60)
    , exposure(32)
    , gain(32)
    , camera_identifier("USB\\VID_1415&PID_2000\\bA_pB.C")
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
WMFStereoTrackerConfig::writeToJSON()
{
    configuru::Config pt{
        {"is_valid", is_valid},
        {"version", WMFStereoTrackerConfig::CONFIG_VERSION},
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
        {"camera_identifier", camera_identifier}
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
WMFStereoTrackerConfig::readFromJSON(const configuru::Config &pt)
{
    int config_version = pt.get_or<int>("version", 0);
    if (config_version == WMFStereoTrackerConfig::CONFIG_VERSION)
    {
        is_valid = pt.get_or<bool>("is_valid", false);
        max_poll_failure_count = pt.get_or<long>("max_poll_failure_count", 100);
		frame_rate = pt.get_or<double>("frame_rate", 60);
        exposure = pt.get_or<double>("exposure", 32);
		gain = pt.get_or<double>("gain", 32);

        camera_identifier= pt.get<std::string>("camera_identifier");

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
        PSVR_LOG_WARNING("WMFStereoTrackerConfig") <<
            "Config version " << config_version << " does not match expected version " <<
            WMFStereoTrackerConfig::CONFIG_VERSION << ", Using defaults.";
    }
}

const PSVR_HSVColorRangeTable *
WMFStereoTrackerConfig::getColorRangeTable(const std::string &table_name) const
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
WMFStereoTrackerConfig::getOrAddColorRangeTable(const std::string &table_name)
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

// -- WMFStereoTracker
WMFStereoTracker::WMFStereoTracker()
    : m_cfg()
	, m_videoDevice(nullptr)
    , m_DriverType(WMFStereoTracker::Generic_Webcam)
    , m_nextPollSequenceNumber(0)
    , m_trackerStates()
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

    bool bSuccess = true;
    
    if (getIsOpen())
    {
        PSVR_LOG_WARNING("WMFStereoTracker::open") << "WMFStereoTracker(" << cur_dev_path << ") already open. Ignoring request.";
    }
    else
    {
        const int camera_index = tracker_enumerator->get_camera_index();

        PSVR_LOG_INFO("WMFStereoTracker::open") << "Opening WMFStereoTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

        m_device_identifier = cur_dev_path;

        // Load the config file for the tracker
        m_cfg = WMFStereoTrackerConfig(m_device_identifier);
        m_cfg.load();

        // Save the config back out again in case defaults changed
        m_cfg.save();

        if (m_cfg.camera_identifier.length() == 0)
        {
            PSVR_LOG_WARNING("WMFStereoTracker::open") << 
                "WMFStereoTracker(" << cur_dev_path << 
                ", camera_index=" << camera_index << 
                ") has empty camera identifier";
            bSuccess= false;
        }

        if (bSuccess)
        {
			const WMFCameraEnumerator *wmf_enumerator= 
				tracker_enumerator->get_windows_media_foundation_camera_enumerator();

			PSVR_LOG_INFO("WMFStereoTracker::open") << "Opening WMFTracker(" << cur_dev_path << ", camera_index=" << camera_index << ")";

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

				if (m_videoDevice->open(desiredFormatIndex))
				{
					setExposure(m_cfg.exposure, false);
					setGain(m_cfg.gain, false);

					bSuccess = true;
				}
			}
        }
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

bool WMFStereoTracker::getIsReadyToPoll() const
{
    return getIsOpen();
}

IDeviceInterface::ePollResult WMFStereoTracker::poll()
{
    IDeviceInterface::ePollResult result = IDeviceInterface::_PollResultFailure;

    if (getIsOpen())
    {
		// Poll any incoming device events
		WMFVideoFrameProcessor *frameProcessor= m_videoDevice->getVideoFrameProcessor();
		while (frameProcessor->pollNextEvent());

		// See if new frame data is available
		WMFStereoCaptureData *captureData= frameProcessor->getStereoCaptureData();
        if (captureData->has_new_frame)
        {
			captureData->has_new_frame= false;
            result = IDeviceInterface::_PollResultSuccessNewData;
        }
        else
        {
            result = IDeviceInterface::_PollResultSuccessNoData;
		}

        {
            WMFStereoTrackerState newState;

            // Increment the sequence for every new polling packet
            newState.PollSequenceNumber = m_nextPollSequenceNumber;
            ++m_nextPollSequenceNumber;

            // Make room for new entry if at the max queue size
            //###bwalker $TODO Make this a fixed size circular buffer
            if (m_trackerStates.size() >= VIRTUAL_STEREO_STATE_BUFFER_MAX)
            {
                m_trackerStates.erase(m_trackerStates.begin(), m_trackerStates.begin() + m_trackerStates.size() - VIRTUAL_STEREO_STATE_BUFFER_MAX);
            }

            m_trackerStates.push_back(newState);
        }
    }

    return result;
}

void WMFStereoTracker::close()
{
	if (m_videoDevice != nullptr)
	{
		delete m_videoDevice;
		m_videoDevice= nullptr;
	}
}

long WMFStereoTracker::getMaxPollFailureCount() const
{
    return m_cfg.max_poll_failure_count;
}

CommonSensorState::eDeviceType WMFStereoTracker::getDeviceType() const
{
    return CommonSensorState::WMFStereoCamera;
}

const CommonSensorState *WMFStereoTracker::getSensorState(int lookBack) const
{
    const int queueSize = static_cast<int>(m_trackerStates.size());
    const CommonSensorState * result =
        (lookBack < queueSize) ? &m_trackerStates.at(queueSize - lookBack - 1) : nullptr;

    return result;
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

const unsigned char *WMFStereoTracker::getVideoFrameBuffer(PSVRVideoFrameSection section) const
{
    const unsigned char *result = nullptr;

    if (getIsOpen())
    {
		const WMFVideoFrameProcessor *frameProcessor= m_videoDevice->getVideoFrameProcessorConst();
		const WMFStereoCaptureData *captureData= frameProcessor->getStereoCaptureDataConst();

        switch (section)
        {
        case PSVRVideoFrameSection_Left:
            result= static_cast<const unsigned char *>(captureData->left_section->data);
            break;
        case PSVRVideoFrameSection_Right:
            result= static_cast<const unsigned char *>(captureData->right_section->data);
            break;
        }
    }

    return result;
}

void WMFStereoTracker::loadSettings()
{
    const double currentExposure= m_videoDevice->getExposure();
    const double currentGain= m_videoDevice->getGain();

    m_cfg.load();

    if (currentExposure != m_cfg.exposure)
    {
        m_videoDevice->setExposure(m_cfg.exposure);
    }

    if (currentGain != m_cfg.gain)
    {
        m_videoDevice->setGain(m_cfg.gain);
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
		m_videoDevice->open(desiredFormatIndex);
	}

	if (bUpdateConfig)
	{
		m_cfg.tracker_intrinsics.pixel_width = static_cast<float>(value);
	}
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
		m_videoDevice->open(desiredFormatIndex);
	}

	if (bUpdateConfig)
	{
		m_cfg.tracker_intrinsics.pixel_height = static_cast<float>(value);
	}
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
			m_videoDevice->open(desiredFormatIndex);
		}

	    if (bUpdateConfig)
	    {
		    m_cfg.frame_rate = value;
	    }
    }
}

double WMFStereoTracker::getFrameRate() const
{
	const WMFDeviceFormatInfo *deviceFormat= m_videoDevice->getCurrentDeviceFormat();
	double frameRate= (double)deviceFormat->frame_rate_numerator / (double)deviceFormat->frame_rate_denominator;

	return frameRate;
}

void WMFStereoTracker::setExposure(double value, bool bUpdateConfig)
{
    if (getExposure() != value)
    {
        m_videoDevice->setExposure(value);

	    if (bUpdateConfig)
	    {
		    m_cfg.exposure = value;
	    }
    }
}

double WMFStereoTracker::getExposure() const
{
	return m_videoDevice->getGain();
}

void WMFStereoTracker::setGain(double value, bool bUpdateConfig)
{
    if (getGain() != value)
    {
		m_videoDevice->setGain(value);

	    if (bUpdateConfig)
	    {
		    m_cfg.gain = value;
	    }
    }
}

double WMFStereoTracker::getGain() const
{
	return m_videoDevice->getGain();
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

// -- WMF Video Device -----
WMFVideoDevice::WMFVideoDevice(const int device_index, const WMFDeviceInfo &device_info) 
	: m_deviceIndex(device_index)
	, m_deviceInfo(device_info)
	, m_deviceFormatIndex(INVALID_DEVICE_FORMAT_INDEX)
	, m_mediaSource(nullptr)
	, m_videoFrameProcessor(nullptr)
{
}

WMFVideoDevice::~WMFVideoDevice()
{
	close();
}

bool WMFVideoDevice::open(int desiredFormatIndex)
{
	HRESULT hr;

	if (getIsOpen() && desiredFormatIndex == m_deviceFormatIndex)
	{
		return true;
	}

	if (desiredFormatIndex > 0 && desiredFormatIndex < m_deviceInfo.deviceAvailableFormats.size())
	{
		IMFAttributes *pAttributes = NULL;
		IMFActivate * vd_pActivate= NULL;

		// Close the device if it's currently open
		if (getIsOpen())
		{
			close();
		}

		// Remember the last 
		this->m_deviceFormatIndex= desiredFormatIndex;

		hr = MFCreateAttributes(&pAttributes, 1);
   
		if (SUCCEEDED(hr))
		{
			hr = pAttributes->SetGUID(
				MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
				MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID);
		}	

		IMFActivate *deviceActivationInterface= nullptr;
		if (SUCCEEDED(hr))
		{
			IMFActivate **ppDevices = nullptr;
			UINT32 deviceCount;
			hr = MFEnumDeviceSources(pAttributes, &ppDevices, &deviceCount);

			if (m_deviceIndex >= 0 && m_deviceIndex < (int)deviceCount)
			{
				deviceActivationInterface= ppDevices[m_deviceIndex];
				deviceActivationInterface->AddRef();
			}

			for(UINT32 i = 0; i < deviceCount; i++)
			{
				Utility::SafeRelease(&ppDevices[i]);
			}

			Utility::SafeReleaseAllCount(ppDevices);
		}

		if (SUCCEEDED(hr))
		{
			hr = deviceActivationInterface->ActivateObject(
				__uuidof(IMFMediaSource),
				(void**)&m_mediaSource);
		}

		IMFPresentationDescriptor *pPD = nullptr;
		if (SUCCEEDED(hr))
			hr = m_mediaSource->CreatePresentationDescriptor(&pPD);

		BOOL fSelected;
		IMFStreamDescriptor *pSD = nullptr;
		if (SUCCEEDED(hr))
			hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);

		IMFMediaTypeHandler *pHandler = nullptr;
		if (SUCCEEDED(hr))
			hr = pSD->GetMediaTypeHandler(&pHandler);

		DWORD cTypes = 0;
		if (SUCCEEDED(hr))
			hr = pHandler->GetMediaTypeCount(&cTypes);

		IMFMediaType *pType = nullptr;
		if (SUCCEEDED(hr))
			hr = pHandler->GetMediaTypeByIndex((DWORD)m_deviceFormatIndex, &pType);

		if (SUCCEEDED(hr))
			hr = pHandler->SetCurrentMediaType(pType);

		if (SUCCEEDED(hr))
		{
			const WMFDeviceFormatInfo &deviceFormat= 
				m_deviceInfo.deviceAvailableFormats[desiredFormatIndex];

			m_videoFrameProcessor = new WMFVideoFrameProcessor(deviceFormat);
			hr= m_videoFrameProcessor->init(m_mediaSource);
		}

		if (SUCCEEDED(hr))
			hr= m_videoFrameProcessor->start();

		Utility::SafeReleaseAllCount(&pPD);
		Utility::SafeRelease(&pSD);
		Utility::SafeRelease(&pHandler);
		Utility::SafeRelease(&pType);
		Utility::SafeReleaseAllCount(&deviceActivationInterface);
		Utility::SafeReleaseAllCount(&pAttributes);

		if (!SUCCEEDED(hr))
		{
			close();
		}
	}
	else
	{
		hr= E_INVALIDARG;
	}

	return SUCCEEDED(hr);
}

void WMFVideoDevice::close()
{
	if (m_videoFrameProcessor != nullptr)
	{
		delete m_videoFrameProcessor;
		m_videoFrameProcessor= nullptr;
	}

	if (m_mediaSource != nullptr)
	{
		m_mediaSource->Stop();
		Utility::SafeRelease(&m_mediaSource);
	}
}

bool WMFVideoDevice::getIsOpen() const
{
	return m_mediaSource != nullptr;
}

/*
	See https://msdn.microsoft.com/en-us/library/windows/desktop/dd407328(v=vs.85).aspx
	VideoProcAmp_Brightness		[-10k, 10k]
	VideoProcAmp_Contrast			[0, 10k]
	VideoProcAmp_Hue				[-180k, 180k]  
	VideoProcAmp_Saturation		[0, 10k]
	VideoProcAmp_Sharpness		[0, 100]
	VideoProcAmp_Gamma			[1, 500]
	VideoProcAmp_ColorEnable		0=off, 1=on
	VideoProcAmp_WhiteBalance		device dependent
	VideoProcAmp_BacklightCompensation		0=off, 1=on
	VideoProcAmp_Gain				device dependent
*/
bool WMFVideoDevice::setProcAmpProperty(VideoProcAmpProperty propId, double unitValue, bool bAuto)
{
	bool bSuccess= false;

	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pProcAmp->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		if (SUCCEEDED(hr))
		{
			long intValue= (long)((double)minValue*unitValue + (double)maxValue*(1.0-unitValue));

			hr = pProcAmp->Set(propId, intValue, bAuto ? VideoProcAmp_Flags_Auto : VideoProcAmp_Flags_Manual);
		}

		pProcAmp->Release();
	}

	return SUCCEEDED(hr);
}

double WMFVideoDevice::getProcAmpProperty(VideoProcAmpProperty propId, bool *bIsAuto) const
{
	double unitValue= 0;
	IAMVideoProcAmp *pProcAmp = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcAmp));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pProcAmp->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		if (SUCCEEDED(hr))
		{
			long intValue;
			hr = pProcAmp->Get(propId, &intValue, &flags);

			unitValue= (double)intValue - (double)(minValue) / ((double)(maxValue - minValue));

			if (bIsAuto != nullptr)
			{
				*bIsAuto = flags == VideoProcAmp_Flags_Auto;
			}
		}

		pProcAmp->Release();
	}

	return unitValue;
}

inline double WMFVideoDevice::getGain() const 
{
	return getProcAmpProperty(VideoProcAmp_Gain);
}

inline bool WMFVideoDevice::setGain(double value)
{
	return setProcAmpProperty(VideoProcAmp_Gain, value, false);
}

bool WMFVideoDevice::setCameraControlProperty(CameraControlProperty propId, double unitValue, bool bAuto)
{
	bool bSuccess= false;

	IAMCameraControl *pProcControl = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pProcControl));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pProcControl->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		if (SUCCEEDED(hr))
		{
			long intValue= (long)((double)minValue*unitValue + (double)maxValue*(1.0-unitValue));

			hr = pProcControl->Set(propId, intValue, bAuto ? CameraControl_Flags_Auto : CameraControl_Flags_Manual);
		}

		pProcControl->Release();
	}

	return SUCCEEDED(hr);
}

double WMFVideoDevice::getCameraControlProperty(CameraControlProperty propId, bool *bIsAuto) const
{
	double unitValue= 0;
	IAMCameraControl *pCameraControl = NULL;
	HRESULT hr = m_mediaSource->QueryInterface(IID_PPV_ARGS(&pCameraControl));

	if (SUCCEEDED(hr))
	{
		long minValue, maxValue, stepSize, defaultValue, flags;
		hr = pCameraControl->GetRange(propId, &minValue, &maxValue, &stepSize, &defaultValue, &flags);

		if (SUCCEEDED(hr))
		{
			long intValue;
			hr = pCameraControl->Get(propId, &intValue, &flags);

			unitValue= (double)intValue - (double)(minValue) / ((double)(maxValue - minValue));

			if (bIsAuto != nullptr)
			{
				*bIsAuto = flags == CameraControl_Flags_Auto;
			}
		}

		pCameraControl->Release();
	}

	return unitValue;
}

double WMFVideoDevice::getExposure() const
{
	return getCameraControlProperty(CameraControl_Exposure);
}

bool WMFVideoDevice::setExposure(double value)
{
	return setCameraControlProperty(CameraControl_Exposure, value, false);
}

const WMFDeviceFormatInfo *WMFVideoDevice::getCurrentDeviceFormat() const
{
	return 
		(m_deviceFormatIndex != INVALID_DEVICE_FORMAT_INDEX)
		? &m_deviceInfo.deviceAvailableFormats[m_deviceFormatIndex]
		: nullptr;
}

// -- WMF Video Frame Processor -----
WMFVideoFrameProcessor::WMFVideoFrameProcessor(
	const WMFDeviceFormatInfo &deviceFormat)
	: m_referenceCount(1)
	, m_pSession(nullptr)
	, m_pTopology(nullptr)
	, m_captureData(new WMFStereoCaptureData(deviceFormat))
	, m_bIsRunning(false)
{
}

WMFVideoFrameProcessor::~WMFVideoFrameProcessor(void)
{
	dispose();
}

HRESULT WMFVideoFrameProcessor::init(IMFMediaSource *pSource)
{
	// Clean up previous session, if any.
    if (m_pSession)
    {
        m_pSession->Shutdown();
    }
    Utility::SafeReleaseAllCount(&m_pSession);
    Utility::SafeReleaseAllCount(&m_pTopology);

	// Configure the media type that the video frame processor will receive.
	// Setting the major and subtype is usually enough for the topology loader
	// to resolve the topology.
    IMFMediaType *pType = nullptr;
	HRESULT hr = MFCreateMediaType(&pType);

	if (SUCCEEDED(hr))
		hr = pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);

	if (SUCCEEDED(hr))
		hr = pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_RGB24);

	// Create the sample grabber sink.
    IMFActivate *pSinkActivate = nullptr;
	if (SUCCEEDED(hr))
		hr = MFCreateSampleGrabberSinkActivate(pType, this, &pSinkActivate);

	// To run as fast as possible, set this attribute (requires Windows 7):
	if (SUCCEEDED(hr))
		hr = pSinkActivate->SetUINT32(MF_SAMPLEGRABBERSINK_IGNORE_CLOCK, TRUE);

	// Create the Media Session.
	if (SUCCEEDED(hr))
		hr = MFCreateMediaSession(NULL, &m_pSession);
	
	// Create the topology.
	if (SUCCEEDED(hr))
		hr = CreateTopology(pSource, pSinkActivate, &m_pTopology);

	
	// Clean up.
	if (FAILED(hr))
	{		
		if (m_pSession)
		{
			m_pSession->Shutdown();
		}

		Utility::SafeRelease(&m_pSession);
		Utility::SafeRelease(&m_pTopology);
	}

    Utility::SafeRelease(&pSinkActivate);
    Utility::SafeRelease(&pType);

	return hr;
}

void WMFVideoFrameProcessor::dispose()
{
	stop();

	if (m_pSession)
    {
		m_pSession->Shutdown();
    }
					
	Utility::SafeReleaseAllCount(&m_pSession);
	Utility::SafeReleaseAllCount(&m_pTopology);

	PSVR_LOG_INFO("WMFVideoFrameProcessor::dispose") << "Disposing video frame grabber for device: " << m_deviceIndex;
}

HRESULT WMFVideoFrameProcessor::start(void)
{
    HRESULT hr = m_pSession->SetTopology(0, m_pTopology);

	if (SUCCEEDED(hr))
	{
		PROPVARIANT var;
		PropVariantInit(&var);

		hr = m_pSession->Start(&GUID_NULL, &var);
	}

	if (SUCCEEDED(hr))
	{
		m_bIsRunning= true;
	}

    return hr;
}

bool WMFVideoFrameProcessor::pollNextEvent()
{
	bool bAnyEvents= false;

	if (m_bIsRunning)
	{
		IMFMediaEvent *pEvent = nullptr;
		HRESULT hr = m_pSession->GetEvent(MF_EVENT_FLAG_NO_WAIT, &pEvent);

		if (SUCCEEDED(hr) && hr != MF_E_NO_EVENTS_AVAILABLE)
		{
			HRESULT hrStatus;
			hr = pEvent->GetStatus(&hrStatus);

			if (!SUCCEEDED(hr) || !SUCCEEDED(hrStatus))
			{
				hr= E_FAIL;
			}

			MediaEventType met;
			if (SUCCEEDED(hr))
			{
				hr = pEvent->GetType(&met);
			}

			if(SUCCEEDED(hr))
			{
				switch (met)
				{
				case MESessionEnded:
					{			
						stop();
					} break;

				case MESessionStopped:
					{
					} break;

				case MEVideoCaptureDeviceRemoved:
					{
					} break;
				}
			}

			bAnyEvents= true;
		}

		Utility::SafeRelease(&pEvent);
	}

	return bAnyEvents;
}

void WMFVideoFrameProcessor::stop()
{
	PSVR_LOG_INFO("WMFVideoFrameProcessor::stop") << "Stopping video frame grabbing on device: " << m_deviceIndex;

	if (m_bIsRunning && m_pSession)
	{
		m_pSession->Stop();
		m_bIsRunning= false;
	}
}

HRESULT WMFVideoFrameProcessor::CreateTopology(
	IMFMediaSource *pSource, IMFActivate *pSinkActivate, IMFTopology **ppTopo)
{
    IMFTopology *pTopology = nullptr;
    HRESULT hr = MFCreateTopology(&pTopology);

    IMFPresentationDescriptor *pPD = nullptr;
	if (SUCCEEDED(hr))
		hr = pSource->CreatePresentationDescriptor(&pPD);

    DWORD cStreams = 0;
	if (SUCCEEDED(hr))
		hr = pPD->GetStreamDescriptorCount(&cStreams);
    
	if (SUCCEEDED(hr))
	{
		for (DWORD i = 0; i < cStreams; i++)
		{
			// Look for video streams and connect them to the sink
			BOOL fSelected = FALSE;
			GUID majorType;

		    IMFStreamDescriptor *pSD = nullptr;
			hr = pPD->GetStreamDescriptorByIndex(i, &fSelected, &pSD);

		    IMFMediaTypeHandler *pHandler = nullptr;
			if (SUCCEEDED(hr))
				hr = pSD->GetMediaTypeHandler(&pHandler);

			if (SUCCEEDED(hr))
				hr = pHandler->GetMajorType(&majorType);

			if (SUCCEEDED(hr))
			{
				if (majorType == MFMediaType_Video && fSelected)
				{
					IMFTopologyNode *pNode1 = nullptr;
					IMFTopologyNode *pNode2 = nullptr;

					hr = AddSourceNode(pTopology, pSource, pPD, pSD, &pNode1);

					if (SUCCEEDED(hr))
						hr = AddOutputNode(pTopology, pSinkActivate, 0, &pNode2);

					if (SUCCEEDED(hr))
						hr = pNode1->ConnectOutput(0, pNode2, 0);

					Utility::SafeRelease(&pNode1);
					Utility::SafeRelease(&pNode2);
					break;
				}
				else
				{
					hr = pPD->DeselectStream(i);
				}
			}

			Utility::SafeRelease(&pSD);
			Utility::SafeRelease(&pHandler);

			if (FAILED(hr))
				break;
		}
	}

	if (SUCCEEDED(hr))
	{
		*ppTopo = pTopology;
		(*ppTopo)->AddRef();
	}

    Utility::SafeRelease(&pTopology);
    Utility::SafeRelease(&pPD);

    return hr;
}

HRESULT WMFVideoFrameProcessor::AddSourceNode(
	IMFTopology *pTopology,           
	IMFMediaSource *pSource,          
	IMFPresentationDescriptor *pPD,   
	IMFStreamDescriptor *pSD,         
	IMFTopologyNode **ppNode)
{
    IMFTopologyNode *pNode = nullptr;
    HRESULT hr = MFCreateTopologyNode(MF_TOPOLOGY_SOURCESTREAM_NODE, &pNode);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_SOURCE, pSource);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_PRESENTATION_DESCRIPTOR, pPD);

	if (SUCCEEDED(hr))
		hr = pNode->SetUnknown(MF_TOPONODE_STREAM_DESCRIPTOR, pSD);

	if (SUCCEEDED(hr))
		hr = pTopology->AddNode(pNode);

    // Return the pointer to the caller.
	if (SUCCEEDED(hr))
	{
		*ppNode = pNode;
		(*ppNode)->AddRef();
	}

    Utility::SafeRelease(&pNode);

    return hr;
}

HRESULT WMFVideoFrameProcessor::AddOutputNode(
	IMFTopology *pTopology,
	IMFActivate *pActivate,
	DWORD dwId,
	IMFTopologyNode **ppNode)
{
    IMFTopologyNode *pNode = NULL;

    HRESULT hr = MFCreateTopologyNode(MF_TOPOLOGY_OUTPUT_NODE, &pNode);

	if (SUCCEEDED(hr))
		hr = pNode->SetObject(pActivate);

	if (SUCCEEDED(hr))
		hr = pNode->SetUINT32(MF_TOPONODE_STREAMID, dwId);

	if (SUCCEEDED(hr))
		hr = pNode->SetUINT32(MF_TOPONODE_NOSHUTDOWN_ON_REMOVE, FALSE);

	if (SUCCEEDED(hr))
		hr = pTopology->AddNode(pNode);

    // Return the pointer to the caller.
	if (SUCCEEDED(hr))
	{
		*ppNode = pNode;
		(*ppNode)->AddRef();
	}

    Utility::SafeRelease(&pNode);

    return hr;
}
	
STDMETHODIMP WMFVideoFrameProcessor::QueryInterface(REFIID riid, void** ppv)
{
	// Creation tab of shifting interfaces from start of this class
    static const QITAB qit[] = 
    {
        QITABENT(WMFVideoFrameProcessor, IMFSampleGrabberSinkCallback),
        QITABENT(WMFVideoFrameProcessor, IMFClockStateSink),
        { 0 }
    };
    return QISearch(this, qit, riid, ppv);
}

STDMETHODIMP_(ULONG) WMFVideoFrameProcessor::AddRef()
{
    return InterlockedIncrement(&m_referenceCount);
}

STDMETHODIMP_(ULONG) WMFVideoFrameProcessor::Release()
{
    ULONG cRef = InterlockedDecrement(&m_referenceCount);
    if (cRef == 0)
    {
        delete this;
    }
    return cRef;
}

STDMETHODIMP WMFVideoFrameProcessor::OnProcessSample(REFGUID guidMajorMediaType, DWORD dwSampleFlags,
	LONGLONG llSampleTime, LONGLONG llSampleDuration, const BYTE * pSampleBuffer,
	DWORD dwSampleSize)
{
	m_captureData->writeFrame(const_cast<unsigned char *>(pSampleBuffer));

	return S_OK;
}

#ifndef PS4CAMERA_TRACKER_H
#define PS4CAMERA_TRACKER_H

// -- includes -----
#include "PSVRConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

// -- pre-declarations -----
namespace cv {
    class VideoCapture;
}

// -- definitions -----
class PS4CameraTrackerConfig : public PSVRConfig
{
public:
    PS4CameraTrackerConfig(const std::string &fnamebase = "PS4CameraTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const PSVR_HSVColorRangeTable *getColorRangeTable(const std::string &table_name) const;
	inline PSVR_HSVColorRangeTable *getOrAddColorRangeTable(const std::string &table_name);
    
    bool is_valid;
    long max_poll_failure_count;
	double frame_rate;
    double exposure;
	double gain;

    PSVRStereoTrackerIntrinsics trackerIntrinsics;
    PSVRPosef pose;
	PSVR_HSVColorRangeTable SharedColorPresets;
	std::vector<PSVR_HSVColorRangeTable> DeviceColorPresets;

    static const int CONFIG_VERSION;
	static const int LENS_CALIBRATION_VERSION;
};

struct PS4CameraTrackerState : public CommonSensorState
{   
    PS4CameraTrackerState()
    {
        clear();
    }
    
    void clear()
    {
        CommonSensorState::clear();
        DeviceType = CommonSensorState::PS3EYE;
    }
};

class PS4CameraTracker : public ITrackerInterface {
public:
    PS4CameraTracker();
    virtual ~PS4CameraTracker();

    static void uploadFirmwareToAllPS4Cameras(const std::string &firmware_path);

    // PSVRTracker
    bool open(); // Opens the first HID device for the controller
    
    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    static CommonSensorState::eDeviceType getDeviceTypeStatic()
    { return CommonSensorState::PS4Camera; }
    CommonSensorState::eDeviceType getDeviceType() const override;
    const CommonSensorState *getSensorState(int lookBack = 0) const override;
    
    // -- ITrackerInterface
    ITrackerInterface::eDriverType getDriverType() const override;
    std::string getUSBDevicePath() const override;
    bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const override;
    bool getIsStereoCamera() const override { return true; }
    const unsigned char *getVideoFrameBuffer(PSVRVideoFrameSection section) const override;
    void loadSettings() override;
    void saveSettings() override;
	void setFrameWidth(double value, bool bUpdateConfig) override;
	double getFrameWidth() const override;
	void setFrameHeight(double value, bool bUpdateConfig) override;
	double getFrameHeight() const override;
	void setFrameRate(double value, bool bUpdateConfig) override;
	double getFrameRate() const override;
    void setExposure(double value, bool bUpdateConfig) override;
    double getExposure() const override;
	void setGain(double value, bool bUpdateConfig) override;
	double getGain() const override;
    void getCameraIntrinsics(PSVRTrackerIntrinsics &out_tracker_intrinsics) const override;
    void setCameraIntrinsics(const PSVRTrackerIntrinsics &tracker_intrinsics) override;
    PSVRPosef getTrackerPose() const override;
    void setTrackerPose(const PSVRPosef *pose) override;
    void getFOV(float &outHFOV, float &outVFOV) const override;
    void getZRange(float &outZNear, float &outZFar) const override;
    void gatherTrackingColorPresets(const std::string &controller_serial, PSVRClientTrackerSettings* settings) const override;
    void setTrackingColorPreset(const std::string &controller_serial, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset) override;
    void getTrackingColorPreset(const std::string &controller_serial, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const override;

    // -- Getters
    inline const PS4CameraTrackerConfig &getConfig() const
    { return cfg; }

private:
    PS4CameraTrackerConfig cfg;
    std::string USBDevicePath;
    class cv::VideoCapture *VideoCapture;
    class PS4CameraCaptureData *CaptureData;
    
    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PS4CameraTrackerState> TrackerStates;
};
#endif // PS4CAMERA_TRACKER_H

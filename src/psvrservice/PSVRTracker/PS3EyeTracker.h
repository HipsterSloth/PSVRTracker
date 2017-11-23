#ifndef PS3EYE_TRACKER_H
#define PS3EYE_TRACKER_H

// -- includes -----
#include "PSVRConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

// -- pre-declarations -----
namespace PSVRProtocol
{
    class Response_ResultTrackerSettings;
};

// -- definitions -----
class PS3EyeTrackerConfig : public PSVRConfig
{
public:
    enum eFOVSetting
    {
        RedDot, // 56 degree FOV
        BlueDot, // 75 degree FOV
        
        MAX_FOV_SETTINGS
    };

    PS3EyeTrackerConfig(const std::string &fnamebase = "PS3EyeTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const PSVR_HSVColorRangeTable *getColorRangeTable(const std::string &table_name) const;
	inline PSVR_HSVColorRangeTable *getOrAddColorRangeTable(const std::string &table_name);
    
    bool is_valid;
    long max_poll_failure_count;
	double frame_rate;
    double exposure;
	double gain;

    eFOVSetting fovSetting;    
    PSVRMonoTrackerIntrinsics trackerIntrinsics;
    PSVRPosef pose;
	PSVR_HSVColorRangeTable SharedColorPresets;
	std::vector<PSVR_HSVColorRangeTable> DeviceColorPresets;

    static const int CONFIG_VERSION;
	static const int LENS_CALIBRATION_VERSION;
};

struct PS3EyeTrackerState : public CommonSensorState
{   
    PS3EyeTrackerState()
    {
        clear();
    }
    
    void clear()
    {
        CommonSensorState::clear();
        DeviceType = CommonSensorState::PS3EYE;
    }
};

class PS3EyeTracker : public ITrackerInterface {
public:
    PS3EyeTracker();
    virtual ~PS3EyeTracker();
        
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
    { return CommonSensorState::PS3EYE; }
    CommonSensorState::eDeviceType getDeviceType() const override;
    const CommonSensorState *getSensorState(int lookBack = 0) const override;
    
    // -- ITrackerInterface
    ITrackerInterface::eDriverType getDriverType() const override;
    std::string getUSBDevicePath() const override;
    bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const override;
    bool getIsStereoCamera() const override { return false; }
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
    inline const PS3EyeTrackerConfig &getConfig() const
    { return cfg; }

private:
    PS3EyeTrackerConfig cfg;
    std::string USBDevicePath;
    class PSEyeVideoCapture *VideoCapture;
    class PSEyeCaptureData *CaptureData;
    ITrackerInterface::eDriverType DriverType;    
    
    // Read Controller State
    int NextPollSequenceNumber;
    std::deque<PS3EyeTrackerState> TrackerStates;
};
#endif // PS3EYE_TRACKER_H

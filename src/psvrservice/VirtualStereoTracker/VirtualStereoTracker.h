#ifndef VIRTUAL_STEREO_TRACKER_H
#define VIRTUAL_STEREO_TRACKER_H

// -- includes -----
#include "PSVRConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <array>
#include <deque>

// -- pre-declarations -----
namespace PSMoveProtocol
{
    class Response_ResultTrackerSettings;
};

// -- definitions -----
class VirtualStereoTrackerConfig : public PSVRConfig
{
public:
    VirtualStereoTrackerConfig(const std::string &fnamebase = "VirtualStereoTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const PSVR_HSVColorRangeTable *getColorRangeTable(const std::string &table_name) const;
	inline PSVR_HSVColorRangeTable *getOrAddColorRangeTable(const std::string &table_name);
    
    bool is_valid;
    long max_poll_failure_count;

	double frame_rate;
    double exposure;
	double gain;

    std::string left_camera_usb_path;
    std::string right_camera_usb_path;

    PSVRStereoTrackerIntrinsics tracker_intrinsics;
    PSVRPosef pose;
	PSVR_HSVColorRangeTable SharedColorPresets;
	std::vector<PSVR_HSVColorRangeTable> DeviceColorPresets;

    static const int CONFIG_VERSION;
};

struct VirtualStereoTrackerState : public CommonSensorState
{   
    VirtualStereoTrackerState()
    {
        clear();
    }
    
    void clear()
    {
        CommonSensorState::clear();
        DeviceType = CommonSensorState::VirtualStereoCamera;
    }
};

class VirtualStereoTracker : public ITrackerInterface {
public:
    VirtualStereoTracker();
    virtual ~VirtualStereoTracker();
        
    // Stereo Tracker
    bool open(); // Opens the first virtual stereo tracker
    
    // -- IDeviceInterface
    bool matchesDeviceEnumerator(const DeviceEnumerator *enumerator) const override;
    bool open(const DeviceEnumerator *enumerator) override;
    bool getIsOpen() const override;
    bool getIsReadyToPoll() const override;
    IDeviceInterface::ePollResult poll() override;
    void close() override;
    long getMaxPollFailureCount() const override;
    static CommonSensorState::eDeviceType getDeviceTypeStatic()
    { return CommonSensorState::VirtualStereoCamera; }
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
	bool getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const override;
    void setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool save_setting) override;
    int getVideoProperty(const PSVRVideoPropertyType property_type) const override;
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
    inline const VirtualStereoTrackerConfig &getConfig() const
    { return cfg; }

private:
    VirtualStereoTrackerConfig cfg;
    std::string device_identifier;

    class ITrackerInterface *LeftTracker;
    class ITrackerInterface *RightTracker;
    class VirtualStereoCaptureData *CaptureData;
    ITrackerInterface::eDriverType DriverType;    
    
    // Read Tracker State
    int NextPollSequenceNumber;
    std::deque<VirtualStereoTrackerState> TrackerStates;
};
#endif // PS3EYE_TRACKER_H

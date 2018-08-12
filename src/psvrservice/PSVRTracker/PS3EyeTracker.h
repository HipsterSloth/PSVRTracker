#ifndef PS3EYE_TRACKER_H
#define PS3EYE_TRACKER_H

// -- includes -----
#include "PS3EyeConfig.h"
#include "CommonTrackerConfig.h"
#include "DeviceEnumerator.h"
#include "DeviceInterface.h"
#include <string>
#include <vector>
#include <deque>

// -- definitions -----
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
    void close() override;
    static CommonSensorState::eDeviceType getDeviceTypeStatic()
    { return CommonSensorState::PS3EYE; }
    CommonSensorState::eDeviceType getDeviceType() const override;
    
    // -- ITrackerInterface
    ITrackerInterface::eDriverType getDriverType() const override;
    std::string getUSBDevicePath() const override;
    bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const override;
    bool getIsStereoCamera() const override { return false; }
	bool getIsFrameMirrored() const override { return false; }
	bool getIsBufferMirrored() const override { return false; }
    void loadSettings() override;
    void saveSettings() override;
	bool getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const override;
	const struct TrackerModeConfig *getTrackerMode() const override;
	bool setTrackerMode(const std::string modeName) override;
	double getFrameWidth() const override;
	double getFrameHeight() const override;
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
	void setTrackerListener(ITrackerListener *listener) override;

    // -- Getters
    inline const PS3EyeTrackerConfig &getConfig() const
    { return m_cfg; }

private:
	const class TrackerCapabilitiesConfig *m_capabilities;
	const struct TrackerModeConfig *m_currentMode;

    PS3EyeTrackerConfig m_cfg;
    std::string m_deviceIdentifier;

    class PS3EyeVideoDevice *m_videoDevice;
    ITrackerInterface::eDriverType m_DriverType;    
	ITrackerListener *m_listener;
};
#endif // PS3EYE_TRACKER_H

#ifndef TRACKER_CAPABILITIES_CONFIG_H
#define TRACKER_CAPABILITIES_CONFIG_H

// -- includes -----
#include "ClientColor_CAPI.h"
#include "DeviceInterface.h"
#include "PSVRConfig.h"

// -- constants -----
#define CAMERA_BUFFER_FORMAT_MJPG			"MJPG"
#define CAMERA_BUFFER_FORMAT_YUY2			"YUY2"
#define CAMERA_BUFFER_FORMAT_NV12			"NV12"
#define CAMERA_BUFFER_FORMAT_BEYER			"BEYER"

// -- definitions -----
struct TrackerFrameSectionInfo
{
    const configuru::Config writeToJSON() const;
    void readFromJSON(const configuru::Config &pt);

	int x;
	int y;
};

struct TrackerModeConfig
{
    const configuru::Config writeToJSON() const;
    void readFromJSON(const configuru::Config &pt);

	std::string modeName;

	float frameRate;
	bool isFrameMirrored;
	bool isBufferMirrored;
	int bufferPixelWidth;
	int bufferPixelHeight;
	std::string bufferFormat;
	std::vector<TrackerFrameSectionInfo> frameSections;
	
	PSVRTrackerIntrinsics intrinsics;
};

class TrackerCapabilitiesConfig : public PSVRConfig
{
public:
    TrackerCapabilitiesConfig(const std::string &fnamebase = "CommonTrackerConfig");
    
    virtual const configuru::Config writeToJSON();
    virtual void readFromJSON(const configuru::Config &pt);

	const TrackerModeConfig *findCameraMode(const std::string &mode_name) const;
	void getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const;

	std::string friendlyName;
	int usbProductId;
	int usbVendorId;
	CommonSensorState::eDeviceType deviceType;
	std::vector<TrackerModeConfig> supportedModes;	
};

class TrackerCapabilitiesSet
{
public:
	bool reloadSupportedTrackerCapabilities();
	bool supportsTracker(unsigned short vendor_id, unsigned short product_id) const;
	CommonSensorState::eDeviceType findTrackerType(unsigned short vendor_id, unsigned short product_id) const;
	const TrackerCapabilitiesConfig *getTrackerCapabilities(unsigned short vendor_id, unsigned short product_id) const;

private:
	std::vector<TrackerCapabilitiesConfig> m_supportedTrackers;
};

#endif // TRACKER_CAPABILITIES_CONFIG_H

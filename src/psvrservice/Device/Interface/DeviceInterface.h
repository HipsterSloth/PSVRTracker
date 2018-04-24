#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

// -- includes -----
#include <string>
#include <tuple>
#include "PSVRClient_CAPI.h"

// -- pre-declarations ----
namespace PSVRProtocol
{
    class Response_ResultTrackerSettings;
    class TrackingColorPreset;
};

// -- definitions -----

struct CommonSensorState
{
    enum eDeviceClass
    {
        TrackingCamera = 0x00,
        HeadMountedDisplay = 0x10
    };
    
    enum eDeviceType
    {       
        PS3EYE = TrackingCamera + 0x00,
        PS4Camera = TrackingCamera + 0x01,
        VirtualStereoCamera = TrackingCamera + 0x02,
		WMFStereoCamera = TrackingCamera + 0x03,
        SUPPORTED_CAMERA_TYPE_COUNT = TrackingCamera + 0x04,
        
        Morpheus = HeadMountedDisplay + 0x00,
        VirtualHMD = HeadMountedDisplay + 0x01,
        SUPPORTED_HMD_TYPE_COUNT = HeadMountedDisplay + 0x02,

		INVALID_DEVICE_TYPE= 0xFF,
    };
    
    eDeviceType DeviceType;
    int PollSequenceNumber;
    
    inline CommonSensorState()
    {
        clear();
    }
    
    inline void clear()
    {
        DeviceType= INVALID_DEVICE_TYPE; // invalid
        PollSequenceNumber= 0;
    }

    static const char *getDeviceTypeString(eDeviceType device_type)
    {
        const char *result = nullptr;

        switch (device_type)
        {
        case PS3EYE:
            result = "PSEYE";
            break;
        case PS4Camera:
            result = "PS4Camera";
            break;
        case VirtualStereoCamera:
            result = "VirtualStereoCamera";
            break;
        case WMFStereoCamera:
            result = "WMFStereoCamera";
            break;
        case Morpheus:
            result = "Morpheus";
            break;
        case VirtualHMD:
            result = "VirtualHMD";
            break;
        default:
            result = "UNKNOWN";
        }

        return result;
    }
};

struct CommonHMDSensorState : CommonSensorState
{
    PSVRPosef Pose;

    inline CommonHMDSensorState()
    {
        clear();
    }

    inline void clear()
    {
        CommonSensorState::clear();

        Pose= *k_PSVR_pose_identity;
    }
};

/// Abstract base class for any device interface. Further defined in specific device abstractions.
class IDeviceInterface
{
public:
    enum ePollResult
    {
        _PollResultSuccessNoData,
        _PollResultSuccessNewData,
        _PollResultFailure,
    };
    
	virtual ~IDeviceInterface() {};

    // Return true if device path matches
    virtual bool matchesDeviceEnumerator(const class DeviceEnumerator *enumerator) const = 0;
    
    // Opens the HID device for the device at the given enumerator
    virtual bool open(const class DeviceEnumerator *enumerator) = 0;
    
    // Returns true if hidapi opened successfully
    virtual bool getIsOpen() const  = 0;
    
    virtual bool getIsReadyToPoll() const = 0;
    
    // Polls for new device data
    virtual ePollResult poll() = 0;
    
    // Closes the HID device for the device
    virtual void close() = 0;
    
    // Get the number of milliseconds we're willing to accept no data from the device before we disconnect it
    virtual long getMaxPollFailureCount() const = 0;
    
    // Returns what type of device
    virtual CommonSensorState::eDeviceType getDeviceType() const = 0;
    
    // Fetch the device sensor state at the given sample index.
    // A lookBack of 0 corresponds to the most recent data.
    virtual const CommonSensorState * getSensorState(int lookBack = 0) const = 0;   
};

/// Abstract class for Tracker interface. Implemented Tracker classes
class ITrackerInterface : public IDeviceInterface
{
public:
    enum eDriverType
    {
        Libusb,
        CL,
        CLMulti,
        Generic_Webcam,

        SUPPORTED_DRIVER_TYPE_COUNT,
    };

    // -- Getters
    // Returns the driver type being used by this camera
    virtual eDriverType getDriverType() const = 0;

    // Returns the full usb device path for the tracker
    virtual std::string getUSBDevicePath() const = 0;

    // Returns the video frame size (used to compute frame buffer size)
    virtual bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const = 0;

    // Returns true if this device is a stereo camera
    virtual bool getIsStereoCamera() const = 0;

    // Returns a pointer to the last video frame buffer captured
    virtual const unsigned char *getVideoFrameBuffer(PSVRVideoFrameSection section) const = 0;

    static const char *getDriverTypeString(eDriverType device_type)
    {
        const char *result = nullptr;

        switch (device_type)
        {
        case Libusb:
            result = "Libusb";
            break;
        case CL:
            result = "CL";
            break;
        case CLMulti:
            result = "CLMulti";
            break;
        case Generic_Webcam:
            result = "Generic_Webcam";
            break;
        default:
            result = "UNKNOWN";
        }

        return result;
    }
    
    virtual void loadSettings() = 0;
    virtual void saveSettings() = 0;

	virtual void setFrameWidth(double value, bool bUpdateConfig) = 0;
	virtual double getFrameWidth() const = 0;

	virtual void setFrameHeight(double value, bool bUpdateConfig) = 0;
	virtual double getFrameHeight() const = 0;

	virtual void setFrameRate(double value, bool bUpdateConfig) = 0;
	virtual double getFrameRate() const = 0;

    virtual void setExposure(double value, bool bUpdateConfig) = 0;
    virtual double getExposure() const = 0;

	virtual void setGain(double value, bool bUpdateConfig) = 0;
	virtual double getGain() const = 0;

    virtual void getCameraIntrinsics(PSVRTrackerIntrinsics &out_tracker_intrinsics) const = 0;
    virtual void setCameraIntrinsics(const PSVRTrackerIntrinsics &tracker_intrinsics) = 0;

    virtual PSVRPosef getTrackerPose() const = 0;
    virtual void setTrackerPose(const PSVRPosef *pose) = 0;

    virtual void getFOV(float &outHFOV, float &outVFOV) const = 0;
    virtual void getZRange(float &outZNear, float &outZFar) const = 0;

    virtual void gatherTrackingColorPresets(const std::string &table_name, PSVRClientTrackerSettings* settings) const = 0;
    virtual void setTrackingColorPreset(const std::string &table_name, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset) = 0;
    virtual void getTrackingColorPreset(const std::string &table_name, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const = 0;
};

/// Abstract class for HMD interface. Implemented HMD classes
class IHMDInterface : public IDeviceInterface
{
public:
    // -- Getters
    // Returns the full usb device path for the HMD
    virtual std::string getUSBDevicePath() const = 0;

	// Get the tracking shape use by the controller
	virtual void getTrackingShape(PSVRTrackingShape &outTrackingShape) const = 0;

	// Sets the tracking color enum of the controller
	virtual bool setTrackingColorID(const PSVRTrackingColorType tracking_color_id) = 0;

	// Get the tracking color enum of the controller
	virtual bool getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const = 0;

	// Get the state prediction time from the HMD config
	virtual float getPredictionTime() const = 0;
};

#endif // DEVICE_INTERFACE_H

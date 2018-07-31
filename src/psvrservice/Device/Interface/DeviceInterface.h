#ifndef DEVICE_INTERFACE_H
#define DEVICE_INTERFACE_H

// -- includes -----
#include <string>
#include <tuple>
#include <vector>

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
        WMFMonoCamera = TrackingCamera + 0x01,
		WMFStereoCamera = TrackingCamera + 0x02,
        SUPPORTED_CAMERA_TYPE_COUNT = TrackingCamera + 0x03,
        
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
        case WMFMonoCamera:
            result = "WMFMonoCamera";
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

/// Interface base class for any device interface. Further defined in specific device abstractions.
class IDeviceInterface
{
public:
   
	virtual ~IDeviceInterface() {};

    // Return true if device path matches
    virtual bool matchesDeviceEnumerator(const class DeviceEnumerator *enumerator) const = 0;
    
    // Opens the HID device for the device at the given enumerator
    virtual bool open(const class DeviceEnumerator *enumerator) = 0;
    
    // Returns true if hidapi opened successfully
    virtual bool getIsOpen() const  = 0;
       
    // Closes the HID device for the device
    virtual void close() = 0;
        
    // Returns what type of device
    virtual CommonSensorState::eDeviceType getDeviceType() const = 0;    
};

/// Interface class for HMD events. Implemented HMD Server View
class ITrackerListener
{
public:
	// Called when new video frame has been received from the tracker device
	virtual void notifyVideoFrameReceived(const unsigned char *raw_video_frame) = 0;
};

/// Interface class for Tracker interface. Implemented Tracker classes
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

    // Returns the driver type being used by this camera
    virtual eDriverType getDriverType() const = 0;

    // Returns the full usb device path for the tracker
    virtual std::string getUSBDevicePath() const = 0;

    // Returns the video frame size (used to compute frame buffer size)
    virtual bool getVideoFrameDimensions(int *out_width, int *out_height, int *out_stride) const = 0;

    // Returns true if this device is a stereo camera
    virtual bool getIsStereoCamera() const = 0;

    // Returns true if the frame coming from the camera is mirrored backwards
    virtual bool getIsFrameMirrored() const = 0;
    // Returns true if the left and right frames coming from the camera are swapped
	virtual bool getIsBufferMirrored() const = 0;

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

	virtual bool getAvailableTrackerModes(std::vector<std::string> &out_mode_names) const = 0;
	virtual const struct TrackerModeConfig *getTrackerMode() const = 0;
	virtual bool setTrackerMode(const std::string modeName) = 0;

	virtual double getFrameWidth() const = 0;
	virtual double getFrameHeight() const = 0;
	virtual double getFrameRate() const = 0;

	virtual bool getVideoPropertyConstraint(const PSVRVideoPropertyType property_type, PSVRVideoPropertyConstraint &outConstraint) const = 0;

    virtual void setVideoProperty(const PSVRVideoPropertyType property_type, int desired_value, bool save_setting) = 0;
    virtual int getVideoProperty(const PSVRVideoPropertyType property_type) const = 0;

    virtual void getCameraIntrinsics(PSVRTrackerIntrinsics &out_tracker_intrinsics) const = 0;
    virtual void setCameraIntrinsics(const PSVRTrackerIntrinsics &tracker_intrinsics) = 0;

    virtual PSVRPosef getTrackerPose() const = 0;
    virtual void setTrackerPose(const PSVRPosef *pose) = 0;

    virtual void getFOV(float &outHFOV, float &outVFOV) const = 0;
    virtual void getZRange(float &outZNear, float &outZFar) const = 0;

    virtual void gatherTrackingColorPresets(const std::string &table_name, PSVRClientTrackerSettings* settings) const = 0;
    virtual void setTrackingColorPreset(const std::string &table_name, PSVRTrackingColorType color, const PSVR_HSVColorRange *preset) = 0;
    virtual void getTrackingColorPreset(const std::string &table_name, PSVRTrackingColorType color, PSVR_HSVColorRange *out_preset) const = 0;

	// Assign a Tracker listener to send Tracker events to
	virtual void setTrackerListener(ITrackerListener *listener) = 0;
};

/// Interface class for HMD events. Implemented HMD Server View
class IHMDListener
{
public:
	// Called when new sensor state has been read from the HMD
	virtual void notifySensorDataReceived(const CommonSensorState *sensor_state) = 0;
};

/// Interface class for HMD interface. Implemented by HMD classes
class IHMDInterface : public IDeviceInterface
{
public:
    // -- Getters
    // Returns the full usb device path for the HMD
    virtual std::string getUSBDevicePath() const = 0;

	// Get the tracking shape use by the controller
	virtual void getTrackingShape(PSVRTrackingShape &outTrackingShape) const = 0;

	// Get the tracking color enum of the controller
	virtual bool getTrackingColorID(PSVRTrackingColorType &out_tracking_color_id) const = 0;

	// Get the state prediction time from the HMD config
	virtual float getPredictionTime() const = 0;

    // -- Mutators
	// Sets the tracking color enum of the controller
	virtual bool setTrackingColorID(const PSVRTrackingColorType tracking_color_id) = 0;

	// Assign an HMD listener to send HMD events to
	virtual void setHMDListener(IHMDListener *listener) = 0;
};

#endif // DEVICE_INTERFACE_H

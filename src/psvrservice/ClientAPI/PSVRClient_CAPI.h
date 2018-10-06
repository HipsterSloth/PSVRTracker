/**
\file
*/ 

#ifndef __PSVRCLIENT_CAPI_H
#define __PSVRCLIENT_CAPI_H
#include "PSVRClient_export.h"
#include "ClientConstants.h"
#include "ClientColor_CAPI.h"
#include "ClientGeometry_CAPI.h"
#include <stdbool.h>
//cut_before

/** 
\brief Client Interface for PSVRService
\defgroup PSVRClient_CAPI Client Interface
\addtogroup PSVRClient_CAPI 
@{ 
*/
 
// Wrapper Types
//--------------

/// The ID of a controller in the controller pool
typedef int PSVRControllerID;

/// The ID of a tracker in the tracker pool
typedef int PSVRTrackerID;

/// The ID of an HMD in the HMD pool
typedef int PSVRHmdID;

// Shared Constants
//-----------------

/// Result enum in response to a client API request
typedef enum
{
	PSVRResult_Canceled              = -3,	///< Request Was Canceled
    PSVRResult_NoData                = -2,	///< Request Returned No Data
    PSVRResult_Error                 = -1, 	///< General Error Result
    PSVRResult_Success               = 0,	///< General Success Result
} PSVRResult;

/// The list of possible camera types tracked by PSVRService
typedef enum
{
    PSVRTracker_None= -1,
    PSVRTracker_PS3Eye,
    PSVRTracker_GenericMonoCamera,
	PSVRTracker_GenericStereoCamera
} PSVRTrackerType;

/// The list of possible HMD types tracked by PSVRService
typedef enum
{
    PSVRHmd_None= -1,
	PSVRHmd_Morpheus= 0,
    PSVRHmd_Virtual= 1,
} PSVRHmdType;

/// The list of possible camera drivers used by PSVRService
typedef enum
{
    PSVRDriver_LIBUSB,
	PSVRDriver_WINUSB,
    PSVRDriver_WINDOWSMEDIAFRAMEWORK
} PSVRTrackerDriver;

/// Connection type for a device
typedef enum
{
    PSVRConnectionType_BLUETOOTH,	///< Device connected over bluetooth
    PSVRConnectionType_USB			///< Device connected over USB
    
} PSVRConnectionType;

/// De-bounced state of a button
typedef enum 
{
    PSVRButtonState_UP = 0x00,       ///< (00b) Not pressed
    PSVRButtonState_PRESSED = 0x01,  ///< (01b) Down for one frame only
    PSVRButtonState_DOWN = 0x03,     ///< (11b) Down for >1 frame
    PSVRButtonState_RELEASED = 0x02, ///< (10b) Up for one frame only
} PSVRButtonState;

/// Battery charge state levels
typedef enum
{
    PSVRBattery_0        = 0,
    PSVRBattery_20       = 1,
    PSVRBattery_40       = 2,
    PSVRBattery_60       = 3,
    PSVRBattery_80       = 4,
    PSVRBattery_100      = 5,
    PSVRBattery_Charging = 0xEE,
    PSVRBattery_Charged  = 0xEF
} PSVRBatteryState;

/// Tracked device data stream options
typedef enum
{
    PSVRStreamFlags_defaultStreamOptions = 0x00,			///< Default minimal data stream
    PSVRStreamFlags_includePositionData = 0x01,			///< Add position data (turns on tracking)
    PSVRStreamFlags_includePhysicsData = 0x02,			///< Add IMU physics state
    PSVRStreamFlags_includeRawSensorData = 0x04,			///< Add raw IMU sensor data
	PSVRStreamFlags_includeCalibratedSensorData = 0x08,	///< Add calibrated IMU sensor state
    PSVRStreamFlags_includeRawTrackerData = 0x10,		///< Add raw optical tracking projection info
	PSVRStreamFlags_disableROI = 0x20,					///< Disable Region-of-Interest tracking optimization
} PSVRDeviceDataStreamFlags;

/// Tracking Debug flags
typedef enum
{
    PSVRTrackerDebugFlags_none = 0x00,						///< Turn off all
    PSVRTrackerDebugFlags_trackingModel = 0x01,				///< Show tracking model debugging
} PSVRTrackerDebugFlags;

/// The possible rumble channels available to the controllers
typedef enum
{
    PSVRControllerRumbleChannel_All,		///< Rumble across all channels
    PSVRControllerRumbleChannel_Left,	///< Rumble on the left channel
    PSVRControllerRumbleChannel_Right	///< Rumble on the right channel
} PSVRControllerRumbleChannel;

/// The list of possible controller types tracked by PSMoveService
typedef enum
{
    PSVRController_None= -1,
    PSVRController_Move,
	PSVRController_DualShock4,
} PSVRControllerType;

/// Describes which hand the given device is intended for
typedef enum 
{
	PSMControllerHand_Any = 0,
	PSMControllerHand_Left = 1,
	PSMControllerHand_Right = 2,
} PSVRControllerHand;

// Tracker State
//--------------

/// Device projection geometry as seen by each tracker
typedef struct
{
	/// ID of the selected tracker
    PSVRTrackerID            TrackerID;
	/// Pixel position of device projection centroid on each tracker
    PSVRVector2f             ScreenLocations[2];
	/// Tracker relative device 3d position on each tracker
    PSVRVector3f             RelativePositionCm;
	/// Tracker relative device 3d orientation on each tracker
    PSVRQuatf                RelativeOrientation;
	/// Tracker relative device projection geometry on each tracker
    PSVRTrackingProjection   TrackingProjection;
	/// World relative estimated tracking shape
	PSVRTrackingShape        WorldRelativeShape;
	/// A bitmask of the trackers with valid projections
    unsigned int            ValidTrackerBitmask;
} PSVRRawTrackerData;

/// Radial and tangential lens distortion coefficients computed during lens lens calibration
/// See the [OpenCV Docs](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) for details
typedef struct 
{
    double k1; ///< Radial Distortion Parameter 1
    double k2; ///< Radial Distortion Parameter 2
    double k3; ///< Radial Distortion Parameter 3
    double p1; ///< Tangential Distortion Parameter 1
    double p2; ///< Tangential Distortion Parameter 2
} PSVRDistortionCoefficients;

/// Camera intrinsic properties for a monoscopic camera
/// See the [OpenCV Docs](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) for details
typedef struct  
{
    float pixel_width;  ///< Width of the camera buffer in pixels
    float pixel_height; ///< Height of the camera buffer in pixels
    float hfov;         ///< The horizontal field of view camera in degrees
    float vfov;         ///< The vertical field of view camera in degrees
    float znear;        ///< The distance of the near clipping plane in cm
    float zfar;         ///< The distance of the far clipping plane in cm
    PSVRDistortionCoefficients distortion_coefficients;   ///< Lens distortion coefficients
    PSVRMatrix3d camera_matrix;   ///< Intrinsic camera matrix containing focal lengths and principal point
} PSVRMonoTrackerIntrinsics;

/// Camera intrinsic properties for a stereoscopic camera
/// See the [OpenCV Docs](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) for details
typedef struct 
{
    // Keep these in sync with PSVRMonoTrackerIntrinsics
    float pixel_width;  ///< Width of the camera buffer in pixels
    float pixel_height; ///< Height of the camera buffer in pixels
    float hfov;         ///< The horizontal field of view camera in degrees
    float vfov;         ///< The vertical field of view camera in degrees
    float znear;        ///< The distance of the near clipping plane in cm
    float zfar;         ///< The distance of the far clipping plane in cm
    PSVRDistortionCoefficients left_distortion_coefficients; ///< Left lens distortion coefficients
    PSVRMatrix3d left_camera_matrix; ///< Intrinsic matrix for left camera containing focal lengths and principal point
    // Keep these in sync with PSVRMonoTrackerIntrinsics

    PSVRDistortionCoefficients right_distortion_coefficients; ///< Right lens distortion coefficients
    PSVRMatrix3d right_camera_matrix; ///< Intrinsic matrix for rotation camera containing focal lengths and principal point
    PSVRMatrix3d left_rectification_rotation; ///< Rotation applied to left camera to rectify the image
    PSVRMatrix3d right_rectification_rotation; ///< Rotation applied to right camera to rectify the image
    PSVRMatrix34d left_rectification_projection; ///< Projection applied to left camera to rectify the image
    PSVRMatrix34d right_rectification_projection; ///< Projection applied to right camera to rectify the image
    PSVRMatrix3d rotation_between_cameras; ///< Rotation between the left and right cameras
    PSVRVector3d translation_between_cameras; ///< Translation between the left and right camera
    PSVRMatrix3d essential_matrix; ///< Transform relating points in unit coordinate space between cameras
    PSVRMatrix3d fundamental_matrix; ///< Transform relating points in pixel coordinates between cameras
    PSVRMatrix4d reprojection_matrix;  ///< Transform relating pixel x,y + disparity to distance from cameras
} PSVRStereoTrackerIntrinsics;

typedef enum 
{
    PSVR_MONO_TRACKER_INTRINSICS,
    PSVR_STEREO_TRACKER_INTRINSICS,
} PSVRTrackerIntrinsicsType;

/// Bundle containing all intrinsic camera properties
typedef struct 
{
    union {
        PSVRMonoTrackerIntrinsics mono;
        PSVRStereoTrackerIntrinsics stereo;
    } intrinsics;

    PSVRTrackerIntrinsicsType intrinsics_type;
} PSVRTrackerIntrinsics;

/// The list of possible camera drivers used by PSVRService
typedef enum
{
    PSVRVideoProperty_Brightness,
	PSVRVideoProperty_Contrast,
	PSVRVideoProperty_Hue,
	PSVRVideoProperty_Saturation,
	PSVRVideoProperty_Sharpness,
	PSVRVideoProperty_Gamma,
	PSVRVideoProperty_WhiteBalance,
	PSVRVideoProperty_RedBalance,
	PSVRVideoProperty_GreenBalance,
	PSVRVideoProperty_BlueBalance,
	PSVRVideoProperty_Gain,
	PSVRVideoProperty_Pan,
	PSVRVideoProperty_Tilt,
	PSVRVideoProperty_Roll,
	PSVRVideoProperty_Zoom,
	PSVRVideoProperty_Exposure,
	PSVRVideoProperty_Iris,
	PSVRVideoProperty_Focus,

	PSVRVideoProperty_COUNT
} PSVRVideoPropertyType;

/// Constraints on the values for a single tracker property
typedef struct
{
	int min_value;
	int max_value;
	int stepping_delta;
	int default_value;
	bool is_automatic;
	bool is_supported;
} PSVRVideoPropertyConstraint;

#define MAX_PSVR_TRACKER_MODE_NAME_LENGTH		64
#define MAX_PSVR_TRACKER_MODE_COUNT				32

/// Static properties about a tracker
typedef struct
{
    // ID of the tracker in the service
    PSVRTrackerID tracker_id;

    // Tracker USB properties
    PSVRTrackerType tracker_type;
    PSVRTrackerDriver tracker_driver;
    char device_path[128];

	// List of possible modes the tracker can be set to
	char mode_list[MAX_PSVR_TRACKER_MODE_COUNT][MAX_PSVR_TRACKER_MODE_NAME_LENGTH];
	int mode_count;

	// Constraints for each video property of the device (gain, exposure, etc)
	PSVRVideoPropertyConstraint video_property_constraints[PSVRVideoProperty_COUNT];

    // Camera Intrinsic properties
    PSVRTrackerIntrinsics tracker_intrinsics;

    // Camera Extrinsic properties
    PSVRPosef tracker_pose; ///< World space location of tracker (relative to calibration mat)
} PSVRClientTrackerInfo;

/// Dynamic properties about a tracker
typedef struct
{
    // ID of the tracker in the service
    PSVRTrackerID tracker_id;

	float frame_width;
	float frame_height;
	float frame_rate;
	int video_properties[PSVRVideoProperty_COUNT];
	
	PSVR_HSVColorRangeTable color_range_table;
} PSVRClientTrackerSettings;

/// Tracker Pool Entry
typedef struct
{
    // Tracker Static Properties
    PSVRClientTrackerInfo tracker_info;

    // Tracker Streaming State
    int listener_count;
    bool is_connected;
    int sequence_num;
    long long data_frame_last_received_time;
    float data_frame_average_fps;

    // SharedVideoFrameBuffer pointer internally
    const void *opaque_shared_video_frame_buffer;
} PSVRTracker;

/// Tracked object physics data state
typedef struct
{
    PSVRVector3f LinearVelocityCmPerSec;
    PSVRVector3f LinearAccelerationCmPerSecSqr;
    PSVRVector3f AngularVelocityRadPerSec;
    PSVRVector3f AngularAccelerationRadPerSecSqr;
    double       TimeInSeconds;
} PSVRPhysicsData;

// HMD State
//----------

/// Static properties about a Controller
typedef struct
{
    PSVRControllerID controller_id;
    PSVRControllerType controller_type;
	PSVRControllerHand controller_hand;
    PSVRTrackingColorType tracking_color_type;
    bool is_bluetooth;
	bool has_magnetometer;
    float prediction_time;
    char device_path[128];
    char orientation_filter[64];
    char position_filter[64];
	char gyro_gain_setting[64];
    char controller_serial[PSVRSERVICE_CONTROLLER_SERIAL_LEN];
	char assigned_host_serial[PSVRSERVICE_CONTROLLER_SERIAL_LEN];
} PSVRClientControllerInfo;

/// Static properties about an HMD
typedef struct
{
    PSVRHmdID hmd_id;
    PSVRHmdType hmd_type;
    PSVRTrackingColorType tracking_color_type;
    char device_path[128];
    char orientation_filter[64];
    char position_filter[64];
    float prediction_time;
} PSVRClientHMDInfo;

/// Morpheus Raw IMU sensor data
typedef struct
{
    PSVRVector3i Accelerometer;
    PSVRVector3i Gyroscope;
    double      TimeInSeconds;
} PSVRMorpheusRawSensorData;

/// Morpheus Calibrated IMU sensor data
typedef struct
{
    PSVRVector3f Accelerometer;
    PSVRVector3f Gyroscope;
    double      TimeInSeconds;
} PSVRMorpheusCalibratedSensorData;

/// Morpheus HMD State in HMD Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    
    PSVRPosef                     Pose;
    PSVRPhysicsData               PhysicsData;
    PSVRMorpheusRawSensorData     RawSensorData;
    PSVRMorpheusCalibratedSensorData CalibratedSensorData;
    PSVRRawTrackerData            RawTrackerData;
} PSVRMorpheus;

/// Virtual HMD State in HMD Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    
    PSVRPosef                     Pose;
    PSVRPhysicsData               PhysicsData;
    PSVRRawTrackerData            RawTrackerData;
} PSVRVirtualHMD;

/// HMD Pool Entry
typedef struct
{
    PSVRHmdID HmdID;
    PSVRHmdType HmdType;
    union
    {
        PSVRMorpheus  MorpheusState;
        PSVRVirtualHMD VirtualHMDState;
    }               HmdState;
    bool            bValid;
    int             OutputSequenceNum;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
    int             ListenerCount;
} PSVRHeadMountedDisplay;

// Controller State
//------------------

/// Raw Sensor data from the PSMove IMU
typedef struct
{
    PSVRVector3i Magnetometer;
    PSVRVector3i Accelerometer;
    PSVRVector3i Gyroscope;
    double      TimeInSeconds;
} PSVRPSMoveRawSensorData;

/// Calibrated Sensor
typedef struct
{
    PSVRVector3f Magnetometer;
    PSVRVector3f Accelerometer;
    PSVRVector3f Gyroscope;
    double      TimeInSeconds;
} PSVRPSMoveCalibratedSensorData;

/// PSMove Controller State in Controller Pool Entry
typedef struct
{
    bool                         bHasValidHardwareCalibration;
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    bool                         bHasUnpublishedState;
    
    char                         DevicePath[256];
    char                         DeviceSerial[128];
    char                         AssignedHostSerial[128];
    bool                         PairedToHost;
    PSVRConnectionType            ConnectionType;
    
    PSVRTrackingColorType         TrackingColorType;
    PSVRPosef                     Pose;
    PSVRPhysicsData               PhysicsData;
    PSVRPSMoveRawSensorData          RawSensorData;
    PSVRPSMoveCalibratedSensorData   CalibratedSensorData;
    PSVRRawTrackerData            RawTrackerData;
    
    PSVRButtonState               TriangleButton;
    PSVRButtonState               CircleButton;
    PSVRButtonState               CrossButton;
    PSVRButtonState               SquareButton;
    PSVRButtonState               SelectButton;
    PSVRButtonState               StartButton;
    PSVRButtonState               PSButton;
    PSVRButtonState               MoveButton;
    PSVRButtonState               TriggerButton;
    PSVRBatteryState              BatteryValue;
    unsigned char                TriggerValue;
    unsigned char                Rumble;
    unsigned char                LED_r, LED_g, LED_b;

    long long                    ResetPoseButtonPressTime;
    bool                         bResetPoseRequestSent;
    bool                         bPoseResetButtonEnabled;
    
} PSVRPSMove;

/// DualShock4 raw IMU sensor data
typedef struct
{
    PSVRVector3i Accelerometer;
    PSVRVector3i Gyroscope;
    double      TimeInSeconds;
} PSVRDS4RawSensorData;

/// DualShock4 calibrated IMU sensor data
typedef struct
{
    PSVRVector3f Accelerometer;
    PSVRVector3f Gyroscope;
    double      TimeInSeconds;
} PSVRDS4CalibratedSensorData;

/// DualShock4 Controller State in Controller Pool Entry
typedef struct
{
    bool                         bHasValidHardwareCalibration;
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    bool                         bHasUnpublishedState;
    
    char                         DevicePath[256];
    char                         DeviceSerial[128];
    char                         AssignedHostSerial[128];
    bool                         PairedToHost;
    PSVRConnectionType            ConnectionType;
    
    PSVRTrackingColorType         TrackingColorType;
    PSVRPosef                     Pose;
    PSVRPhysicsData               PhysicsData;
    PSVRDS4RawSensorData           RawSensorData;
    PSVRDS4CalibratedSensorData    CalibratedSensorData;
    PSVRRawTrackerData            RawTrackerData;
    
    PSVRButtonState               DPadUpButton;
    PSVRButtonState               DPadDownButton;
    PSVRButtonState               DPadLeftButton;
    PSVRButtonState               DPadRightButton;

    PSVRButtonState               SquareButton;
    PSVRButtonState               CrossButton;
    PSVRButtonState               CircleButton;
    PSVRButtonState               TriangleButton;

    PSVRButtonState               L1Button;
    PSVRButtonState               R1Button;
    PSVRButtonState               L2Button;
    PSVRButtonState               R2Button;
    PSVRButtonState               L3Button;
    PSVRButtonState               R3Button;

    PSVRButtonState               ShareButton;
    PSVRButtonState               OptionsButton;

    PSVRButtonState               PSButton;
    PSVRButtonState               TrackPadButton;

    float                        LeftAnalogX;
    float                        LeftAnalogY;
    float                        RightAnalogX;
    float                        RightAnalogY;
    float                        LeftTriggerValue;
    float                        RightTriggerValue;

    unsigned char                BigRumble, SmallRumble;
    unsigned char                LED_r, LED_g, LED_b;

    long long                    ResetPoseButtonPressTime;
    bool                         bResetPoseRequestSent;
    bool                         bPoseResetButtonEnabled;
    
} PSVRDualShock4;

/// Controller Pool Entry
typedef struct
{
    PSVRControllerID ControllerID;
    PSVRControllerType ControllerType;
	PSVRControllerHand ControllerHand;
    union
    {
        PSVRPSMove PSMoveState;
		PSVRDualShock4 PSDS4State;
    }               ControllerState;
    bool            bValid;
    int             OutputSequenceNum;
    int             InputSequenceNum;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
    int             ListenerCount;
} PSVRController;

// Service Events
//------------------

typedef enum 
{
	PSVREvent_controllerListUpdated,
    PSVREvent_trackerListUpdated,
    PSVREvent_hmdListUpdated,
	PSVREvent_pairControllerComplete,
	PSVREvent_pairControllerProgress,
	PSVREvent_unpairControllerComplete,
} PSVREventType;


typedef struct 
{
	PSVRResult result_code;
} PSVRPairControllerResult;

typedef struct 
{
	PSVRControllerID controller_id;
	int steps_completed;
	int total_steps;
} PSVRPairControllerProgress;

typedef struct 
{
	PSVRResult result_code;
} PSVRUnpairControllerResult;

/// A container for all PSVRService events
typedef struct
{
    PSVREventType event_type;
	union{
		PSVRPairControllerResult pair_controller_result;
		PSVRPairControllerProgress pair_controller_progress;
		PSVRUnpairControllerResult unpair_controller_result;
	} event_parameters;
} PSVREventMessage;

// Service Responses
//------------------

/// Current version of PSVRService
typedef struct
{
	char version_string[PSVRSERVICE_MAX_VERSION_STRING_LEN];
} PSVRServiceVersion;

/// List of controllers attached to PSMoveService
typedef struct
{
	char host_serial[PSVRSERVICE_CONTROLLER_SERIAL_LEN];
	PSVRClientControllerInfo controllers[PSVRSERVICE_MAX_CONTROLLER_COUNT];
    int count;
} PSVRControllerList;

/// List of trackers connected to PSVRService
typedef struct
{
    PSVRClientTrackerInfo trackers[PSVRSERVICE_MAX_TRACKER_COUNT];
    int count;
    float global_forward_degrees;
} PSVRTrackerList;

/// List of HMDs connected to PSVRSerivce
typedef struct
{
    PSVRClientHMDInfo hmds[PSVRSERVICE_MAX_HMD_COUNT];
    int count;
} PSVRHmdList;

/// Tracking Space Parameters
typedef struct
{
    float global_forward_degrees;
} PSVRTrackingSpace;

// Interface
//----------

// Blocking Connection Methods
/** \brief Initializes a connection to PSVRService.
 Attempts to initialize PSVRervice. 
 This function must be called before calling any other client functions. 
 Calling this function again after a connection is already started will return PSVRResult_Success.

 \param log_level The level of logging to emit
 \returns PSVRResult_Success on success, PSVRResult_Timeout, or PSVRResult_Error on a general connection error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_Initialize(PSVRLogSeverityLevel log_level); 

/** \brief Shuts down connection to PSVRService
 Shuts down PSVRService. 
 This function should be called when closing down the client.
 Calling this function again after a connection is alread closed will return PSVRResult_Error.

  \returns PSVRResult_Success on success or PSVRResult_Error if there was no valid connection.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_Shutdown();

// Update
/** \brief Poll the connection and process messages.
	This function will poll the connection for new messages from PSVRService.
	If new events are received they are processed right away and the the appropriate status flag will be set.
	The following state polling functions can be called after an update:
	  - \ref PSVR_GetIsInitialized()
	  - \ref PSVR_HasTrackerListChanged()
	  - \ref PSVR_HasHMDListChanged()
	  - \ref PSVR_WasSystemButtonPressed()
	  
	\return PSVRResult_Success if initialize or PSVRResult_Error otherwise
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_Update();

/** \brief Poll the connection and DO NOT process messages.
	This function will poll the connection for new messages from PSVRService.
	If new events are received they are put in a queue. The messages are extracted using \ref PSVR_PollNextMessage().
	Messages not read from the queue will get cleared on the next update call. 
	
	\return PSVRResult_Success if there is an active connection or PSVRResult_Error if there is no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_UpdateNoPollEvents();

// System State Queries
/** \brief Get the API initialization status
	\return true if the client API is initialized
 */
PSVR_PUBLIC_FUNCTION(bool) PSVR_GetIsInitialized();

/** \brief Get the controller list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get controller
	list change events.
	
	\return true if the controller list changed
 */
PSVR_PUBLIC_FUNCTION(bool) PSVR_HasControllerListChanged();

/** \brief Get the tracker list change flag
	This flag is only filled in when \ref PSVR_Update() is called.
	If you instead call PSVR_UpdateNoPollMessages() you'll need to process the event queue yourself to get tracker
	list change events.
	
	\return true if the tracker list changed
 */
PSVR_PUBLIC_FUNCTION(bool) PSVR_HasTrackerListChanged();

/** \brief Get the HMD list change flag
	This flag is only filled in when \ref PSVR_Update() is called.
	If you instead call PSVR_UpdateNoPollMessages() you'll need to process the event queue yourself to get HMD
	list change events.
	
	\return true if the HMD list changed
 */
PSVR_PUBLIC_FUNCTION(bool) PSVR_HasHMDListChanged();

// System Queries
/** \brief Get the client API version string from PSVRService
	\param[out] out_version_string The string buffer to write the version into
	\param max_version_string The size of the output buffer
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetVersionString(char *out_version_string, size_t max_version_string);

// Message Handling API
/** \brief Retrieve the next message from the message queue.
	A call to \ref PSVR_UpdateNoPollMessages will queue messages received from PSVRService.
	Use this function to processes the queued event and response messages one by one.
	If a response message does not have a callback registered with \ref PSVR_RegisterCallback it will get returned here.	
	\param[out] out_messaage The next \ref PSVREventMessage read from the incoming message queue.
	\param message_size The size of the message structure. Pass in sizeof(PSVREventMessage).
	\return PSVRResult_Success or PSVRResult_NoData if no more messages are available.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_PollNextMessage(PSVREventMessage *out_message, size_t message_size);

// Controller Pool
/** \brief Fetches the \ref PSVRController data for the given controller
	The client API maintains a pool of controller structs. 
	We can fetch a given controller by \ref PSVRControllerID.
	DO NOT DELETE the controller pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param controller_id The id of the controller structure to fetch
	\return A pointer to a \ref PSMController
 */
PSVR_PUBLIC_FUNCTION(PSVRController *) PSVR_GetController(PSVRControllerID controller_id);

/** \brief Allocate a reference to a controller.
	This function tells the client API to increment a reference count for a given controller.
	This function should be called before fetching the controller data using \ref PSM_GetController.
	When done with the controller, make sure to call \ref PSM_FreeControllerListener.
	\param controller_id The id of the controller we want to allocate a listener for
	\return PSMResult_Success if a valid controller id is given
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_AllocateControllerListener(PSVRControllerID controller_id);

/** \brief Free a reference to a controller
	This function tells the client API to decrement a reference count for a given controller.
	\param controller_id The of of the controller we want to free the listener for.
	\return PSMResult_Success if a valid controller id is given that has a non-zero ref count
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_FreeControllerListener(PSVRControllerID controller_id);

// Controller Requests
/** \brief Requests a list of the streamable controllers currently connected to PSVRService.
	Sends a request to PSVRService to get the list of currently streamable controllers.
	\param include_usb If true, we also include usb connected controllers in this list
	\param[out] out_controller_list The controller list to write the result into.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerList(const bool include_usb, PSVRControllerList *out_controller_list);

/** \brief Requests start of a data stream for a given controller
	Asks PSVRService to start stream data for the given controller with the given set of stream properties.
	The data in the associated \ref PSVRController state will get updated automatically in calls to \ref PSVR_Update or 
	\ref PSVR_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param controller_id The id of the controller to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSVRStreamFlags_defaultStreamOptions = minimal controller stream info
		- PSVRStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSVRStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSVRStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSVRStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSVRStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSVRStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StartControllerDataStream(PSVRControllerID controller_id, unsigned int data_stream_flags);

/** \brief Requests stop of an unreliable(udp) data stream for a given controller
	Asks PSVRService to start stream data for the given controller with the given set of stream properties.
	The data in the associated \ref PSVRController state will get updated automatically in calls to \ref PSVR_Update or 
	\ref PSVR_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Blocking - Returns after either stream start response comes back OR the timeout period is reached. 
	\param controller_id The id of the controller to start the stream for.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error. */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StopControllerDataStream(PSVRControllerID controller_id);

/** \brief Requests changing the tracking color type of a given controller.
	Sends a request to PSVRService to change the tracking color of a controller.
	If another controller already is using the color being assigned to this controller, it will be assigned an available color.
	\remark Blocking - Returns after either the new color is set OR the timeout period is reached. 
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerLEDTrackingColor(PSVRControllerID controller_id, PSVRTrackingColorType tracking_color);

/** \brief Requests resetting the controllers current orientation.
	This request is used to reset any drift that has occured in the pose filter's orientation from the controllers true
	orientation. Resetting the controller orientation assumes that controller is currently being held in the "identity" orientation,
	which is typically pointing down the -Z axis. 
	This request is typically sent in reponse to a certain combonation of buttons being held (usually SELECT).
	\remark Blocking - Returns after either the version string is returned OR the timeout period is reached. 
	\param controller_id The ID of the whose orientation we want to reset
	\param q_pose The pose the controller is currently being held in relative to the identity pose (like straight up).
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_ResetControllerOrientation(PSVRControllerID controller_id, const PSVRQuatf *q_pose);

/** \brief Requests setting the selected tracker index for a controller
	This request is used to set the selected tracker index on a controller data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\param controller_id The ID of the controller whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\return PSVRResult_RequestSent on success or PSVRResult_Error if there was no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerDataStreamTrackerIndex(PSVRControllerID controller_id, PSVRTrackerID tracker_id);

/** \brief Requests setting the hand assigned to a controller
	This request is used to set the suggested hand for a controller.
	Hand information is used by external APIs and not by PSVRService.
	No restrictions are made about which hands are assigned to a given controller.
	\param controller_id The ID of the controller whose data stream we want to modify
    \param hand The hand to assign to a controller (Any, Left or Right)
	\return PSVRResult_RequestSent on success or PSVRResult_Error if there was no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerHand(PSVRControllerID controller_id, PSVRControllerHand hand);

/** \brief Requests setting the acceleration calibration settings for a controller
	This request is used to set the accelerometer calibration settings for a controller.
	\param controller_id The ID of the controller whose accelerometer settings we want to modify
    \param noise_radius The radius of the measured accelerometer readings when stationary
	\param noise_variance The variance of the measures accelerometer readings when stationary
	\return PSVRResult_RequestSent on success or PSVRResult_Error if there was no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerAccelerometerCalibration(PSVRControllerID controller_id, float noise_radius, float noise_variance);

/** \brief Requests setting the acceleration calibration settings for a controller
	This request is used to set the accelerometer calibration settings for a controller.
	\param controller_id The ID of the controller whose accelerometer settings we want to modify
    \param drift The radius of the measured accelerometer readings when stationary
	\param variance The variance of the measures accelerometer readings when stationary
	\param gain_setting The Dualshock 4 gain setting preset: "125deg/s", "250deg/s", "500deg/s", "1000deg/s" or "2000deg/s"
	\return PSVRResult_RequestSent on success or PSVRResult_Error if there was no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerGyroscopeCalibration(PSVRControllerID controller_id, float drift, float variance, const char *gain_setting);

/** \brief Selects the position filter used by the given controller
	\param controller_id The ID of the controller whose position filter we want to set
    \param position_filter The string name of the position filter to set
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the filter type was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerPositionFilter(PSVRControllerID controller_id, const char *position_filter);

/** \brief Selects the orientation filter used by the given controller
	\param controller_id The ID of the controller whose position filter we want to set
    \param orientation_filter The string name of the orientation filter to set
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the filter type was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerOrientationFilter(PSVRControllerID controller_id, const char *orientation_filter);

/** \brief Sets the amount of prediction to use when computing final controller pose
	\param controller_id The ID of the controller whose position filter we want to set
    \param prediction_time The prediction time in seconds
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the hmd was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerPredictionTime(PSVRControllerID controller_id, float prediction_time);

// Controller State Methods
/** \brief Get the current orientation of a controller
	\param controller_id The id of the controller
	\param[out] out_orientation The orientation of the controller
	\return PSVRResult_Success if controller has a valid orientation
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerOrientation(PSVRControllerID controller_id, PSVRQuatf *out_orientation);

/** \brief Get the current position of a controller
	\param controller_id The id of the controller
	\param[out] out_position The position of the controller
	\return PSVRResult_Success if controller has a valid position
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerPosition(PSVRControllerID controller_id, PSVRVector3f *out_position);

/** \brief Get the current pose (orienation and position) of a controller
	\param controller_id The id of the controller
	\param[out] out_pose The pose of the controller
	\return PSVRResult_Success if controller has a valid pose
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerPose(PSVRControllerID controller_id, PSVRPosef *out_pose);

/** \brief Get the current rumble fraction of a controller
	\param controller_id The id of the controller
	\param channel The channel to get the rumble for. The PSVRove has one channel. The DualShock4 has two.
	\param[out] out_rumble_fraction The 0.0-1.0 fraction of rumble the controller is currently set to
	\return PSVRResult_Success if controller has a valid rumble state.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerRumble(PSVRControllerID controller_id, PSVRControllerRumbleChannel channel, float *out_rumble_fraction);

/** \brief Helper used to tell if the controller is upright on a level surface.
	This method is used as a calibration helper when you want to get a number of controller samples. 
	Often in this instance you want to make sure the controller is sitting upright on a table.
	\param controller_id The id of the controller
	\param[out] out_is_stable True if the controller is stable and upright.
	\return PSVRResult_Success if controller can be tested for stability.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetIsControllerStable(PSVRControllerID controller_id, bool *out_is_stable);

/** \brief See if the controller is currently being tracked by at least one tracking camera.
	\param controller_id The id of the controller
	\param[out] out_is_tracking True if the controller is currently tracking
	\return PSVRResult_Success if controller can be tracked at all.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetIsControllerTracking(PSVRControllerID controller_id, bool *out_is_tracking);

/** \brief Helper function for getting the tracking centroid for a controller on a given tracker
	Each tracking camera sees a projection of a controllers tracking light. 
	This method gets the pixel centroid of controller projection.
	\param controller_id The controller id to get the tracking projection location
	\param projection_index The index of the left or right projection (for stereo trackers) or 0 for mono trackers
	\param tracker_id The tracker id of the tracker that has the controller projection we care about.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_location The center pixel location of the controller projection on the tracker.
	\return PSVRResult_Success if controller has a valid projection on the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerPixelLocationOnTracker(PSVRControllerID controller_id, PSVRTrackingProjectionCount projection_index, PSVRTrackerID *out_tracker_id, PSVRVector2f *out_location);

/** \brief Helper function for getting the tracker relative 3d position of the controller.
	Each tracking camera can compute a estimate of the controllers 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param controller_id The controller id to get the tracking position for.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_position Tracker relative centroid position of controller in cm.
	\return PSVRResult_Success if controller has a valid position relative to the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerPositionOnTracker(PSVRControllerID controller_id, PSVRTrackerID *out_tracker_id, PSVRVector3f *outPosition);

/** \brief Helper function for getting the tracker relative 3d orientation of the controller.
	Each tracking camera can compute a estimate of the controllers 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param controller_id The controller id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_orientation Tracker relative centroid orientation of controller.
	\return PSVRResult_Success if controller has a valid optical orientation relative to the tracker (PSVRove can't, DS4 can).
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerOrientationOnTracker(PSVRControllerID controller_id, PSVRTrackerID *out_tracker_id, PSVRQuatf *outOrientation);

/** \brief Helper function for getting the tracker relative projection of a controller
	Each tracking camera can have a projection of the controller.
	This method gets the pixel geomtry of that projection.
	For a PSVRoveController this is an ellipse.
	For a DualShock4 this is a quad.
	\param controller_id The controller id to get the tracking projection for.
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_projection The tracking projection shape of the controller.
	\return PSVRResult_Success if controller has a valid projection on the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetControllerProjectionOnTracker(PSVRControllerID controller_id, PSVRTrackerID *out_tracker_id, PSVRTrackingProjection *out_projection);

/** \brief Sets a temporary RGB override for the controller's light
	The light color override will be sent on the outbound controller UDP stream.
	Tracking will not run when a light color override is set.
	The override can be cleared by setting the override color to (0, 0, 0)
	\param controller_id The controller whose light override value we want to set.
	\param r The red color override, range [0. 255]
	\param g The green color override, range [0. 255]
	\param b The blue color override, range [0. 255]
	\return PSVRResult_Success if the controller can have a light color override set
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerLEDOverrideColor(PSVRControllerID controller_id, unsigned char r, unsigned char g, unsigned char b);

/** \brief Sets the controller rumble fraction
	The controller rumble is set on the outbound UDP stream.
	The rumble for the controller stays on this setting until you change it.
	\param controller_id The id of the controller to set the rumble for
	\param channel The channel for the rumble (PSVRove has one channel, DS4 has two channels)
	\param rumble_fraction A rumble value in the range 0.0 to 1.0
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetControllerRumble(PSVRControllerID controller_id, PSVRControllerRumbleChannel channel, float rumble_fraction);

// Tracker Pool
/** \brief Fetches the \ref PSVRTracker data for the given tracker
	The client API maintains a pool of tracker structs. 
	We can fetch a given tracker by \ref PSVRTrackerID.
	DO NOT DELETE the tracker pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param tracker_id The id of the tracker structure to fetch
	\return A pointer to a \ref PSVRTracker
 */
PSVR_PUBLIC_FUNCTION(PSVRTracker *) PSVR_GetTracker(PSVRTrackerID tracker_id);

/** \brief Allocate a reference to a tracker.
	This function tells the client API to increment a reference count for a given tracker.
	This function should be called before fetching the tracker data using \ref PSVR_GetTracker.
	When done with the tracker, make sure to call \ref PSVR_FreeTrackerListener.
	\param tracker_id The id of the tracker we want to allocate a listener for
	\return PSVRResult_Success if a valid tracker id is given
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_AllocateTrackerListener(PSVRTrackerID tracker_id, const PSVRClientTrackerInfo *tracker_info);

/** \brief Free a reference to a tracker
	This function tells the client API to decrement a reference count for a given tracker.
	\param tracker_id The of of the tracker we want to free the listener for.
	\return PSVRResult_Success if a valid tracker id is given that has a non-zero ref count
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_FreeTrackerListener(PSVRTrackerID controller_id);

// Tracker State Methods
/** \brief Get the dimensions of the camera frame for a tracker in pixels
	\param tracker_id The id of the tracker
	\param[out] out_screen_size The width and height of one tracker frame
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerScreenSize(PSVRTrackerID tracker_id, PSVRVector2f *out_screen_size);

/** \brief Extract the camera intrinsics for the given tracker
	\param tracker_id The id of the tracker
	\param[out] out_intrinsics The full set of camera intrinsics for the tracker
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerIntrinsics(PSVRTrackerID tracker_id, PSVRTrackerIntrinsics *out_intrinsics);

/** \brief Set the camera intrinsics for the given tracker
	\param tracker_id The id of the tracker
	\param intrinsics The full set of camera intrinsics for the tracker
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetTrackerIntrinsics(PSVRTrackerID tracker_id, PSVRTrackerIntrinsics *intrinsics);

/** \brief Get the tracker config settings for the given tracker and hmd
	\param tracker_id The id of the tracker
    \param hmd_id The id of the hmd
	\param[out] out_settings The settings for the tracker
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerSettings(PSVRTrackerID tracker_id, PSVRHmdID hmd_id, PSVRClientTrackerSettings *out_settings);

/** \brief Restores tracker settings from the config file
	\param tracker_id The id of the tracker
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_ReloadTrackerSettings(PSVRTrackerID tracker_id);

/** \brief Sets the HSV color filter parameters for the given tracking color
    \param tracker_id The id of the tracker
	\param hmd_id The ID of the HMD whose color filters we want to modify
    \param tracking_color_type The filter color ID we want to modify
    \param desired_color_filter The desired HSV filter
    \param out_color_filter The actual resulting HSV filter
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the color was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetTrackerColorFilter(
    PSVRTrackerID tracker_id, PSVRHmdID HmdID, PSVRTrackingColorType tracking_color_type,
    PSVR_HSVColorRange *desired_color_filter, PSVR_HSVColorRange *out_color_filter);

// Tracker Methods
/** \brief Requests a list of the trackers currently connected to PSVRService.
	Sends a request to PSVRService to get the list of trackers.
	\remark Blocking - Returns after either the tracker list is returned OR the timeout period is reached. 
	\param[out] out_tracker_list The tracker list to write the result into.
	\return A zero-terminated version string of the format "Product.Major-Phase Minor.Release.Hotfix", ex: "0.9-alpha 8.1.0"
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerList(PSVRTrackerList *out_tracker_list);

/** \brief Requests start of a shared memory video stream for a given tracker
	Asks PSVRService to start a video stream for the given tracker.
	PSVRService will then start writing the video stream to a shared memory buffer.
	The data in the associated \ref PSVRTracker state will get updated automatically in calls to \ref PSVR_Update or 
	\ref PSVR_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Video streams can only be started on clients that run on the same machine as PSVRService is running on.
	\param tracker_id The id of the tracker to start the stream for.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StartTrackerDataStream(PSVRTrackerID tracker_id);

/** \brief Requests stop of a shared memory video stream for a given tracker
	Asks PSVRService to stop an active video stream for the given tracker.
	\remark Video streams can only be started on clients that run on the same machine as PSVRService is running on.
	\param tracker_id The id of the tracker to start the stream for.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StopTrackerDataStream(PSVRTrackerID tracker_id);

/** \brief Request the tracking space settings
	Sends a request to PSVRService to get the tracking space settings for PSVRService.
	The settings contain the direction of global forward (usually the -Z axis)
	\param out_tracking_space The \ref PSVRTrackingSpace settings for PSVRService
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackingSpaceSettings(PSVRTrackingSpace *out_tracking_space);

/** \brief Opens the tracker video stream buffer on the client.
	Starts reading tracker video stream from a shared memory buffer.
	A call to \ref PSVR_StartTrackerDataStream must be done first to open the video stream on PSVRServices end.
	\param tracker_id The id of the tracker we wish to open the video stream for.
	\return PSVRResult_Success if the shared memory buffer was activated by PSVRService.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_OpenTrackerVideoStream(PSVRTrackerID tracker_id);

/** \brief Closes the tracker video stream buffer on the client.
	Stops reading tracker video stream from a shared memory buffer.
	A call to \ref PSVR_StopTrackerDataStream must be done after closing the video stream.
	\param tracker_id The id of the tracker we wish to close the video stream for.
	\return PSVRResult_Success if the shared memory buffer was active.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_CloseTrackerVideoStream(PSVRTrackerID tracker_id);

/** \brief Set the frame rate of the target tracker
	\param tracker_id The id of the tracker
    \param[out] out_mode The name of the current mode
	\param max_mode_name_size The size of the out_mode string
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerMode(PSVRTrackerID tracker_id, char *out_mode, size_t max_mode_name_size);

/** \brief Set the frame rate of the target tracker
	\param tracker_id The id of the tracker
    \param new_mode The desired mode to use for the tracker
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetTrackerMode(PSVRTrackerID tracker_id, const char *new_mode);

/** \brief Set the video property of the target tracker
	\param tracker_id The id of the tracker
	\param property_type The video property to adjust
    \param desired_value The desired value of the video property for the tracker
    \param save_setting If true the desired value is saved to the tracker config
    \param[out] out_value The actual resulting value applied to the property
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetTrackerVideoProperty(PSVRTrackerID tracker_id, PSVRVideoPropertyType property_type, int desired_value, bool save_setting, int *out_value);

/** \brief Get the number of sections contained in a video buffer for the given tracker
	\remark This will be 2 for stereo cameras and 1 for mono camera. Sections are assumed to have the same dimensions.
	\param tracker_id The tracker to poll the next video frame from
	\param[out] out_section_count The section count of the video frame
	\return PSVRResult_Success if there was frame data available to read
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerVideoFrameSectionCount(PSVRTrackerID tracker_id, int *out_section_count); 

/** \brief Fetch the next video frame buffer from an opened tracker video stream
	\remark Make sure the video buffer is large enough to hold tracker dimension x 3 bytes.
	\param tracker_id The tracker to poll the next video frame from
    \param section_index The portion of the video buffer desired (0 or 1 for stereo cameras, 0 for mono camera)
	\param[out] out_buffer A pointer to the buffer to copy the video frame into
	\return PSVRResult_Success if there was frame data available to read
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerVideoFrameBuffer(PSVRTrackerID tracker_id, PSVRVideoFrameSection section_index, const unsigned char **out_buffer); 

/** \brief Helper function to fetch tracking frustum properties from a tracker
	\param The id of the tracker we wish to get the tracking frustum properties for
	\param out_frustum The tracking frustum properties to write the result into
	\return PSVRResult_Success if the tracker state is valid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerFrustum(PSVRTrackerID tracker_id, PSVRFrustum *out_frustum);

/** \brief Get the global debug flags for the tracker system
	\param[out] debug_flags Bitmask of currently set debug flags
	\return PSVRResult_Success if the debug flags could be fetched
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetTrackerDebugFlags(PSVRTrackerDebugFlags *out_debug_flags);

/** \brief Set the global debug flags for the tracker system
	\param  debug_flags Bitmask of debug flags to get
	\return PSVRResult_Success if the debug flags could be set
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetTrackerDebugFlags(PSVRTrackerDebugFlags debug_flags);

// HMD Pool
/** \brief Fetches the \ref PSVRHeadMountedDisplay data for the given HMD
	The client API maintains a pool of HMD structs. 
	We can fetch a given HMD by \ref PSVRHmdID
	DO NOT DELETE the HMD pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param hmd_id The id of the hmd structure to fetch
	\return A pointer to a \ref PSVRHeadMountedDisplay
 */
PSVR_PUBLIC_FUNCTION(PSVRHeadMountedDisplay *) PSVR_GetHmd(PSVRHmdID hmd_id);

/** \brief Allocate a reference to an HMD.
	This function tells the client API to increment a reference count for a given HMD.
	This function should be called before fetching the hmd data using \ref PSVR_GetHmd.
	When done with the HMD, make sure to call \ref PSVR_FreeHmdListener.
	\param hmd_id The id of the HMD we want to allocate a listener for
	\return PSVRResult_Success if a valid controller id is given
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_AllocateHmdListener(PSVRHmdID hmd_id);

/** \brief Free a reference to an HMD
	This function tells the client API to decrement a reference count for a given HMD.
	\param hmd_id The id of the HMD we want to free the listener for.
	\return PSVRResult_Success if a valid HMD id is given that has a non-zero ref count
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_FreeHmdListener(PSVRHmdID hmd_id);

// HMD State Methods
/** \brief Get the current orientation of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_orientation The orientation of the HMD
	\return PSVRResult_Success if HMD has a valid orientation
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdOrientation(PSVRHmdID hmd_id, PSVRQuatf *out_orientation);

/** \brief Get the current position of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_position The position of the HMD
	\return PSVRResult_Success if HMD has a valid position
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdPosition(PSVRHmdID hmd_id, PSVRVector3f *out_position);

/** \brief Get the current pose (orienation and position) of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_pose The pose of the HMD
	\return PSVRResult_Success if HMD has a valid pose
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdPose(PSVRHmdID hmd_id, PSVRPosef *out_pose);

/** \brief Helper used to tell if the HMD is upright on a level surface.
	This method is used as a calibration helper when you want to get a number of HMD samples. 
	Often in this instance you want to make sure the HMD is sitting upright on a table.
	\param hmd_id The id of the HMD
	\param[out] out_is_stable True if the HMD is stable and upright.
	\return PSVRResult_Success if HMD can be tested for stability.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetIsHmdStable(PSVRHmdID hmd_id, bool *out_is_stable);

/** \brief See if the HMD is currently being tracked by at least one tracking camera.
	\param hmd_id The id of the HMD
	\param[out] out_is_tracking True if the hmd is currently tracking
	\return PSVRResult_Success if hmd can be tracked at all.
 */ 
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetIsHmdTracking(PSVRHmdID hmd_id, bool *out_is_tracking);

/** \brief Helper function for getting the tracking centroid for an HMD on a given tracker
	Each tracking camera sees a projection of an HMD's tracking light(s). 
	This method gets the pixel centroid of the HMD projection.
	\param hmd_id The hmd id to get the tracking projection location
    \param projection_index The index of the left or right projection (for stereo trackers) or 0 for mono trackers
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_location The center pixel location of the HMD projection on the tracker.
	\return PSVRResult_Success if HMD has a valid projection on the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdPixelLocationOnTracker(PSVRHmdID hmd_id, PSVRTrackingProjectionCount projection_index, PSVRTrackerID *out_tracker_id, PSVRVector2f *out_location);

/** \brief Helper function for getting the tracker relative 3d position of the HMD.
	Each tracking camera can compute a estimate of the HMD's 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light(s) in tracker relative coordinates.
	\param hmd_id The hmd id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_position Tracker relative centroid position of the HMD in cm.
	\return PSVRResult_Success if the HMD has a valid position relative to the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdPositionOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *out_tracker_id, PSVRVector3f *out_position);

/** \brief Helper function for getting the tracker relative 3d orientation of the HMD
	Each tracking camera can compute a estimate of an HMDs 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param hmd_id The HMD id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_orientation Tracker relative centroid orientation of the HMD.
	\return PSVRResult_Success if HMD has a valid optical orientation relative to the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdOrientationOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *out_tracker_id, PSVRQuatf *out_orientation);

/** \brief Helper function for getting raw tracker projection state of an HMD
	Each tracking camera can have a projection of the HMD.
	This method fetches the full PSVRRawTrackerData state of that projection.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_raw_tracker_data The full set of tracker projection state for the given HMD.
	\return PSVRResult_Success if HMD has a valid projection on the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdRawTrackerData(PSVRHmdID hmd_id, PSVRRawTrackerData *out_raw_tracker_data);

/** \brief Helper function for getting the tracker relative projection of an HMD
	Each tracking camera can have a projection of the HMD.
	This method gets the pixel geometry of that projection.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_projection The tracking projection shape of the HMD.
	\return PSVRResult_Success if HMD has a valid projection on the tracker.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdProjectionOnTracker(PSVRHmdID hmd_id, PSVRTrackerID *out_tracker_id, PSVRTrackingProjection *out_projection);

/** \brief Helper function for getting the tracking shape geometry of an HMD
	An HMDs tracking geometry gets projected onto each tracker.
	The tracking geometry can be a sphere, an tracking bar, or a point cloud.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_shape The tracking shape of the HMD.
	\param timeout_ms The conection timeout period in milliseconds, usually PSVR_DEFAULT_TIMEOUT	
	\return PSVRResult_Success if HMD has a valid shape.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdTrackingShape(PSVRHmdID hmd_id, PSVRTrackingShape *out_shape, int timeout_ms);

// Blocking HMD Methods
/** \brief Requests a list of the HMDs currently connected to PSVRSERVICE.
	Sends a request to PSVRSERVICE to get the list of HMDs.
	\remark Blocking - Returns after either the HMD list is returned OR the timeout period is reached. 
	\param[out] out_hmd_list The hmd list to write the result into.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_GetHmdList(PSVRHmdList *out_hmd_list);

/** \brief Requests start of an unreliable(udp) data stream for a given HMD
	Asks PSVRService to start stream data for the given HMD with the given set of stream properties.
	The data in the associated \ref PSVRHeadMountedDisplay state will get updated automatically in calls to \ref PSVR_Update or 
	\ref PSVR_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\param hmd_id The id of the head mounted display to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSVRStreamFlags_defaultStreamOptions = minimal HMD stream info
		- PSVRStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSVRStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSVRStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSVRStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSVRStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSVRStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb(s)
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error.
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StartHmdDataStream(PSVRHmdID hmd_id, unsigned int data_stream_flags);

/** \brief Requests stop of an unreliable(udp) data stream for a given HMD
	Asks PSVRService to stop stream data for the given HMD.
	\param hmd_id The id of the HMD to start the stream for.
	\return PSVRResult_Success upon receiving result, PSVRResult_Timeoout, or PSVRResult_Error on request error. 
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_StopHmdDataStream(PSVRHmdID hmd_id);

/** \brief Requests setting the selected tracker index for an HMD
	This request is used to set the selected tracker index on an HMD data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\param hmd_id The ID of the HMD whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\return PSVRResult_RequestSent on success or PSVRResult_Error if there was no valid connection
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetHmdDataStreamTrackerIndex(PSVRHmdID hmd_id, PSVRTrackerID tracker_id);

/** \brief Selects the position filter used by the given HMD
	\param hmd_id The ID of the HMD whose position filter we want to set
    \param position_filter The string name of the position filter to set
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the filter type was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetHmdPositionFilter(PSVRHmdID hmd_id, const char *position_filter);

/** \brief Selects the orientation filter used by the given HMD
	\param hmd_id The ID of the HMD whose position filter we want to set
    \param orientation_filter The string name of the orientation filter to set
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the filter type was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetHmdOrientationFilter(PSVRHmdID hmd_id, const char *orientation_filter);

/** \brief Sets the amount of prediction to use when computing final HMD post
	\param hmd_id The ID of the HMD whose position filter we want to set
    \param prediction_time The prediction time in seconds
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the hmd was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetHmdPredictionTime(PSVRHmdID hmd_id, float prediction_time);

/** \brief Sets the assigned tracking color for the given HMD (only blue allowed for Morpheus)
	\param hmd_id The ID of the HMD whose position filter we want to set
    \param tracking_color_type The tracking color to use for this HMD
	\return PSVRResult_RequestSent on success or PSVRResult_Error if the color was invalid
 */
PSVR_PUBLIC_FUNCTION(PSVRResult) PSVR_SetHmdTrackingColorID(PSVRHmdID HmdID, PSVRTrackingColorType tracking_color_type);

/** 
@} 
*/ 

//cut_after
#endif

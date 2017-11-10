/**
\file
*/ 

#ifndef __PSMOVECLIENT_CAPI_H
#define __PSMOVECLIENT_CAPI_H
#include "PSMoveClient_export.h"
#include "ClientConstants.h"
#include "ClientGeometry_CAPI.h"
#include <stdbool.h>
//cut_before

/** 
\brief Client Interface for PSVRService
\defgroup PSMoveClient_CAPI Client Interface
\addtogroup PSMoveClient_CAPI 
@{ 
*/
 
// Wrapper Types
//--------------

/// The ID of a tracker in the tracker pool
typedef int PSMTrackerID;

/// The ID of an HMD in the HMD pool
typedef int PSMHmdID;

// Shared Constants
//-----------------

/// Result enum in response to a client API request
typedef enum
{
    PSMResult_NoData                = -2,	///< Request Returned No Data
    PSMResult_Error                 = -1, 	///< General Error Result
    PSMResult_Success               = 0,	///< General Success Result
} PSMResult;

/// The available tracking color types
typedef enum
{
    PSMTrackingColorType_Magenta,    ///< R:0xFF, G:0x00, B:0xFF
    PSMTrackingColorType_Cyan,       ///< R:0x00, G:0xFF, B:0xFF
    PSMTrackingColorType_Yellow,     ///< R:0xFF, G:0xFF, B:0x00
    PSMTrackingColorType_Red,        //</ R:0xFF, G:0x00, B:0x00
    PSMTrackingColorType_Green,      ///< R:0x00, G:0xFF, B:0x00
    PSMTrackingColorType_Blue,       ///< R:0x00, G:0x00, B:0xFF
	
	PSMTrackingColorType_MaxColorTypes
} PSMTrackingColorType;

/// The list of possible camera types tracked by PSVRService
typedef enum
{
    PSMTracker_None= -1,
    PSMTracker_PS3Eye,
	PSMTracker_PS4Camera
} PSMTrackerType;

/// The list of possible HMD types tracked by PSVRService
typedef enum
{
    PSMHmd_None= -1,
	PSMHmd_Morpheus= 0,
    PSMHmd_Virtual= 1,
} PSMHmdType;

/// The list of possible camera drivers used by PSVRService
typedef enum
{
    PSMDriver_LIBUSB,
    PSMDriver_GENERIC_WEBCAM
} PSMTrackerDriver;

// Tracker State
//--------------

/// A range of colors in the Hue-Saturation-Value color space
typedef struct
{
    PSMRangef hue_range;
	PSMRangef saturation_range;
	PSMRangef value_range;
} PSMHSVColorRange;

/// A table of color filters for each tracking color
typedef struct
{
	char table_name[64];
	CommonHSVColorRange color_presets[PSMTrackingColorType_MaxColorTypes];	
} PSMHSVColorRangeTable;

/// Device projection geometry as seen by each tracker
typedef struct
{
	/// ID of the selected tracker
    PSMTrackerID            TrackerID;
	/// Pixel position of device projection centroid on each tracker
    PSMVector2f             ScreenLocations[2];
	/// Tracker relative device 3d position on each tracker
    PSMVector3f             RelativePositionCm;
	/// Tracker relative device 3d orientation on each tracker
    PSMQuatf                RelativeOrientation;
	/// Tracker relative device projection geometry on each tracker
    PSMTrackingProjection   TrackingProjection;
	/// A bitmask of the trackers with valid projections
    unsigned int            ValidTrackerBitmask;

    // Multicam triangulated position and orientation, pre-filtered
	/// Optically derived world space position of device in cm
    PSMVector3f             MulticamPositionCm;
	/// Optically derived world space orientation of device in cm
    PSMQuatf                MulticamOrientation;
	/// Flag if the world space optical position is valid
    bool                    bMulticamPositionValid;
	/// Flag if the world space optical orientation is valid
    bool                    bMulticamOrientationValid;
} PSMRawTrackerData;

/// Radial and tangential lens distortion coefficients computed during lens lens calibration
/// See the [OpenCV Docs](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) for details
typedef struct 
{
    double k1; ///< Radial Distortion Parameter 1
    double k2; ///< Radial Distortion Parameter 2
    double k3; ///< Radial Distortion Parameter 3
    double p1; ///< Tangential Distortion Parameter 1
    double p2; ///< Tangential Distortion Parameter 2
} PSMDistortionCoefficients;

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
    PSMDistortionCoefficients distortion_coefficients;   ///< Lens distortion coefficients
    PSMMatrix3d camera_matrix;   ///< Intrinsic camera matrix containing focal lengths and principal point
} PSMMonoTrackerIntrinsics;

/// Camera intrinsic properties for a stereoscopic camera
/// See the [OpenCV Docs](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html) for details
typedef struct 
{
    // Keep these in sync with PSMMonoTrackerIntrinsics
    float pixel_width;  ///< Width of the camera buffer in pixels
    float pixel_height; ///< Height of the camera buffer in pixels
    float hfov;         ///< The horizontal field of view camera in degrees
    float vfov;         ///< The vertical field of view camera in degrees
    float znear;        ///< The distance of the near clipping plane in cm
    float zfar;         ///< The distance of the far clipping plane in cm
    PSMDistortionCoefficients left_distortion_coefficients; ///< Left lens distortion coefficients
    PSMMatrix3d left_camera_matrix; ///< Intrinsic matrix for left camera containing focal lengths and principal point
    // Keep these in sync with PSMMonoTrackerIntrinsics

    PSMDistortionCoefficients right_distortion_coefficients; ///< Right lens distortion coefficients
    PSMMatrix3d right_camera_matrix; ///< Intrinsic matrix for rotation camera containing focal lengths and principal point
    PSMMatrix3d left_rectification_rotation; ///< Rotation applied to left camera to rectify the image
    PSMMatrix3d right_rectification_rotation; ///< Rotation applied to right camera to rectify the image
    PSMMatrix34d left_rectification_projection; ///< Projection applied to left camera to rectify the image
    PSMMatrix34d right_rectification_projection; ///< Projection applied to right camera to rectify the image
    PSMMatrix3d rotation_between_cameras; ///< Rotation between the left and right cameras
    PSMVector3d translation_between_cameras; ///< Translation between the left and right camera
    PSMMatrix3d essential_matrix; ///< Transform relating points in unit coordinate space between cameras
    PSMMatrix3d fundamental_matrix; ///< Transform relating points in pixel coordinates between cameras
    PSMMatrix4d reprojection_matrix;  ///< Transform relating pixel x,y + disparity to distance from cameras
} PSMStereoTrackerIntrinsics;

/// Bundle containing all intrinsic camera properties
typedef struct 
{
    union {
        PSMMonoTrackerIntrinsics mono;
        PSMStereoTrackerIntrinsics stereo;
    } intrinsics;

    enum eTrackerIntrinsicsType
    {
        PSM_MONO_TRACKER_INTRINSICS,
        PSM_STEREO_TRACKER_INTRINSICS,
    } intrinsics_type;
} PSMTrackerIntrinsics;

/// Static properties about a tracker
typedef struct
{
    // ID of the tracker in the service
    PSMTrackerID tracker_id;

    // Tracker USB properties
    PSMTrackerType tracker_type;
    PSMTrackerDriver tracker_driver;
    char device_path[128];

    // Camera Intrinsic properties
    PSMTrackerIntrinsics tracker_intrinsics;

    // Camera Extrinsic properties
    PSMPosef tracker_pose; ///< World space location of tracker (relative to calibration mat)
} PSMClientTrackerInfo;

/// Dynamic properties about a tracker
typedef struct
{
    // ID of the tracker in the service
    PSMTrackerID tracker_id;

	float frame_width;
	float frame_height;
	float frame_rate;
	float exposure;
	float gain;
	
	PSMHSVColorRangeTable color_range_table;
} PSMClientTrackerSettings;

/// Tracker Pool Entry
typedef struct
{
    // Tracker Static Properties
    PSMClientTrackerInfo tracker_info;

    // Tracker Streaming State
    int listener_count;
    bool is_connected;
    int sequence_num;
    long long data_frame_last_received_time;
    float data_frame_average_fps;

    // SharedVideoFrameBuffer pointer internally
    void *opaque_shared_video_frame_buffer;
} PSMTracker;

// HMD State
//----------

/// Static properties about an HMD
typedef struct
{
	PSMHmdID hmd_id;
	PSMHmdType hmd_type;
	PSMTrackingColorType tracking_color_type;
	char device_path[128];
	char orientation_filter[64];
	char position_filter[64];
	float prediction_time;
} PSMClientHMDInfo;

/// Tracked object physics data state
typedef struct
{
    PSMVector3f LinearVelocityCmPerSec;
    PSMVector3f LinearAccelerationCmPerSecSqr;
    PSMVector3f AngularVelocityRadPerSec;
    PSMVector3f AngularAccelerationRadPerSecSqr;
    double       TimeInSeconds;
} PSMPhysicsData;

/// Morpheus Raw IMU sensor data
typedef struct
{
    PSMVector3i Accelerometer;
    PSMVector3i Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusRawSensorData;

/// Morpheus Calibrated IMU sensor data
typedef struct
{
    PSMVector3f Accelerometer;
    PSMVector3f Gyroscope;
    double      TimeInSeconds;
} PSMMorpheusCalibratedSensorData;

/// Morpheus HMD State in HMD Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsOrientationValid;
    bool                         bIsPositionValid;
    
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMMorpheusRawSensorData     RawSensorData;
    PSMMorpheusCalibratedSensorData CalibratedSensorData;
    PSMRawTrackerData            RawTrackerData;
} PSMMorpheus;

/// Virtual HMD State in HMD Pool Entry
typedef struct
{
    bool                         bIsTrackingEnabled;
    bool                         bIsCurrentlyTracking;
    bool                         bIsPositionValid;
    
    PSMPosef                     Pose;
    PSMPhysicsData               PhysicsData;
    PSMRawTrackerData            RawTrackerData;
} PSMVirtualHMD;

/// HMD Pool Entry
typedef struct
{
    PSMHmdID HmdID;
    PSMHmdType HmdType;
    union
    {
        PSMMorpheus  MorpheusState;
        PSMVirtualHMD VirtualHMDState;
    }               HmdState;
    bool            bValid;
    int             OutputSequenceNum;
    bool            IsConnected;
    long long       DataFrameLastReceivedTime;
    float           DataFrameAverageFPS;
    int             ListenerCount;
} PSMHeadMountedDisplay;

// Service Events
//------------------

/// A container for all PSVRService events
typedef struct
{
    enum eEventType
    {
        PSMEvent_trackerListUpdated,
        PSMEvent_hmdListUpdated
    } event_type;

} PSMEventMessage;

// Service Responses
//------------------

/// Current version of PSVRService
typedef struct
{
	char version_string[PSVRSERVICE_MAX_VERSION_STRING_LEN];
} PSMServiceVersion;

/// List of trackers connected to PSVRService
typedef struct
{
    PSMClientTrackerInfo trackers[PSVRSERVICE_MAX_TRACKER_COUNT];
    int count;
    float global_forward_degrees;
} PSMTrackerList;

/// List of HMDs connected to PSMoveSerivce
typedef struct
{
    PSMClientHMDInfo hmds[PSVRSERVICE_MAX_HMD_COUNT];
    int count;
} PSMHmdList;

/// Tracking Space Parameters
typedef struct
{
    float global_forward_degrees;
} PSMTrackingSpace;

// Interface
//----------

// Blocking Connection Methods
/** \brief Initializes a connection to PSVRService.
 Attempts to initialize PSVRervice. 
 This function must be called before calling any other client functions. 
 Calling this function again after a connection is already started will return PSMResult_Success.

 \param log_level The level of logging to emit
 \returns PSMResult_Success on success, PSMResult_Timeout, or PSMResult_Error on a general connection error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Initialize(PSVRLogSeverityLevel log_level); 

/** \brief Shuts down connection to PSVRService
 Shuts down PSVRService. 
 This function should be called when closing down the client.
 Calling this function again after a connection is alread closed will return PSMResult_Error.

  \returns PSMResult_Success on success or PSMResult_Error if there was no valid connection.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Shutdown();

// Update
/** \brief Poll the connection and process messages.
	This function will poll the connection for new messages from PSVRService.
	If new events are received they are processed right away and the the appropriate status flag will be set.
	The following state polling functions can be called after an update:
	  - \ref PSM_GetIsInitialized()
	  - \ref PSM_HasTrackerListChanged()
	  - \ref PSM_HasHMDListChanged()
	  - \ref PSM_WasSystemButtonPressed()
	  
	\return PSMResult_Success if initialize or PSMResult_Error otherwise
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_Update();

/** \brief Poll the connection and DO NOT process messages.
	This function will poll the connection for new messages from PSVRService.
	If new events are received they are put in a queue. The messages are extracted using \ref PSM_PollNextMessage().
	Messages not read from the queue will get cleared on the next update call. 
	
	\return PSMResult_Success if there is an active connection or PSMResult_Error if there is no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_UpdateNoPollEvents();

// System State Queries
/** \brief Get the client API version string 
	\return A zero-terminated version string of the format "Product.Major-Phase Minor.Release.Hotfix", ex: "0.9-alpha 8.1.0"
 */
PSM_PUBLIC_FUNCTION(const char*) PSM_GetClientVersionString();

/** \brief Get the API initialization status
	\return true if the client API is initialized
 */
PSM_PUBLIC_FUNCTION(bool) PSM_GetIsInitialized();

/** \brief Get the tracker list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get tracker
	list change events.
	
	\return true if the tracker list changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasTrackerListChanged();

/** \brief Get the HMD list change flag
	This flag is only filled in when \ref PSM_Update() is called.
	If you instead call PSM_UpdateNoPollMessages() you'll need to process the event queue yourself to get HMD
	list change events.
	
	\return true if the HMD list changed
 */
PSM_PUBLIC_FUNCTION(bool) PSM_HasHMDListChanged();

// System Queries
/** \brief Get the client API version string from PSVRService
	Sends a request to PSVRService to get the protocol version.
	This should be compared against the version returned from \ref PSM_GetClientVersionString() as a way to verify
	that the an outdated client isn't being used with PSVRService.
	\param[out] out_version_string The string buffer to write the version into
	\param max_version_string The size of the output buffer
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetServiceVersionString(char *out_version_string, size_t max_version_string);

// Message Handling API
/** \brief Retrieve the next message from the message queue.
	A call to \ref PSM_UpdateNoPollMessages will queue messages received from PSVRService.
	Use this function to processes the queued event and response messages one by one.
	If a response message does not have a callback registered with \ref PSM_RegisterCallback it will get returned here.	
	\param[out] out_messaage The next \ref PSMEventMessage read from the incoming message queue.
	\param message_size The size of the message structure. Pass in sizeof(PSMEventMessage).
	\return PSMResult_Success or PSMResult_NoData if no more messages are available.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_PollNextMessage(PSMEventMessage *out_message, size_t message_size);

// Tracker Pool
/** \brief Fetches the \ref PSMTracker data for the given tracker
	The client API maintains a pool of tracker structs. 
	We can fetch a given tracker by \ref PSMTrackerID.
	DO NOT DELETE the tracker pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param tracker_id The id of the tracker structure to fetch
	\return A pointer to a \ref PSMTracker
 */
PSM_PUBLIC_FUNCTION(PSMTracker *) PSM_GetTracker(PSMTrackerID tracker_id);

/** \brief Allocate a reference to a tracker.
	This function tells the client API to increment a reference count for a given tracker.
	This function should be called before fetching the tracker data using \ref PSM_GetTracker.
	When done with the tracker, make sure to call \ref PSM_FreeTrackerListener.
	\param tracker_id The id of the tracker we want to allocate a listener for
	\return PSMResult_Success if a valid tracker id is given
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateTrackerListener(PSMTrackerID tracker_id, const PSMClientTrackerInfo *tracker_info);

/** \brief Free a reference to a tracker
	This function tells the client API to decrement a reference count for a given tracker.
	\param tracker_id The of of the tracker we want to free the listener for.
	\return PSMResult_Success if a valid tracker id is given that has a non-zero ref count
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeTrackerListener(PSMTrackerID controller_id);

// Tracker State Methods
/** \brief Get the dimensions of the camera frame for a tracker in pixels
	\param tracker_id The id of the tracker
	\param[out] out_screen_size The width and height of one tracker frame
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerScreenSize(PSMTrackerID tracker_id, PSMVector2f *out_screen_size);

/** \brief Extract the camera intrinsics for the given tracker
	\param tracker_id The id of the tracker
	\param[out] out_intrinsics The full set of camera intrinsics for the tracker
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerIntrinsics(PSMTrackerID tracker_id, PSMTrackerIntrinsics *out_intrinsics);

// Tracker Methods
/** \brief Requests a list of the trackers currently connected to PSVRService.
	Sends a request to PSVRService to get the list of trackers.
	\remark Blocking - Returns after either the tracker list is returned OR the timeout period is reached. 
	\param[out] out_tracker_list The tracker list to write the result into.
	\return A zero-terminated version string of the format "Product.Major-Phase Minor.Release.Hotfix", ex: "0.9-alpha 8.1.0"
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerList(PSMTrackerList *out_tracker_list);

/** \brief Requests start of a shared memory video stream for a given tracker
	Asks PSVRService to start a video stream for the given tracker.
	PSVRService will then start writing the video stream to a shared memory buffer.
	The data in the associated \ref PSMTracker state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\remark Video streams can only be started on clients that run on the same machine as PSVRService is running on.
	\param tracker_id The id of the tracker to start the stream for.
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartTrackerDataStream(PSMTrackerID tracker_id);

/** \brief Requests stop of a shared memory video stream for a given tracker
	Asks PSVRService to stop an active video stream for the given tracker.
	\remark Video streams can only be started on clients that run on the same machine as PSVRService is running on.
	\param tracker_id The id of the tracker to start the stream for.
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopTrackerDataStream(PSMTrackerID tracker_id);

/** \brief Request the tracking space settings
	Sends a request to PSVRService to get the tracking space settings for PSVRService.
	The settings contain the direction of global forward (usually the -Z axis)
	\param out_tracking_space The \ref PSMTrackingSpace settings for PSVRService
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackingSpaceSettings(PSMTrackingSpace *out_tracking_space);

/** \brief Opens the tracker video stream buffer on the client.
	Starts reading tracker video stream from a shared memory buffer.
	A call to \ref PSM_StartTrackerDataStream must be done first to open the video stream on PSVRServices end.
	\param tracker_id The id of the tracker we wish to open the video stream for.
	\return PSMResult_Success if the shared memory buffer was activated by PSVRService.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_OpenTrackerVideoStream(PSMTrackerID tracker_id);

/** \brief Closes the tracker video stream buffer on the client.
	Stops reading tracker video stream from a shared memory buffer.
	A call to \ref PSM_StopTrackerDataStream must be done after closing the video stream.
	\param tracker_id The id of the tracker we wish to close the video stream for.
	\return PSMResult_Success if the shared memory buffer was active.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_CloseTrackerVideoStream(PSMTrackerID tracker_id);

/** \brief Get the number of sections contained in a video buffer for the given tracker
	\remark This will be 2 for stereo cameras and 1 for mono camera. Sections are assumed to have the same dimensions.
	\param tracker_id The tracker to poll the next video frame from
	\param[out] out_section_count The section count of the video frame
	\return PSMResult_Success if there was frame data available to read
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerVideoFrameSectionCount(PSMTrackerID tracker_id, int *out_section_count); 

/** \brief Fetch the next video frame buffer from an opened tracker video stream
	\remark Make sure the video buffer is large enough to hold tracker dimension x 3 bytes.
	\param tracker_id The tracker to poll the next video frame from
    \param section_index The portion of the video buffer desired (0 or 1 for stereo cameras, 0 for mono camera)
	\param[out] out_buffer A pointer to the buffer to copy the video frame into
	\return PSMResult_Success if there was frame data available to read
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerVideoFrameBuffer(PSMTrackerID tracker_id, PSMVideoFrameSection section_index, const unsigned char **out_buffer); 

/** \brief Helper function to fetch tracking frustum properties from a tracker
	\param The id of the tracker we wish to get the tracking frustum properties for
	\param out_frustum The tracking frustum properties to write the result into
	\return PSMResult_Success if the tracker state is valid
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetTrackerFrustum(PSMTrackerID tracker_id, PSMFrustum *out_frustum);

// HMD Pool
/** \brief Fetches the \ref PSMHeadMountedDisplay data for the given HMD
	The client API maintains a pool of HMD structs. 
	We can fetch a given HMD by \ref PSMHmdID
	DO NOT DELETE the HMD pointer returned by this function.
	It is safe to copy this pointer on to other structures so long as the pointer is cleared once the client API is shutdown.
	\param hmd_id The id of the hmd structure to fetch
	\return A pointer to a \ref PSMHeadMountedDisplay
 */
PSM_PUBLIC_FUNCTION(PSMHeadMountedDisplay *) PSM_GetHmd(PSMHmdID hmd_id);

/** \brief Allocate a reference to an HMD.
	This function tells the client API to increment a reference count for a given HMD.
	This function should be called before fetching the hmd data using \ref PSM_GetHmd.
	When done with the HMD, make sure to call \ref PSM_FreeHmdListener.
	\param hmd_id The id of the HMD we want to allocate a listener for
	\return PSMResult_Success if a valid controller id is given
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_AllocateHmdListener(PSMHmdID hmd_id);

/** \brief Free a reference to an HMD
	This function tells the client API to decrement a reference count for a given HMD.
	\param hmd_id The id of the HMD we want to free the listener for.
	\return PSMResult_Success if a valid HMD id is given that has a non-zero ref count
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_FreeHmdListener(PSMHmdID hmd_id);

// HMD State Methods
/** \brief Get the current orientation of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_orientation The orientation of the HMD
	\return PSMResult_Success if HMD has a valid orientation
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientation(PSMHmdID hmd_id, PSMQuatf *out_orientation);

/** \brief Get the current position of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_position The position of the HMD
	\return PSMResult_Success if HMD has a valid position
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPosition(PSMHmdID hmd_id, PSMVector3f *out_position);

/** \brief Get the current pose (orienation and position) of an HMD
	\param hmd_id The id of the HMD
	\param[out] out_pose The pose of the HMD
	\return PSMResult_Success if HMD has a valid pose
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPose(PSMHmdID hmd_id, PSMPosef *out_pose);

/** \brief Helper used to tell if the HMD is upright on a level surface.
	This method is used as a calibration helper when you want to get a number of HMD samples. 
	Often in this instance you want to make sure the HMD is sitting upright on a table.
	\param hmd_id The id of the HMD
	\param[out] out_is_stable True if the HMD is stable and upright.
	\return PSMResult_Success if HMD can be tested for stability.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdStable(PSMHmdID hmd_id, bool *out_is_stable);

/** \brief See if the HMD is currently being tracked by at least one tracking camera.
	\param hmd_id The id of the HMD
	\param[out] out_is_tracking True if the hmd is currently tracking
	\return PSMResult_Success if hmd can be tracked at all.
 */ 
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetIsHmdTracking(PSMHmdID hmd_id, bool *out_is_tracking);

/** \brief Helper function for getting the tracking centroid for an HMD on a given tracker
	Each tracking camera sees a projection of an HMD's tracking light(s). 
	This method gets the pixel centroid of the HMD projection.
	\param hmd_id The hmd id to get the tracking projection location
    \param projection_index The index of the left or right projection (for stereo trackers) or 0 for mono trackers
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_location The center pixel location of the HMD projection on the tracker.
	\return PSMResult_Success if HMD has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPixelLocationOnTracker(PSMHmdID hmd_id, PSMTrackingProjectionCount projection_index, PSMTrackerID *out_tracker_id, PSMVector2f *out_location);

/** \brief Helper function for getting the tracker relative 3d position of the HMD.
	Each tracking camera can compute a estimate of the HMD's 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light(s) in tracker relative coordinates.
	\param hmd_id The hmd id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_position Tracker relative centroid position of the HMD in cm.
	\return PSMResult_Success if the HMD has a valid position relative to the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdPositionOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMVector3f *out_position);

/** \brief Helper function for getting the tracker relative 3d orientation of the HMD
	Each tracking camera can compute a estimate of an HMDs 3d position relative to the tracker.
	This method gets the 3d centroid of the tracking light in tracker relative coordinates.
	\param hmd_id The HMD id to get the tracking position for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_orientation Tracker relative centroid orientation of the HMD.
	\return PSMResult_Success if HMD has a valid optical orientation relative to the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdOrientationOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMQuatf *out_orientation);

/** \brief Helper function for getting the tracker relative projection of an HMD
	Each tracking camera can have a projection of the HMD.
	This method gets the pixel geometry of that projection.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_tracker_id The id of the tracker this projection is for
	\param[out] out_projection The tracking projection shape of the HMD.
	\return PSMResult_Success if HMD has a valid projection on the tracker.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdProjectionOnTracker(PSMHmdID hmd_id, PSMTrackerID *out_tracker_id, PSMTrackingProjection *out_projection);

/** \brief Helper function for getting the tracking shape geometry of an HMD
	An HMDs tracking geometry gets projected onto each tracker.
	The tracking geometry can be a sphere, an tracking bar, or a point cloud.
	\param hmd_id The hmd id to get the tracking projection for
	\param[out] out_shape The tracking shape of the HMD.
	\param timeout_ms The conection timeout period in milliseconds, usually PSM_DEFAULT_TIMEOUT	
	\return PSMResult_Success if HMD has a valid shape.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdTrackingShape(PSMHmdID hmd_id, PSMTrackingShape *out_shape, int timeout_ms);

// Blocking HMD Methods
/** \brief Requests a list of the HMDs currently connected to PSMoveService.
	Sends a request to PSMoveService to get the list of HMDs.
	\remark Blocking - Returns after either the HMD list is returned OR the timeout period is reached. 
	\param[out] out_hmd_list The hmd list to write the result into.
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_GetHmdList(PSMHmdList *out_hmd_list);

/** \brief Requests start of an unreliable(udp) data stream for a given HMD
	Asks PSVRService to start stream data for the given HMD with the given set of stream properties.
	The data in the associated \ref PSMHeadMountedDisplay state will get updated automatically in calls to \ref PSM_Update or 
	\ref PSM_UpdateNoPollMessages.
	Requests to restart an already started stream will return an error.
	\param hmd_id The id of the head mounted display to start the stream for.
	\param data_stream_flags One or more of the following steam:
	    - PSMStreamFlags_defaultStreamOptions = minimal HMD stream info
		- PSMStreamFlags_includePositionData = add position to pose data (which turns on tracking lights)
		- PSMStreamFlags_includePhysicsData = add linear and angular velocity and acceleration
		- PSMStreamFlags_includeRawSensorData = add raw IMU sensor data values
		- PSMStreamFlags_includeCalibratedSensorData = add calibrated sensor data values
		- PSMStreamFlags_includeRawTrackerData = add tracker projection info for each tacker
		- PSMStreamFlags_disableROI = turns off RegionOfInterest optimization used to reduce CPU load when finding tracking bulb(s)
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error.
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StartHmdDataStream(PSMHmdID hmd_id, unsigned int data_stream_flags);

/** \brief Requests stop of an unreliable(udp) data stream for a given HMD
	Asks PSVRService to stop stream data for the given HMD.
	\param hmd_id The id of the HMD to start the stream for.
	\return PSMResult_Success upon receiving result, PSMResult_Timeoout, or PSMResult_Error on request error. 
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_StopHmdDataStream(PSMHmdID hmd_id);

/** \brief Requests setting the selected tracker index for an HMD
	This request is used to set the selected tracker index on an HMD data stream
    when the data stream has tracking projection data active. The projection data is
    only provided for the selected tracker.
	\param hmd_id The ID of the HMD whose data stream we want to modify
    \param tracker_id The ID of the tracker we want to assign as the active tracker
	\return PSMResult_RequestSent on success or PSMResult_Error if there was no valid connection
 */
PSM_PUBLIC_FUNCTION(PSMResult) PSM_SetHmdDataStreamTrackerIndex(PSMHmdID hmd_id, PSMTrackerID tracker_id);

/** 
@} 
*/ 

//cut_after
#endif

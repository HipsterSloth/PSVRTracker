#ifndef CLIENT_CONSTANTS_H
#define CLIENT_CONSTANTS_H

//-- includes -----
/** 
\addtogroup PSVRClient_CAPI 
@{ 
*/

//-- constants -----
typedef enum
{
    PSVRLogSeverityLevel_trace,
    PSVRLogSeverityLevel_debug,
    PSVRLogSeverityLevel_info,
    PSVRLogSeverityLevel_warning,
    PSVRLogSeverityLevel_error,
    PSVRLogSeverityLevel_fatal
} PSVRLogSeverityLevel;

/// The list of possible sub sections to extract from a video frame
typedef enum
{
    PSVRVideoFrameSection_Left                 = 0, ///< The left frame from a stereo camera
    PSVRVideoFrameSection_Right                = 1, ///< The right frame from a stereo camera
    PSVRVideoFrameSection_Primary              = 0  ///< The only frame from a stereo camera
} PSVRVideoFrameSection;

// See ControllerManager.h in PSMoveService
#define PSVRSERVICE_MAX_CONTROLLER_COUNT  5

// See TrackerManager.h in PSVRSERVICE
#define PSVRSERVICE_MAX_TRACKER_COUNT  8

// See HMDManager.h in PSVRSERVICE
#define PSVRSERVICE_MAX_HMD_COUNT  4

// The max length of the service version string
#define PSVRSERVICE_MAX_VERSION_STRING_LEN 32

// The length of a controller serial string: "xx:xx:xx:xx:xx:xx\0"
#define PSVRSERVICE_CONTROLLER_SERIAL_LEN  18

// Defines a standard _PAUSE function
#if __cplusplus >= 199711L  // if C++11
    #include <thread>
    #define _PAUSE(ms) (std::this_thread::sleep_for(std::chrono::milliseconds(ms)))
#elif defined(_WIN32)       // if windows system
    #include <windows.h>
    #define _PAUSE(ms) (Sleep(ms))
#else                       // assume this is Unix system
    #include <unistd.h>
    #define _PAUSE(ms) (usleep(1000 * ms))
#endif

// Wrapper Types
//--------------

/// The ID of a controller in the controller pool
typedef int PSVRControllerID;

/// The ID of a tracker in the tracker pool
typedef int PSVRTrackerID;

/// The ID of an HMD in the HMD pool
typedef int PSVRHmdID;

/** 
@} 
*/ 

#endif // CLIENT_CONSTANTS_H

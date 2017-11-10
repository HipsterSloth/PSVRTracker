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

// See TrackerManager.h in PSMoveService
#define PSMOVESERVICE_MAX_TRACKER_COUNT  8

// See HMDManager.h in PSMoveService
#define PSMOVESERVICE_MAX_HMD_COUNT  4

// The max length of the service version string
#define PSMOVESERVICE_MAX_VERSION_STRING_LEN 32

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

/** 
@} 
*/ 

#endif // CLIENT_CONSTANTS_H

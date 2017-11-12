// -- includes -----
#include "Utility.h"
#include <wchar.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdarg.h>
#include <locale>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>

#if defined WIN32 || defined _WIN32 || defined WINCE
    #include <windows.h>
    #include <direct.h>
    #include <algorithm>

    #ifdef _MSC_VER
        #pragma warning (disable: 4996) // 'This function or variable may be unsafe': vsnprintf
        #define vsnprintf _vsnprintf
    #endif
#else
    #include <sys/time.h>
    #include <sys/types.h>
    #include <sys/stat.h>

	#include <time.h>
	#if defined __MACH__ && defined __APPLE__
		#include <mach/mach.h>
		#include <mach/mach_time.h>
	#endif
    #define MILLISECONDS_TO_NANOSECONDS 1000000
#endif

// -- public methods -----
namespace Utility
{
    unsigned char int32_to_int8_verify(int value)
    {
        assert(value >= 0 && value <= 255);
        return static_cast<unsigned char>(value);
    }

    bool convert_wcs_to_mbs(const wchar_t *wc_string, char *out_mb_serial, const size_t mb_buffer_size)
    {
        bool success= false;

        if (wc_string != nullptr)
        {
#if defined WIN32 || defined _WIN32 || defined WINCE
            size_t countConverted;
            const wchar_t *wcsIndirectString = wc_string;
            mbstate_t mbstate;

            success= wcsrtombs_s(
                &countConverted,
                out_mb_serial,
                mb_buffer_size,
                &wcsIndirectString,
                _TRUNCATE,
                &mbstate) == 0;
#else
            success=
                wcstombs(
                    out_mb_serial, 
                    wc_string, 
                    mb_buffer_size) != static_cast<size_t>(-1);
#endif
        }

        return success;
    }

    int format_string(char *buffer, size_t buffer_size, const char *format, ...)
    {
        // Bake out the text string
        va_list args;
        va_start(args, format);
        int chars_written = vsnprintf(buffer, buffer_size, format, args);
        buffer[buffer_size - 1] = 0;
        va_end(args);

        return chars_written;
    }

#if defined WIN32 || defined _WIN32 || defined WINCE
    const DWORD MS_VC_EXCEPTION = 0x406D1388;

    #pragma pack(push,8)
    typedef struct tagTHREADNAME_INFO
    {
        DWORD dwType;		// Must be 0x1000.
        LPCSTR szName;		// Pointer to name (in user addr space).
        DWORD dwThreadID;	// Thread ID (-1=caller thread).
        DWORD dwFlags;		// Reserved for future use, must be zero.
    } THREADNAME_INFO;
    #pragma pack(pop)

    void set_current_thread_name(const char* thread_name)
    {
        THREADNAME_INFO info;
        info.dwType = 0x1000;
        info.szName = thread_name;
        info.dwThreadID = GetCurrentThreadId();
        info.dwFlags = 0;

        __try
        {
            RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
        }
        __except (EXCEPTION_EXECUTE_HANDLER)
        {
        }
    }
#else
    void set_current_thread_name(const char* threadName)
    {
        // Not sure how to implement this on linux/osx, so left empty...
    }
#endif

    void sleep_ms(int milliseconds)
    {
#ifdef _MSC_VER
        Sleep(milliseconds);
#else
        struct timespec req = {0};
        req.tv_sec = 0;
        req.tv_nsec = milliseconds * MILLISECONDS_TO_NANOSECONDS;
        nanosleep(&req, (struct timespec *)NULL);
#endif
    }

    std::string get_home_directory()
    {
        std::string home_dir;

#if defined WIN32 || defined _WIN32 || defined WINCE
        size_t homedir_buffer_req_size;
        char homedir_buffer[512];
        getenv_s(&homedir_buffer_req_size, homedir_buffer, "APPDATA");
        assert(homedir_buffer_req_size <= sizeof(homedir_buffer));
        home_dir= homedir_buffer;
#else    
        // if run as root, use system-wide data directory
        if (geteuid() == 0)
        {
            home_dir = "/etc/";
        }
        else
        {
            homedir = getenv("HOME");
        }
#endif
        return home_dir;
    }

    bool create_directory(const std::string &path)
    {
        bool bSuccess= false;

#if defined WIN32 || defined _WIN32 || defined WINCE
        if (_mkdir(path.c_str()) == 0)
        {
            bSuccess= true;
        }
#else 
        mode_t nMode = 0733; // UNIX style permissions
        if (mkdir(path.c_str(), nMode) == 0)
        {
            bSuccess= true;
        }
#endif
        else if (errno == EEXIST)
        {
            bSuccess= true;
        }

        return bSuccess;
    }

    bool file_exists(const std::string& filename) 
    {
        std::ifstream file(filename.c_str());

        return (bool)file;
    }

};
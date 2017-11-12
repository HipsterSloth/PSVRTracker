#ifndef PSVRCLIENT_EXPORT_H
#define PSVRCLIENT_EXPORT_H

/** \def PSVR_CALL
 * \ingroup psvr_client_misc
 * PSVRService's Windows calling convention.
 *
 * Under Windows, the selection of available compilers and configurations
 * means that, unlike other platforms, there is not <em>one true calling
 * convention</em> (calling convention: the manner in which parameters are
 * passed to functions in the generated assembly code).
 *
 * Matching the Windows API itself, PSVRService's client API uses the 
 * __cdecl convention and guarantees that the library is compiled in this way. 
 * The public header file also includes appropriate annotations so that 
 * your own software will use the right convention, even if another convention 
 * is being used by default within your codebase.
 *
 * On non-Windows operating systems, this macro is defined as nothing. This
 * means that you can apply it to your code without worrying about
 * cross-platform compatibility.
 */
#ifndef PSVR_CALL
    #if defined _WIN32 || defined __CYGWIN__
        #define PSVR_CALL __cdecl
    #else
        #define PSVR_CALL
    #endif
#endif

#ifndef PSVR_EXTERN_C
    #ifdef __cplusplus
        #define PSVR_EXTERN_C extern "C"
    #else
        #define PSVR_EXTERN_C
    #endif
#endif

#ifndef PSVR_PUBLIC_FUNCTION
    #if defined(PSVRClient_EXPORTS)  // CMake-defined when creating shared library
        #if defined _WIN32 || defined __CYGWIN__
            #define PSVR_PUBLIC_FUNCTION(rval)       PSVR_EXTERN_C    __declspec(dllexport)                   rval    PSVR_CALL
            #define PSVR_PUBLIC_CLASS                                __declspec(dllexport)
            #define PSVR_PRIVATE_FUNCTION(rval)                                                              rval    PSVR_CALL
            #define PSVR_PRIVATE_CLASS
        #else  // Not Windows
            #if __GNUC__ >= 4
                #define PSVR_PUBLIC_FUNCTION(rval)   PSVR_EXTERN_C    __attribute__((visibility("default")))  rval    PSVR_CALL
                #define PSVR_PUBLIC_CLASS                            __attribute__((visibility("default")))
            #else
                #define PSVR_PUBLIC_FUNCTION(rval)   PSVR_EXTERN_C rval PSVR_CALL
                #define PSVR_PUBLIC_CLASS
            #endif
            #define PSVR_PRIVATE_FUNCTION(rval)                      __attribute__((visibility("hidden")))   rval    PSVR_CALL
            #define PSVR_PRIVATE_CLASS                               __attribute__((visibility("hidden")))
        #endif  //defined _WIN32 || defined __CYGWIN__
    #elif defined(PSVRService_STATIC) // CMake-defined when creating static library
        #define PSVR_PUBLIC_FUNCTION(rval)           PSVR_EXTERN_C                                            rval    PSVR_CALL
        #define PSVR_PUBLIC_CLASS
        #define PSVR_PRIVATE_FUNCTION(rval)                                                                  rval    PSVR_CALL
        #define PSVR_PRIVATE_CLASS
    #else //This DLL/so/dylib is being imported
        #if defined _WIN32 || defined __CYGWIN__
            #define PSVR_PUBLIC_FUNCTION(rval)       PSVR_EXTERN_C    __declspec(dllimport)                   rval    PSVR_CALL
            #define PSVR_PUBLIC_CLASS                                __declspec(dllimport)
        #else  // Not Windows
            #define PSVR_PUBLIC_FUNCTION(rval)       PSVR_EXTERN_C                                            rval    PSVR_CALL
            #define PSVR_PUBLIC_CLASS
        #endif  //defined _WIN32 || defined __CYGWIN__
        #define PSVR_PRIVATE_FUNCTION(rval)                                                                  rval    PSVR_CALL
        #define PSVR_PRIVATE_CLASS
    #endif //PSVRClient_EXPORTS
#endif //!defined(PSVR_PUBLIC_FUNCTION)

/*
 For any CPP API we decide to expose:
 We want those classes/functions to NOT be exported/imported when building/using
 the C API, but we do want them exported/imported when building/using the C++ API.
 */

#if !defined(PSVR_CPP_PUBLIC_FUNCTION) && defined(PSVRSERVICE_CPP_API)
    #define PSVR_CPP_PUBLIC_FUNCTION(rval) PSVR_PUBLIC_FUNCTION(rval)
    #define PSVR_CPP_PUBLIC_CLASS PSVR_PUBLIC_CLASS
    #define PSVR_CPP_PRIVATE_FUNCTION(rval) PSVR_PRIVATE_FUNCTION(rval)
    #define PSVR_CPP_PRIVATE_CLASS PSVR_PRIVATE_CLASS
#else
    #define PSVR_CPP_PUBLIC_FUNCTION(rval) PSVR_PRIVATE_FUNCTION(rval)
    #define PSVR_CPP_PUBLIC_CLASS PSVR_PRIVATE_CLASS
    #define PSVR_CPP_PRIVATE_FUNCTION(rval) PSVR_PRIVATE_FUNCTION(rval)
    #define PSVR_CPP_PRIVATE_CLASS PSVR_PRIVATE_CLASS
#endif

#endif // PSVRCLIENT_EXPORT_H
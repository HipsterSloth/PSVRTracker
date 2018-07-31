# When not using MSVC, we recommend using system-wide libraries
# (installed via homebrew on Mac or apt-get in Linux/Ubuntu)
# In MSVC, we auto-download the source and make it an external_project

# Platform specific libraries
SET(PLATFORM_LIBS)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(IOKIT_FRAMEWORK IOKit)
    find_library(COREFOUNDATION_FRAMEWORK CoreFoundation)
    #find_library(QUARTZCORE QuartzCore)
    find_library(APPKIT_FRAMEWORK AppKit)
    #find_library(QTKIT QTKit)
    find_library(AVFOUNDATION AVFoundation)
    find_library(IOBLUETOOTH IOBluetooth)
    #stdc++ ${QUARTZCORE} ${APPKIT_FRAMEWORK} ${QTKIT} ${AVFOUNDATION}
    list(APPEND PLATFORM_LIBS
        ${COREFOUNDATION_FRAMEWORK}
        ${IOKIT_FRAMEWORK}
        ${APPKIT_FRAMEWORK}
        ${AVFOUNDATION}
        ${IOBLUETOOTH})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    #OpenCV extra dependencies: comctl32 gdi32 ole32 setupapi ws2_32 vfw32
    #hid required for HidD_SetOutputReport() in DualShock4 controller
    #setupapi required by hidapi
    #dinput8 required by libstem_gamepad
    list(APPEND PLATFORM_LIBS bthprops setupapi hid dinput8 shlwapi)
	# Media foundation plaform for USB cameras in windows
	list(APPEND PLATFORM_LIBS mfplat)
    IF(MINGW)
        #list(APPEND PLATFORM_LIBS stdc++)
    ENDIF(MINGW)
ELSE() #Linux
    list(APPEND PLATFORM_LIBS rt)
ENDIF()

# Eigen3
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # TODO: Convert this to ExternalProject_Add
    # Can manually set EIGEN3_INCLUDE_DIR to "${ROOT_DIR}/thirdparty/eigen
    MESSAGE(STATUS "Using Eigen3 in submodule")
    LIST(APPEND CMAKE_MODULE_PATH "${ROOT_DIR}/thirdparty/eigen/cmake")
    SET(ENV{EIGEN3_ROOT} "${ROOT_DIR}/thirdparty/eigen")
    find_package(Eigen3 REQUIRED)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    MESSAGE(STATUS "Using homebrew Eigen3")
    find_package(Eigen3 REQUIRED CONFIG PATHS /usr/local/opt/eigen/lib/cmake/eigen3)
ELSE()
    LIST(APPEND CMAKE_MODULE_PATH "${ROOT_DIR}/thirdparty/eigen/cmake")
    SET(ENV{EIGEN3_ROOT} "/usr/include/eigen3")
    find_package(Eigen3 REQUIRED)
ENDIF()

# OpenCV
# Override by adding "-DOpenCV_DIR=C:\path\to\opencv\build" to your cmake command
IF(NOT OpenCV_DIR)
    IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        ExternalProject_Add(opencv
          PREFIX ${ROOT_DIR}/deps/opencv
          GIT_REPOSITORY https://github.com/Armada651/opencv.git
          GIT_SHALLOW 1
          GIT_TAG msmf-media-types
          CMAKE_GENERATOR ${gen}
          CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
            -DCMAKE_INCLUDE_PATH=${ROOT_DIR}/deps/local/include
            -DCMAKE_LIBRARY_PATH=${ROOT_DIR}/deps/local/lib
            -DBUILD_WITH_STATIC_CRT:BOOL=ON
            -DBUILD_SHARED_LIBS:BOOL=OFF
            -DBUILD_DOCS:BOOL=OFF
            -DBUILD_EXAMPLES:BOOL=OFF
            -DBUILD_TESTS:BOOL=OFF
            -DBUILD_PERF_TESTS:BOOL=OFF
            #-DCMAKE_BUILD_TYPE:STRING=Release
            -DWITH_FFMPEG:BOOL=OFF
            -DWITH_OPENEXR:BOOL=OFF
            -DWITH_JASPER:BOOL=OFF
            -DWITH_TIFF:BOOL=OFF
            -DWITH_IPP:BOOL=OFF
			-DWITH_CUDA:BOOL=OFF
			-DWITH_MSMF:BOOL=ON
            -DBUILD_opencv_apps:BOOL=OFF
            -DBUILD_opencv_calib3d:BOOL=ON
            -DBUILD_opencv_flann:BOOL=ON
            -DBUILD_opencv_features2d:BOOL=ON
            -DBUILD_opencv_objdetect:BOOL=ON
            -DBUILD_opencv_photo:BOOL=ON
            -DBUILD_opencv_ts:BOOL=OFF
            -DBUILD_opencv_ml:BOOL=ON
            -DBUILD_opencv_video:BOOL=ON
            -DBUILD_opencv_java:BOOL=OFF
            -DBUILD_opencv_python2:BOOL=OFF
            -DBUILD_opencv_python3:BOOL=OFF
            -DPYTHON2_LIBRARY:STRING=C:/Python27/libs/python27.lib
            -DPYTHON3_LIBRARY:STRING=C:/Python35/libs/python35.lib
          INSTALL_DIR ${ROOT_DIR}/deps/local/
        )

        add_definitions(-DHAS_OPENCV)

        set(OpenCV_DIR ${ROOT_DIR}/deps/local)
        set(OpenCV_INCLUDE_DIRS ${ROOT_DIR}/deps/local/include )
        if (${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8)
            set(OPENCV_LIBS_DIR ${ROOT_DIR}/deps/local/x64/vc14/staticlib)
        else()
            set(OPENCV_LIBS_DIR ${ROOT_DIR}/deps/local/x86/vc14/staticlib)
        endif()

        foreach(__CVLIB core calib3d features2d flann imgproc imgcodecs ml highgui objdetect video videoio)
            set(OpenCV_${__CVLIB}_LIBRARY debug ${OPENCV_LIBS_DIR}/opencv_${__CVLIB}330d.lib optimized ${OPENCV_LIBS_DIR}/opencv_${__CVLIB}330.lib CACHE STRING "" FORCE)
            set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
        endforeach(__CVLIB)    

        foreach(__CVLIB libjpeg libpng libwebp zlib ittnotify)
            set(OpenCV_${__CVLIB}_LIBRARY debug ${OPENCV_LIBS_DIR}/${__CVLIB}d.lib optimized ${OPENCV_LIBS_DIR}/${__CVLIB}.lib CACHE STRING "" FORCE)
            set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
        endforeach(__CVLIB)     

        LIST(APPEND OpenCV_LIBS vfw32.lib)

    ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        # Location of homebrew opencv3's OpenCVConfig.cmake
        # Alternatively, can do `brew ln opencv3 --force`
        MESSAGE(STATUS "Using homebrew opencv3")
        set(OpenCV_DIR "/usr/local/opt/opencv3/share/OpenCV")
    ELSE()
        set(OpenCV_DIR "/usr/local/share/OpenCV")
    ENDIF()#Windows or Darwin
ENDIF(NOT OpenCV_DIR)
LIST(APPEND CMAKE_MODULE_PATH ${OpenCV_DIR})
set(OpenCV_STATIC ON)
IF(NOT(${CMAKE_SYSTEM_NAME} MATCHES "Windows"))
    FIND_PACKAGE(OpenCV REQUIRED)
ENDIF()

# SDL and GL
set(SDL_GL_INCLUDE_DIRS)
set(SDL_GL_LIBS)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    find_library(OPENGL_FRAMEWORK OpenGL)
    find_package(SDL2)
    list(APPEND SDL_GL_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND SDL_GL_LIBS
        ${SDL2_LIBRARY} ${OPENGL_FRAMEWORK} ${GLUT_FRAMEWORK})
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    find_package(SDL2)
    list(APPEND SDL_GL_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND SDL_GL_LIBS ${SDL2_LIBRARY} GL)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    SET(ENV{SDL2DIR} ${ROOT_DIR}/thirdparty/SDL2/)
    find_package(SDL2)
    list(APPEND SDL_GL_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    list(APPEND SDL_GL_LIBS 
        ${SDL2_LIBRARY}
        imm32.lib
        version.lib)
ENDIF()

# libusb
IF (NOT LIBUSB_INCLUDE_DIR OR NOT LIBUSB_LIBRARIES)
	IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin" OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))    
		#Requires libusb
		find_package(USB1 REQUIRED)
	ENDIF()
ENDIF()

# For PSEye camera
set(PSEYE_SRC)
set(PSEYE_INCLUDE_DIRS)
set(PSEYE_LIBRARIES)
IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin" OR (${CMAKE_SYSTEM_NAME} MATCHES "Windows"))    
    #PS3EYEDriver
    list(APPEND PSEYE_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/PS3EYEDriver/src)
    list(APPEND PSEYE_SRC
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.h
        ${ROOT_DIR}/thirdparty/PS3EYEDriver/src/ps3eye.cpp)
    add_definitions(-DHAVE_PS3EYE)
	
    #Requires libusb
    list(APPEND PSEYE_INCLUDE_DIRS ${LIBUSB_INCLUDE_DIR})
    list(APPEND PSEYE_LIBRARIES ${LIBUSB_LIBRARIES})
	
    IF (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-braced-scalar-init")
    ENDIF()	
ENDIF()

# hidapi
set(HIDAPI_INCLUDE_DIRS ${ROOT_DIR}/thirdparty/hidapi/hidapi)
set(HIDAPI_SRC)
set(HIDAPI_LIBS)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/windows/hid.c)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    list(APPEND HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/mac/hid.c)
ELSE()
    list(APPEND HIDAPI_SRC ${ROOT_DIR}/thirdparty/hidapi/linux/hid.c)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(UDEV REQUIRED libudev)
    list(APPEND HIDAPI_INCLUDE_DIRS ${UDEV_INCLUDE_DIRS})
    list(APPEND HIDAPI_LIBS ${UDEV_LIBRARIES})
    pkg_check_modules(BLUEZ REQUIRED bluez)
    list(APPEND HIDAPI_INCLUDE_DIRS ${BLUEZ_INCLUDE_DIRS})
    list(APPEND HIDAPI_LIBS ${BLUEZ_LIBRARIES})
ENDIF()
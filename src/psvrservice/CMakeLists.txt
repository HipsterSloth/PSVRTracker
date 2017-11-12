cmake_minimum_required(VERSION 3.0)

# Dependencies
set(PSVR_SERVICE_INCL_DIRS)
set(PSVR_SERVICE_REQ_LIBS)

list(APPEND PSVR_SERVICE_REQ_LIBS ${PLATFORM_LIBS})

# Source files for PSVRService
file(GLOB PSVR_CLIENTAPI_SRC
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/*.h"
)
source_group("ClientAPI" FILES ${PSVR_CLIENTAPI_SRC})

file(GLOB PSVR_CONFIG_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSVRConfig/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSVRConfig/*.h"
)
source_group("Config" FILES ${PSVR_CONFIG_SRC})

file(GLOB PSVR_DEVICE_ENUM_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator/*.h"
)
source_group("Device\\Enumerator" FILES ${PSVR_DEVICE_ENUM_SRC})

file(GLOB PSVR_DEVICE_INT_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Interface/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Interface/*.h"
)
source_group("Device\\Interface" FILES ${PSVR_DEVICE_INT_SRC})

file(GLOB PSVR_DEVICE_MGR_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Manager/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Manager/*.h"
)
source_group("Device\\Manager" FILES ${PSVR_DEVICE_MGR_SRC})

file(GLOB PSVR_DEVICE_USB_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/USB/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/USB/*.h"
)
source_group("Device\\USB" FILES ${PSVR_DEVICE_USB_SRC})

file(GLOB PSVR_DEVICE_VIEW_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/View/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/View/*.h"
)
source_group("Device\\View" FILES ${PSVR_DEVICE_VIEW_SRC})

file(GLOB PSVR_HMD_SRC
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualHMD/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/VirtualHMD/*.h"
)
source_group("HMD" FILES ${PSVR_HMD_SRC})

file(GLOB PSVR_FILTER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.h"
)
source_group("Filter" FILES ${PSVR_FILTER_SRC})

file(GLOB SERVICE_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Service/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Service/*.h"
)
source_group("Service" FILES ${SERVICE_SRC})

file(GLOB PSVR_TRACKER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSVRTracker/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSVRTracker/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/PSVRTracker/PSEye/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSVRTracker/PSEye/*.h"
)
source_group("Tracker" FILES ${PSVR_TRACKER_SRC})

file(GLOB PSVR_UTILS_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Utils/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Utils/*.h"
)
source_group("Utils" FILES ${PSVR_UTILS_SRC})

set(PSVR_SERVICE_SRC
	${PSVR_CLIENTAPI_SRC}
    ${PSVR_CONFIG_SRC}
    ${PSVR_DEVICE_ENUM_SRC}
    ${PSVR_DEVICE_INT_SRC}
    ${PSVR_DEVICE_MGR_SRC}
    ${PSVR_DEVICE_USB_SRC}
    ${PSVR_DEVICE_VIEW_SRC}
    ${PSVR_HMD_SRC}
    ${PSVR_FILTER_SRC}
    ${SERVICE_SRC} 
    ${PSVR_TRACKER_SRC}
	${PSVR_UTILS_SRC}
)

list(APPEND PSVR_SERVICE_INCL_DIRS
	${CMAKE_CURRENT_LIST_DIR}/ClientAPI
    ${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator
    ${CMAKE_CURRENT_LIST_DIR}/Device/Interface
    ${CMAKE_CURRENT_LIST_DIR}/Device/Manager
    ${CMAKE_CURRENT_LIST_DIR}/Device/USB
    ${CMAKE_CURRENT_LIST_DIR}/Device/View
    ${CMAKE_CURRENT_LIST_DIR}/Filter
    ${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD
    ${CMAKE_CURRENT_LIST_DIR}/Platform
    ${CMAKE_CURRENT_LIST_DIR}/PSVRConfig
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker
    ${CMAKE_CURRENT_LIST_DIR}/PSMoveTracker/PSEye
    ${CMAKE_CURRENT_LIST_DIR}/Service
	${CMAKE_CURRENT_LIST_DIR}/Utils
    ${CMAKE_CURRENT_LIST_DIR}/VirtualHMD
)

# Eigen math library
list(APPEND PSVR_SERVICE_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

# mherb/Kalman library
list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/kalman/include)

# hidapi
list(APPEND PSVR_SERVICE_INCL_DIRS ${HIDAPI_INCLUDE_DIRS})
list(APPEND PSVR_SERVICE_SRC ${HIDAPI_SRC})
list(APPEND PSVR_SERVICE_REQ_LIBS ${HIDAPI_LIBS})

# LibUSB for device management
list(APPEND PSVR_SERVICE_INCL_DIRS ${LIBUSB_INCLUDE_DIR})
list(APPEND PSVR_SERVICE_REQ_LIBS ${LIBUSB_LIBRARIES})

# Platform Specific Device Management
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    # Windows utilities for querying driver infomation (provider name)
    list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Platform)
    list(APPEND PSVR_SERVICE_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.h
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
ELSE()
ENDIF()

# PSMoveMath
list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psvrmath/)
list(APPEND PSVR_SERVICE_REQ_LIBS PSVRMath)

# Tracker
# Requires OpenCV, PS3EYEDriver (Mac/Win64), CLEye (Win32)

# OpenCV - empty on Windows
IF(MSVC) # not necessary for OpenCV > 2.8 on other build systems
    list(APPEND PSVR_SERVICE_INCL_DIRS ${OpenCV_INCLUDE_DIRS}) 
ENDIF()
list(APPEND PSVR_SERVICE_REQ_LIBS ${OpenCV_LIBS})

# PS Eye - This brings in LIBUSB on Windows and Mac, but not Linux
list(APPEND PSVR_SERVICE_SRC ${PSEYE_SRC})
list(APPEND PSVR_SERVICE_INCL_DIRS ${PSEYE_INCLUDE_DIRS})
list(APPEND PSVR_SERVICE_REQ_LIBS ${PSEYE_LIBRARIES})

#
# PSVRService Static library
#

# TODO: Build PSMoveClient as a STATIC or OBJECT w/ $<TARGET_OBJECTS:objlib>
add_library(PSVRService_static STATIC ${PSVR_SERVICE_SRC})
target_include_directories(PSVRService_static PUBLIC ${PSVR_SERVICE_INCL_DIRS})
target_link_libraries(PSVRService_static PUBLIC ${PSVR_SERVICE_REQ_LIBS})
target_compile_definitions(PSVRService_static PRIVATE PSVRService_STATIC) # See PSVRClient_export.h
target_compile_definitions(PSVRService_static PRIVATE PSVRSERVICE_CPP_API) # See PSVRClient_export.h

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_dependencies(PSVRService_static opencv)
ENDIF()

#
# PSVRService Shared library
#

set(PSVR_SERVICE_CAPI_REQ_LIBS)

# PSVRService_static
list(APPEND PSVR_SERVICE_CAPI_REQ_LIBS PSVRService_static)
#Via PSVRService_static, transitively inherits PSMoveMath

# Source files to develop the shared library.
list(APPEND PSVR_SERVICE_CAPI_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/PSVRClient_export.h"
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/ClientGeometry_CAPI.h"
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/ClientGeometry_CAPI.cpp"	
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/PSVRClient_CAPI.h"
    "${CMAKE_CURRENT_LIST_DIR}/ClientAPI/PSVRClient_CAPI.cpp"
)

# Shared library
add_library(PSVRService SHARED ${PSVR_SERVICE_CAPI_LIBRARY_SRC})
target_include_directories(PSVRService PUBLIC ${CMAKE_CURRENT_LIST_DIR}/ClientAPI)
target_link_libraries(PSVRService PRIVATE ${PSVR_SERVICE_CAPI_REQ_LIBS})
target_compile_definitions(PSVRService_static PRIVATE PSVRClient_EXPORTS) # See PSVRClient_export.h
set_target_properties(PSVRService PROPERTIES PUBLIC_HEADER "ClientConstants.h;ClientGeometry_CAPI.h;PSMoveClient_CAPI.h;PSMoveClient_export.h")
set_target_properties(PSVRService PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(PSVRService PROPERTIES C_VISIBILITY_PRESET hidden)	

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSVRService
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
		PUBLIC_HEADER DESTINATION ${PSM_DEBUG_INSTALL_PATH}/include)
    install(TARGETS PSVRService
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
		PUBLIC_HEADER DESTINATION ${PSM_RELEASE_INSTALL_PATH}/include)        
ELSE() #Linux/Darwin
ENDIF()

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

file(GLOB PSVR_CONTROLLER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveController/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/PSMoveController/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/DualShock4/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/DualShock4/*.h"
)
source_group("Controller" FILES ${PSVR_CONTROLLER_SRC})

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

file(GLOB PSVR_DEVICE_MODEL_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Device/Model/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Device/Model/*.h"
)
source_group("Device\\Model" FILES ${PSVR_DEVICE_MODEL_SRC})

file(GLOB PSVR_HMD_SRC
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD/*.h"
)
source_group("HMD" FILES ${PSVR_HMD_SRC})

file(GLOB PSVR_FILTER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/Filter/*.h"
)
source_group("Filter" FILES ${PSVR_FILTER_SRC})

list(APPEND PSVR_PLATFORM_SRC
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueries.h
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequests.h
    ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequests.cpp)
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    list(APPEND PSVR_PLATFORM_SRC
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothRequestsWin32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/BluetoothQueriesWin32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.h
        ${CMAKE_CURRENT_LIST_DIR}/Platform/PlatformDeviceAPIWin32.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/WinUSBApi.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/WinUSBApi.h
        ${CMAKE_CURRENT_LIST_DIR}/Platform/WinUSBTransferBundle.cpp
        ${CMAKE_CURRENT_LIST_DIR}/Platform/WinUSBTransferBundle.h)
ENDIF()
source_group("Platform" FILES ${PSVR_PLATFORM_SRC})
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
    "${CMAKE_CURRENT_LIST_DIR}/WMFTracker/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/WMFTracker/*.h"		
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
	${PSVR_CONTROLLER_SRC}
    ${PSVR_DEVICE_ENUM_SRC}
    ${PSVR_DEVICE_INT_SRC}
    ${PSVR_DEVICE_MGR_SRC}
	${PSVR_DEVICE_MODEL_SRC}
    ${PSVR_DEVICE_USB_SRC}
    ${PSVR_DEVICE_VIEW_SRC}
    ${PSVR_HMD_SRC}
    ${PSVR_FILTER_SRC}
    ${PSVR_PLATFORM_SRC}
    ${SERVICE_SRC} 
    ${PSVR_TRACKER_SRC}
	${PSVR_UTILS_SRC}
)

list(APPEND PSVR_SERVICE_INCL_DIRS
	${CMAKE_CURRENT_LIST_DIR}/ClientAPI
    ${CMAKE_CURRENT_LIST_DIR}/Device/Enumerator
    ${CMAKE_CURRENT_LIST_DIR}/Device/Interface
    ${CMAKE_CURRENT_LIST_DIR}/Device/Manager
	${CMAKE_CURRENT_LIST_DIR}/Device/Model
    ${CMAKE_CURRENT_LIST_DIR}/Device/USB
    ${CMAKE_CURRENT_LIST_DIR}/Device/View
	${CMAKE_CURRENT_LIST_DIR}/DualShock4
    ${CMAKE_CURRENT_LIST_DIR}/Filter
    ${CMAKE_CURRENT_LIST_DIR}/MorpheusHMD
    ${CMAKE_CURRENT_LIST_DIR}/Platform
	${CMAKE_CURRENT_LIST_DIR}/PSMoveController
    ${CMAKE_CURRENT_LIST_DIR}/PSVRConfig
    ${CMAKE_CURRENT_LIST_DIR}/PSVRTracker
    ${CMAKE_CURRENT_LIST_DIR}/PSVRTracker/PSEye
    ${CMAKE_CURRENT_LIST_DIR}/Service
	${CMAKE_CURRENT_LIST_DIR}/Utils
	${CMAKE_CURRENT_LIST_DIR}/WMFTracker
)

# Configuru
list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/Configuru)

# Lockfree Queue
list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/thirdparty/lockfreequeue)

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
    list(APPEND PSVR_SERVICE_SRC ${PSVR_WIN32_PLATFORM_SRC})	
	source_group("Device\\USB" FILES ${PSVR_WIN32_PLATFORM_SRC})
		
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
ELSE()
ENDIF()

# PSMoveMath
list(APPEND PSVR_SERVICE_INCL_DIRS ${ROOT_DIR}/src/psvrmath/)
list(APPEND PSVR_SERVICE_REQ_LIBS PSVRMath)

# Tracker
# Requires OpenCV

# OpenCV - empty on Windows
IF(MSVC) # not necessary for OpenCV > 2.8 on other build systems
    list(APPEND PSVR_SERVICE_INCL_DIRS ${OpenCV_INCLUDE_DIRS}) 
ENDIF()
list(APPEND PSVR_SERVICE_REQ_LIBS ${OpenCV_LIBS})

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
	"${CMAKE_CURRENT_LIST_DIR}/ClientAPI/ClientColor_CAPI.h"
	"${CMAKE_CURRENT_LIST_DIR}/ClientAPI/ClientConstants.h"
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
set_target_properties(PSVRService PROPERTIES PUBLIC_HEADER "ClientAPI/ClientConstants.h;ClientAPI/ClientColor_CAPI.h;ClientAPI/ClientGeometry_CAPI.h;ClientAPI/PSVRClient_CAPI.h;ClientAPI/PSVRClient_export.h")
set_target_properties(PSVRService PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties(PSVRService PROPERTIES C_VISIBILITY_PRESET hidden)	

# Post build dependencies (resources)
add_custom_command(TARGET PSVRService POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${ROOT_DIR}/resources"
        $<TARGET_FILE_DIR:PSVRService>/resources)        
		
# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS PSVRService
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/lib
		PUBLIC_HEADER DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/include)
    install(TARGETS PSVRService
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/lib
		PUBLIC_HEADER DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/include)  
    install(DIRECTORY ${ROOT_DIR}/resources/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/bin/resources
            FILES_MATCHING PATTERN "*.bin" PATTERN "*.pdf" PATTERN "*.json")
    install(DIRECTORY ${ROOT_DIR}/resources/ 
            CONFIGURATIONS Release
            DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/bin/resources
            FILES_MATCHING PATTERN "*.bin" PATTERN "*.pdf" PATTERN "*.json")
ELSE() #Linux/Darwin
ENDIF()

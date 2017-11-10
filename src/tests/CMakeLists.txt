#
# TEST_CAMERA
#

SET(TEST_CAMERA_SRC)
SET(TEST_CAMERA_INCL_DIRS)
SET(TEST_CAMERA_REQ_LIBS)

# OpenCV
IF(MSVC) # not necessary for OpenCV > 2.8 on other build systems
    list(APPEND TEST_CAMERA_INCL_DIRS ${OpenCV_INCLUDE_DIRS}) 
ENDIF()
list(APPEND TEST_CAMERA_REQ_LIBS ${OpenCV_LIBS})

# PS3EYE
list(APPEND TEST_CAMERA_SRC ${PSEYE_SRC})
list(APPEND TEST_CAMERA_INCL_DIRS ${PSEYE_INCLUDE_DIRS})
list(APPEND TEST_CAMERA_REQ_LIBS ${PSEYE_LIBRARIES})
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows"
    AND NOT(${CMAKE_C_SIZEOF_DATA_PTR} EQUAL 8))
    # Windows utilities for querying driver infomation (provider name)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Device/Interface)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Server)
    list(APPEND TEST_CAMERA_INCL_DIRS ${ROOT_DIR}/src/psmoveservice/Platform)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Device/Interface/DevicePlatformInterface.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Server/ServerLog.cpp)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Platform/PlatformDeviceAPIWin32.h)
    list(APPEND TEST_CAMERA_SRC ${ROOT_DIR}/src/psmoveservice/Platform/PlatformDeviceAPIWin32.cpp)   
ENDIF()

# Our custom OpenCV VideoCapture classes
# We could include the PSMoveService project but we want our test as isolated as possible.
list(APPEND TEST_CAMERA_INCL_DIRS 
    ${ROOT_DIR}/src/psmoveclient/
    ${ROOT_DIR}/src/psmoveprotocol/
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye)
list(APPEND TEST_CAMERA_SRC
    ${ROOT_DIR}/src/psmoveclient/ClientConstants.h
    ${ROOT_DIR}/src/psmoveprotocol/SharedConstants.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.h
    ${ROOT_DIR}/src/psmoveservice/PSMoveTracker/PSEye/PSEyeVideoCapture.cpp)

# The test_camera app
add_executable(test_camera ${CMAKE_CURRENT_LIST_DIR}/test_camera.cpp ${TEST_CAMERA_SRC})
target_include_directories(test_camera PUBLIC ${TEST_CAMERA_INCL_DIRS})
target_link_libraries(test_camera ${PLATFORM_LIBS} ${TEST_CAMERA_REQ_LIBS})
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    add_dependencies(test_camera opencv)
ENDIF()
SET_TARGET_PROPERTIES(test_camera PROPERTIES FOLDER Test)
    
# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS test_camera
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS test_camera
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)
ELSE() #Linux/Darwin
ENDIF()

#
# UNIT_TESTS
#

list(APPEND UNIT_TEST_INCL_DIRS
    ${ROOT_DIR}/src/psmovemath/)

# Eigen math library
list(APPEND UNIT_TEST_INCL_DIRS ${EIGEN3_INCLUDE_DIR})

list(APPEND UNIT_TEST_SRC
    ${ROOT_DIR}/src/psmovemath/MathAlignment.h
    ${ROOT_DIR}/src/psmovemath/MathAlignment.cpp
    ${ROOT_DIR}/src/psmovemath/MathEigen.h
    ${ROOT_DIR}/src/psmovemath/MathEigen.cpp
    ${ROOT_DIR}/src/psmovemath/MathUtility.h
    ${ROOT_DIR}/src/psmovemath/MathUtility.cpp
    ${ROOT_DIR}/src/tests/math_alignment_unit_tests.cpp
    ${ROOT_DIR}/src/tests/math_eigen_unit_tests.cpp
    ${ROOT_DIR}/src/tests/math_utility_unit_tests.cpp
    ${ROOT_DIR}/src/tests/unit_test.h)

add_executable(unit_test_suite ${CMAKE_CURRENT_LIST_DIR}/unit_test_suite.cpp ${UNIT_TEST_SRC})
target_include_directories(unit_test_suite PUBLIC ${UNIT_TEST_INCL_DIRS})
SET_TARGET_PROPERTIES(unit_test_suite PROPERTIES FOLDER Test)

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    install(TARGETS unit_test_suite
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(TARGETS unit_test_suite
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)        
ELSE() #Linux/Darwin
ENDIF()

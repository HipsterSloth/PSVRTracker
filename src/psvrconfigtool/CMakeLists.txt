set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSVRCONFIGTOOL_INCL_DIRS)
set(PSVRCONFIGTOOL_REQ_LIBS)

# platform independent include paths
list(APPEND PSVRCONFIGTOOL_INCL_DIRS
    ${EIGEN3_INCLUDE_DIR}
	${ROOT_DIR}/src/psvrmath/
	${ROOT_DIR}/src/psvrservice/ClientAPI
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui)

# platform independent libraries
list(APPEND PSMOVECONFIGTOOL_REQ_LIBS 
    PSMoveMath
    PSVRService
    ${PROTOBUF_LIBRARIES})
	
# SDL/GL
list(APPEND PSVRCONFIGTOOL_INCL_DIRS ${SDL_GL_INCLUDE_DIRS})
list(APPEND PSVRCONFIGTOOL_REQ_LIBS ${SDL_GL_LIBS})

# OpenCV
list(APPEND PSVRCONFIGTOOL_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSVRCONFIGTOOL_REQ_LIBS ${OpenCV_LIBS})

# Source files that are needed for the psmove config tool
file(GLOB PSVRConfigTool_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${ROOT_DIR}/thirdparty/imgui/*.cpp"
)

# PSVRConfigTool
add_executable(PSVRConfigTool ${PSVRConfigTool_SRC})
target_include_directories(PSVRConfigTool PUBLIC ${PSVRCONFIGTOOL_INCL_DIRS})
target_link_libraries(PSVRConfigTool ${PSVRCONFIGTOOL_REQ_LIBS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
add_dependencies(PSVRConfigTool opencv)
ENDIF()

#MacOS OpenCV must be self-built, this links against older std, which is hidden
#Therefore the PSVRConfigTool must be hidden
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES(PSVRConfigTool
        PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
ENDIF()

# Post build dependencies (assets and external DLLs)
add_custom_command(TARGET PSVRConfigTool POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/assets"
        $<TARGET_FILE_DIR:PSVRConfigTool>/assets)        

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")   
    install(TARGETS PSVRConfigTool
        CONFIGURATIONS Debug
        RUNTIME DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_DEBUG_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSM_DEBUG_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
    install(TARGETS PSVRConfigTool
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSM_RELEASE_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Release
            DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Release
            DESTINATION ${PSM_RELEASE_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
ELSE() #Linux/Darwin
ENDIF()

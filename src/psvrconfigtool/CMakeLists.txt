set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSVRTRACKER_INCL_DIRS)
set(PSVRTRACKER_REQ_LIBS)

# platform independent include paths
list(APPEND PSVRTRACKER_INCL_DIRS
    ${EIGEN3_INCLUDE_DIR}
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui)
   
# SDL/GL
list(APPEND PSVRTRACKER_INCL_DIRS ${SDL_GL_INCLUDE_DIRS})
list(APPEND PSVRTRACKER_REQ_LIBS ${SDL_GL_LIBS})

# OpenCV
list(APPEND PSVRTRACKER_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSVRTRACKER_REQ_LIBS ${OpenCV_LIBS})

# Source files that are needed for the psmove config tool
file(GLOB PSVRTRACKER_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${ROOT_DIR}/thirdparty/imgui/*.cpp"
)

# PSVRTracker
add_executable(PSVRTracker ${PSVRTRACKER_SRC})
target_include_directories(PSVRTracker PUBLIC ${PSVRTRACKER_INCL_DIRS})
target_link_libraries(PSVRTracker ${PSVRTRACKER_REQ_LIBS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
add_dependencies(PSMoveService opencv)
ENDIF()

#MacOS OpenCV must be self-built, this links against older std, which is hidden
#Therefore the PSVRTracker must be hidden
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    SET_TARGET_PROPERTIES(PSVRTracker
        PROPERTIES COMPILE_FLAGS "-fvisibility=hidden")
ENDIF()

# Post build dependencies (assets and external DLLs)
add_custom_command(TARGET PSVRTracker POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_LIST_DIR}/assets"
        $<TARGET_FILE_DIR:PSVRTracker>/assets)        

# Install    
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")   
    install(TARGETS PSVRTracker
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
    install(TARGETS PSVRTracker
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

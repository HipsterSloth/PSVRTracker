set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(PSVRCONFIGTOOL_INCL_DIRS)
set(PSVRCONFIGTOOL_REQ_LIBS)

# platform independent include paths
list(APPEND PSVRCONFIGTOOL_INCL_DIRS
    ${EIGEN3_INCLUDE_DIR}
	${ROOT_DIR}/src/psvrmath/
	${ROOT_DIR}/src/psvrservice/ClientAPI
	${ROOT_DIR}/src/psvrservice/Utils
    ${ROOT_DIR}/thirdparty/glm
    ${ROOT_DIR}/thirdparty/stb
    ${ROOT_DIR}/thirdparty/imgui)

# platform independent libraries
list(APPEND PSVRCONFIGTOOL_REQ_LIBS 
    PSVRMath
    PSVRService)
	
# SDL/GL
list(APPEND PSVRCONFIGTOOL_INCL_DIRS ${SDL_GL_INCLUDE_DIRS})
list(APPEND PSVRCONFIGTOOL_REQ_LIBS ${SDL_GL_LIBS})

# OpenCV
list(APPEND PSVRCONFIGTOOL_INCL_DIRS ${OpenCV_INCLUDE_DIRS})
list(APPEND PSVRCONFIGTOOL_REQ_LIBS ${OpenCV_LIBS})

file(GLOB PSVRCONFIGTOOL_APP_STAGE_SRC
    "${CMAKE_CURRENT_LIST_DIR}/AppStage*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/AppStage*.h"
    "${CMAKE_CURRENT_LIST_DIR}/AppSubStage*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/AppSubStage*.h"
)
source_group("AppStages" FILES ${PSVRCONFIGTOOL_APP_STAGE_SRC})

file(GLOB PSVRCONFIGTOOL_APP_CORE_SRC
    "${CMAKE_CURRENT_LIST_DIR}/App.*"
	"${CMAKE_CURRENT_LIST_DIR}/AssetManager.*"
	"${CMAKE_CURRENT_LIST_DIR}/Camera.*"
	"${CMAKE_CURRENT_LIST_DIR}/Logger.*"
	"${CMAKE_CURRENT_LIST_DIR}/Renderer.*"
	"${CMAKE_CURRENT_LIST_DIR}/PSVRTracker.cpp"
	"${CMAKE_CURRENT_LIST_DIR}/UIConstants.h"
	"${ROOT_DIR}/src/psvrservice/Utils/MathTypeConversion.h"
	"${ROOT_DIR}/src/psvrservice/Utils/MathTypeConversion.cpp"
    "${ROOT_DIR}/thirdparty/imgui/*.cpp"	
)
source_group("AppCore" FILES ${PSVRCONFIGTOOL_APP_CORE_SRC})

# Source files that are needed for the psmove config tool
file(GLOB PSVRConfigTool_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
	"${ROOT_DIR}/src/psvrservice/Utils/MathTypeConversion.h"
	"${ROOT_DIR}/src/psvrservice/Utils/MathTypeConversion.cpp"
    "${ROOT_DIR}/thirdparty/imgui/*.cpp"
)

# PSVRConfigTool
add_executable(PSVRConfigTool ${PSVRConfigTool_SRC})
target_include_directories(PSVRConfigTool PUBLIC ${PSVRCONFIGTOOL_INCL_DIRS})
target_link_libraries(PSVRConfigTool ${PSVRCONFIGTOOL_REQ_LIBS})

IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
add_dependencies(PSVRConfigTool opencv)
set_property(TARGET PSVRConfigTool PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${PSVR_DEBUG_INSTALL_PATH}/bin")
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
        RUNTIME DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Debug
            DESTINATION ${PSVR_DEBUG_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
    install(TARGETS PSVRConfigTool
        CONFIGURATIONS Release
        RUNTIME DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/bin
        LIBRARY DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/lib
        ARCHIVE DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/lib)
    install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/assets/ 
            CONFIGURATIONS Release
            DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/bin/assets
            FILES_MATCHING PATTERN "*.ttf"  PATTERN "*.jpg")
    install(DIRECTORY ${OPENVR_BINARIES_DIR}/ 
            CONFIGURATIONS Release
            DESTINATION ${PSVR_RELEASE_INSTALL_PATH}/bin
            FILES_MATCHING PATTERN "*.dll" PATTERN "*.pdb")   
ELSE() #Linux/Darwin
ENDIF()

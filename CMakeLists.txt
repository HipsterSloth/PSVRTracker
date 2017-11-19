cmake_minimum_required(VERSION 3.2)
project(PSVRTracker VERSION 0.0.1)
include(ExternalProject)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(cmake/Environment.cmake)
include(cmake/Version.cmake)

set(PSVR_PROJECT_NAME "PSVRService_${PSVR_VERSION_STRING}")
set(PSVR_ROOT_INSTALL_PATH "${ROOT_DIR}/dist")
set(PSVR_DEBUG_INSTALL_PATH "${PSVR_ROOT_INSTALL_PATH}/${ARCH_LABEL}/Debug")
set(PSVR_RELEASE_INSTALL_PATH "${PSVR_ROOT_INSTALL_PATH}/${ARCH_LABEL}/Release")
file(MAKE_DIRECTORY "${PSVR_ROOT_INSTALL_PATH}")
file(WRITE "${PSVR_ROOT_INSTALL_PATH}/version.txt" "${PSVR_PROJECT_NAME}")

include(cmake/ThirdParty.cmake)

# Step into the subdirectories
add_subdirectory(src)


cmake_minimum_required(VERSION 3.2)
project(PSVRTracker VERSION 0.0.1)
include(ExternalProject)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR})

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(cmake/Environment.cmake)
include(cmake/ThirdParty.cmake)

# Step into the subdirectories
add_subdirectory(src)


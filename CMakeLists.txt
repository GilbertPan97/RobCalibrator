cmake_minimum_required(VERSION 3.5)

# config project version
set(SFRCALIB_VERSION_MAJOR 0)
set(SFRCALIB_VERSION_MINOR 4)
set(SFRCALIB_VERSION_PATCH 0)
set(SFRCALIB_VERSION "${SFRCALIB_VERSION_MAJOR}.${SFRCALIB_VERSION_MINOR}.${SFRCALIB_VERSION_PATCH}")
message(STATUS "SfrCalib version is: ${SFRCALIB_VERSION}")

# local dependent package
set(THIRD_PARTY "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
project(SfrCalib VERSION ${SFRCALIB_VERSION})

option(BUILD_EXAMPLES "build the examples" ON)
option(BUILD_DESIGNERPLUGIN "build the Designer Plugin" OFF)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_DEBUG_POSTFIX "d" CACHE STRING "add a postfix, usually d on windows")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "add a postfix, usually empty on windows")
set(CMAKE_RELWITHDEBINFO_POSTFIX "rd" CACHE STRING "add a postfix, usually empty on windows")
set(CMAKE_MINSIZEREL_POSTFIX "s" CACHE STRING "add a postfix, usually empty on windows")

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4819")
    add_compile_options(/bigobj)    # add big obj option to avoid calibrator build fail
endif()

# # cmd not visible when run exec
# if(WIN32)
#   set(CMAKE_EXE_LINKER_FLAGS "/SUBSYSTEM:WINDOWS")
# elseif(UNIX AND NOT APPLE)
#   set(CMAKE_EXE_LINKER_FLAGS "-ldl")
# endif()

include(CPack)
# platform check
if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "4")
    set(SFRCALIB_PLATFORM "86")
else()
    set(SFRCALIB_PLATFORM "64")
endif()

# add window platform resource config macro definition
include(cmake/WinResource.cmake)

# The bin file directory is one level above the current directory
if((${CMAKE_BUILD_TYPE}) STREQUAL "Debug")
    set(SFRCALIB_BUILD_TYPE debug)
elseif((${CMAKE_BUILD_TYPE}) STREQUAL "Release")
    set(SFRCALIB_BUILD_TYPE release)
endif()

set(SFRCALIB_BIN_NAME bin_${SFRCALIB_VERSION})
set(SFRCALIB_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${SFRCALIB_BIN_NAME})

add_subdirectory(src)
add_subdirectory(cmake)

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()
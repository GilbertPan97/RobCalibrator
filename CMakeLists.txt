cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(vtk_point_display)

set(SFR_SDK_DIR "C:/SFR_SDK")
find_package(VTK REQUIRED HINTS ${SFR_SDK_DIR})

include(${VTK_USE_FILE})

if (NOT VTK_FOUND)
  message("Skipping vtk_point_display: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")

add_executable(vtk_point_display vtk_point_display.cpp)
target_link_libraries(vtk_point_display PRIVATE ${VTK_LIBRARIES})

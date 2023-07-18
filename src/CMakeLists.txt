cmake_minimum_required(VERSION 3.5)
set(SFRCALIB_MAINWIN RobCalib)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# WARNING: Variables defined in a sub-level CMake file are only visible within that file and its immediate sub-level files. 
# They are not automatically propagated to the parent or higher-level CMake files.

# find qt package: for main window exec
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)
message(STATUS "current Qt version is Qt${QT_VERSION_MAJOR}.${QT_VERSION_MINOR}.${QT_VERSION_PATCH}")

# find Eigen package: used for Calibrator
find_package(Eigen3 REQUIRED HINTS ${THIRD_PARTY})
message(STATUS "Eigen dir is: ${EIGEN3_INCLUDE_DIRS}")

# find vtk package: used for QtVTKviewer
find_package(VTK REQUIRED HINTS ${THIRD_PARTY}
             COMPONENTS
             CommonCore
             CommonColor
             RenderingCore
             RenderingAnnotation
             GUISupportQt
             IOGeometry)

option(VTK_USE_QVTK_OPENGL_NATIVE "Use QVTKOpenGLNativeWidget for rendering" ON)
if (VTK_USE_QVTK_OPENGL_NATIVE)
  add_definitions(-DVTK_USE_QVTK_OPENGL_NATIVE)
endif()
message(STATUS "VTK used package include: ${VTK_LIBRARIES}")

# find nlohmann_json package: for main window exec
find_package(nlohmann_json REQUIRED HINTS ${THIRD_PARTY}/json-3.11.2/share/cmake/nlohmann_json)

# build all dependent libs before main window
add_subdirectory(SfrCalibLibs)

set(HEADER_FILES 
    mainwindow.h
    startupprogresswindow.h)

set(SRC_FILES 
    mainwindow.cpp 
    startupprogresswindow.cpp
    main.cpp)

set(QRC_FILES 
    mainwindow.qrc 
    SfrCalibLibs/QtDataViewer/QtDataViewer.qrc
    SfrCalibLibs/SARibbonBar/resource.qrc
)

# build main window execution
add_executable(${SFRCALIB_MAINWIN} ${SRC_FILES} ${HEADER_FILES} ${QRC_FILES})

target_include_directories(${SFRCALIB_MAINWIN}  PRIVATE 
                    "${PROJECT_BINARY_DIR}"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/SARibbonBar"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/QtDataViewer"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/QtPropertyBrowser"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/Calibrator"
                    ${EIGEN3_INCLUDE_DIRS})
message(STATUS "Sfr calibration bin dir is: ${SFRCALIB_BIN_NAME}")

target_link_libraries(${SFRCALIB_MAINWIN} PRIVATE 
                      QtAdvancedDocking
                      QtPropertyBrowser
                      QtDataViewer
                      SARibbonBar
                      Calibrator)

target_link_libraries(${SFRCALIB_MAINWIN} PUBLIC
                      Qt${QT_VERSION_MAJOR}::Core 
                      Qt${QT_VERSION_MAJOR}::Gui 
                      Qt${QT_VERSION_MAJOR}::Widgets
                      ${VTK_LIBRARIES}
                      nlohmann_json::nlohmann_json)

set_target_properties(${SFRCALIB_MAINWIN} PROPERTIES 
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    # WIN32_EXECUTABLE true
)

install(TARGETS ${SFRCALIB_MAINWIN} RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

if(WIN32)
    # set ICO file path
    set(ICON_FILE ${CMAKE_CURRENT_SOURCE_DIR}/resource/RobHand.ico)

    add_custom_command(TARGET ${SFRCALIB_MAINWIN}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E
                copy_if_different 
                    "$<TARGET_FILE:SARibbonBar>"                # get SARibbonBar library path
                    "$<TARGET_FILE_DIR:${SFRCALIB_MAINWIN}>"    # get SFRCALIB_MAINWIN (exe) dir
    )
	create_win32_resource_version(
		TARGET ${SFRCALIB_MAINWIN}
		FILENAME ${SFRCALIB_MAINWIN}
        ICON ${ICON_FILE}
        VERSION ${SFRCALIB_VERSION}
        DESCRIPTION "Ribbon application for robot-camera calibration."
        COMPANY_NAME "Shanghai Fanuc Robotics Co., Ltd."
        COPYRIGHT "Copyright (c) 2023, Shanghai Fanuc Robotics Co. All rights reserved."
        TRADEMARK "SfrCalib is a registered trademark of Shanghai Fanuc Robotics, Inc."
		EXT "exe"
	)
	visual_studio_qt_helper(${SFRCALIB_MAINWIN})
endif()
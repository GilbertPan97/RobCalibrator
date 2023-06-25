set(SRC_FILES
    StyleTrackballCamera.cpp    # used for vtk 3D viewer
    QVtkViewer.cpp
    RenderWidget.cpp            # used for opencv image viewer
    ImageViewer.cpp
    ChartViewer.cpp             # used for qt chart viewer
)

set(HEADER_FILES
    StyleTrackballCamera.h
    QVtkViewer.h
    RenderWidget.h
    ImageViewer.h
    ChartViewer.h
)
set(QRC_FILES QtDataViewer.qrc)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(QtDataViewer ${HEADER_FILES} ${SRC_FILES} ${QRC_FILES})

# find qt package: for all component
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets Charts REQUIRED)

if(QT_VERSION_MAJOR STREQUAL "5")
    set_target_properties(${library_name} PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON)
elseif(QT_VERSION_MAJOR STREQUAL "6")
    set_target_properties(${library_name} PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON)
endif()

# find vtk package: used for QtDataViewer
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

# find opencv package: used for QtDataViewer
find_package(OpenCV REQUIRED HINTS ${THIRD_PARTY})

target_link_libraries(QtDataViewer PUBLIC 
                      Qt${QT_VERSION_MAJOR}::Core 
                      Qt${QT_VERSION_MAJOR}::Gui 
                      Qt${QT_VERSION_MAJOR}::Widgets
                      Qt${QT_VERSION_MAJOR}::Charts
                      ${VTK_LIBRARIES} 
                      opencv_core)

set_target_properties(QtDataViewer PROPERTIES 
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

set(library_name "QtDataViewer")

include(CMakePackageConfigHelpers)
# configure_package_config_file(${CMAKE_SOURCE_DIR }/cmake/project-config-version.cmake.cmake.in
#                               ${CMAKE_CURRENT_BINARY_DIR}/${library_name}Config.cmake
#                               INSTALL_DESTINATION lib/cmake/${library_name})

write_basic_package_version_file(
    "${library_name}ConfigVersion.cmake"
    VERSION ${VERSION_SHORT}
    COMPATIBILITY SameMajorVersion
)

install(TARGETS QtDataViewer
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(FILES ${HEADER_FILES}
    DESTINATION include/QtDataViewer)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${library_name}ConfigVersion.cmake"
    DESTINATION lib/cmake/${library_name}
)

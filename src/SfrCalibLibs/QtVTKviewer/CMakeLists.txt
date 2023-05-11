set(SRC_FILES
    thinteractorstyletrackballcamera.cpp
    thQVTKOpenGLNativeWidget.cpp
)

set(HEADER_FILES
    thinteractorstyletrackballcamera.h
    thQVTKOpenGLNativeWidget.h
)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(QtVTKviewer ${HEADER_FILES} ${SRC_FILES} ${QRC_FILES})

# find qt package: for all component
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)

# find vtk package: used for QtVTKviewer
find_package(VTK REQUIRED HINTS ${THIRD_PARTY})
option(VTK_USE_QVTK_OPENGL_NATIVE "Use QVTKOpenGLNativeWidget for rendering" ON)
if (VTK_USE_QVTK_OPENGL_NATIVE)
  add_definitions(-DVTK_USE_QVTK_OPENGL_NATIVE)
endif()
message(STATUS "VTK package dir is: ${VTK_DIR}")

# find opencv package: used for QtVTKviewer
find_package(OpenCV REQUIRED HINTS ${THIRD_PARTY})

target_link_libraries(QtVTKviewer ${VTK_LIBRARIES} opencv_core)

set_target_properties(QtVTKviewer PROPERTIES 
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

install(TARGETS QtVTKviewer
  RUNTIME DESTINATION ${INSTALL_DIR}/bin/$<CONFIG>/QtVTKviewer
  LIBRARY DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/QtVTKviewer
  ARCHIVE DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/QtVTKviewer
)

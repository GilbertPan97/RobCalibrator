set(SRC_FILES
    ImageViewer.cpp
    RenderWidget.cpp
)

set(HEADER_FILES
    ImageViewer.h
    RenderWidget.h
)

set(QRC_FILES QtImgViewer.qrc)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(QtImgviewer ${HEADER_FILES} ${SRC_FILES} ${QRC_FILES})

# find qt package: for all component
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)

# find opencv package: used for QtImgviewer
find_package(OpenCV REQUIRED HINTS ${THIRD_PARTY})

target_link_libraries(QtImgviewer PUBLIC
                      Qt${QT_VERSION_MAJOR}::Core 
                      Qt${QT_VERSION_MAJOR}::Gui 
                      Qt${QT_VERSION_MAJOR}::Widgets
                      ${VTK_LIBRARIES} 
                      opencv_core)

set_target_properties(QtImgviewer PROPERTIES 
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

install(TARGETS QtImgviewer
  RUNTIME DESTINATION ${INSTALL_DIR}/bin/$<CONFIG>/QtImgviewer
  LIBRARY DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/QtImgviewer
  ARCHIVE DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/QtImgviewer
)
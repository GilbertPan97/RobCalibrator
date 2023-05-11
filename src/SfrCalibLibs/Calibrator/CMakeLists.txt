set(SRC_FILES
    algorithm.cpp
    calibrator.cpp
    data_processor.cpp
    utils.cpp
)

set(HEADER_FILES
    algorithm.h
    calibrator.h
    data_processor.h
    utils.h
    common.h
)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(Calibrator ${HEADER_FILES} ${SRC_FILES})

# # find qt package: for all component
# find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
# find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)

# find opencv package: used for Calibrator
find_package(OpenCV REQUIRED HINTS ${THIRD_PARTY})

target_link_libraries(Calibrator opencv_core opencv_calib3d opencv_highgui
    opencv_imgcodecs opencv_imgproc opencv_features2d opencv_viz)

# find Eigen package: used for Calibrator
find_package(Eigen3 REQUIRED HINTS ${THIRD_PARTY})
message(STATUS ${EIGEN3_INCLUDE_DIRS})
target_include_directories(Calibrator PRIVATE ${EIGEN3_INCLUDE_DIRS})

set_target_properties(Calibrator PROPERTIES 
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

install(TARGETS Calibrator
  RUNTIME DESTINATION ${INSTALL_DIR}/bin/$<CONFIG>/Calibrator
  LIBRARY DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/Calibrator
  ARCHIVE DESTINATION ${INSTALL_DIR}/lib/$<CONFIG>/Calibrator
)
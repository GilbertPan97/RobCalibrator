set(TARGET_NAME CamManager)

# prerequisite SR camera driver, opencv and nlohmann_json
find_package(OpenCV REQUIRED HINTS ${THIRD_PARTY})
find_package(SR7Link REQUIRED HINTS ${THIRD_PARTY})
find_package(nlohmann_json REQUIRED HINTS ${THIRD_PARTY}/json-3.11.2/share/cmake/nlohmann_json)

# find qt package: for all component
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets Charts REQUIRED)

set(SRC_FILES 
    camera.cpp
    sszn.cpp
    grabThread.cpp
    SFRSensorManager.cpp)

set(HEADER_FILES
    camera.h
    sszn.h
    grabThread.h
    SFRSensorManager.h)

set(FORM_FILES SFRSensorManager.ui)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_library(${TARGET_NAME} ${SRC_FILES} ${HEADER_FILES} ${FORM_FILES})

set_target_properties(${TARGET_NAME} PROPERTIES
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

target_link_libraries(${TARGET_NAME} PRIVATE ${SR7Link_LIBRARIES})

target_include_directories(${TARGET_NAME} PRIVATE
                        ${SR7Link_INCLUDE_DIRS}
                        ${CMAKE_CURRENT_SOURCE_DIR}
                        ${CMAKE_CURRENT_SOURCE_DIR}/../QtPropertyBrowser)

target_link_libraries(${TARGET_NAME} PRIVATE 
                Qt${QT_VERSION_MAJOR}::Core 
                Qt${QT_VERSION_MAJOR}::Gui 
                Qt${QT_VERSION_MAJOR}::Widgets
                Qt${QT_VERSION_MAJOR}::Charts
                nlohmann_json::nlohmann_json
                QtPropertyBrowser opencv_core opencv_calib3d 
                opencv_highgui opencv_imgcodecs opencv_imgproc 
                opencv_features2d opencv_viz)



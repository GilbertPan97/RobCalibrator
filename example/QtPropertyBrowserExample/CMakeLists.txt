
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

set(QRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/demo.qrc)
  
set(CMAKE_AUTORCC ON)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_ROOT_DIR}/build
)

add_executable(QtPropertyBrowserExample main.cpp ${QRC_FILES})
target_link_libraries(QtPropertyBrowserExample
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    QtPropertyBrowser
)

target_include_directories(QtPropertyBrowserExample PUBLIC
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src/SfrCalibLibs/QtPropertyBrowser"
)

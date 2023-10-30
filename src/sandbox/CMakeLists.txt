cmake_minimum_required(VERSION 3.5)
set(SFRCALIB_MAINWIN MainWindow)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find qt package: for main window exec
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)
message(STATUS "current Qt version is Qt${QT_VERSION_MAJOR}.${QT_VERSION_MINOR}.${QT_VERSION_PATCH}")

# build all dependent libs before main window
add_subdirectory(SfrCalibLibs)

# build main window execution
add_executable(${SFRCALIB_MAINWIN} mainwindow.h mainwindow.cpp main.cpp mainwindow.qrc)
target_include_directories(${SFRCALIB_MAINWIN}  PRIVATE 
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/SARibbonBar"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/QtVTKviewer"
                    "${CMAKE_CURRENT_SOURCE_DIR}/SfrCalibLibs/QtPropertyBrowser")
message(STATUS "Sfr calibration bin dir is: ${SFRCALIB_BIN_NAME}")

target_link_libraries(${SFRCALIB_MAINWIN} PRIVATE 
                      QtAdvancedDocking
                      QtPropertyBrowser
                      QtVTKviewer
                      SARibbonBar)
target_link_libraries(${SFRCALIB_MAINWIN} PUBLIC
                      Qt${QT_VERSION_MAJOR}::Core 
                      Qt${QT_VERSION_MAJOR}::Gui 
                      Qt${QT_VERSION_MAJOR}::Widgets)
                                           
set_target_properties(${SFRCALIB_MAINWIN} PROPERTIES 
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    VERSION ${VERSION_SHORT}
    EXPORT_NAME ${EXPAMPLE_NAME}
    ARCHIVE_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SFRCALIB_BIN_DIR}"
)

install(TARGETS ${SFRCALIB_MAINWIN} RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

if(WIN32)
    add_custom_command(TARGET ${SFRCALIB_MAINWIN}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E
                copy_if_different 
                    "$<TARGET_FILE:SARibbonBar>" 
                    "$<TARGET_FILE_DIR:${SFRCALIB_MAINWIN}>"
    )
	create_win32_resource_version(
		TARGET ${SFRCALIB_MAINWIN}
		FILENAME ${SFRCALIB_MAINWIN}
		EXT "exe"
		DESCRIPTION "Example application for Qt Ribbon Control"
	)
	visual_studio_qt_helper(${SFRCALIB_MAINWIN})
endif()

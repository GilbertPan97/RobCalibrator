cmake_minimum_required(VERSION 3.5)
project(MainWindowExample VERSION "1.0.0.0") 
set(SARIBBON_EXPAMPLE_NAME MainWindowExample)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# qt package import, minimum required 5.7
find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} 5.7 COMPONENTS Core Gui Widgets REQUIRED)

set(SRC_FILES
    mainwindow.cpp  
    thQVTKOpenGLNativeWidget.cpp
    thinteractorstyletrackballcamera.cpp
)

set(HEADER_FILES
    mainwindow.h
    thQVTKOpenGLNativeWidget.h
    thinteractorstyletrackballcamera.h
)

set(QRC_FILES saribbonresouce.qrc)

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})
source_group("Resource Files" FILES ${QRC_FILES})

add_executable(${SARIBBON_EXPAMPLE_NAME} main.cpp ${SRC_FILES} ${HEADER_FILES} ${QRC_FILES})

target_include_directories(${SARIBBON_EXPAMPLE_NAME}  PRIVATE 
                      "${CMAKE_SOURCE_DIR}/src/SfrCalibLibs/SARibbonBar")

target_link_libraries(${SARIBBON_EXPAMPLE_NAME} PRIVATE
                      QtVTKviewer
                      SARibbonBar)
target_link_libraries(${SARIBBON_EXPAMPLE_NAME} PUBLIC
                      Qt${QT_VERSION_MAJOR}::Core 
                      Qt${QT_VERSION_MAJOR}::Gui 
                      Qt${QT_VERSION_MAJOR}::Widgets)
                                           
set_target_properties(${SARIBBON_EXPAMPLE_NAME} PROPERTIES 
    AUTOMOC ON
    AUTORCC ON
    AUTOUIC ON
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    ARCHIVE_OUTPUT_DIRECTORY "${SARIBBON_BIN_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${SARIBBON_BIN_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${SARIBBON_BIN_DIR}"
)

install(TARGETS ${SARIBBON_EXPAMPLE_NAME} RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)

if(WIN32)
    add_custom_command(TARGET ${SARIBBON_EXPAMPLE_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E
                copy_if_different 
                    "$<TARGET_FILE:SARibbonBar>" 
                    "$<TARGET_FILE_DIR:${SARIBBON_EXPAMPLE_NAME}>"
    )
	create_win32_resource_version(
		TARGET ${SARIBBON_EXPAMPLE_NAME}
		FILENAME ${SARIBBON_EXPAMPLE_NAME}
		EXT "exe"
		DESCRIPTION "Example application for Qt Ribbon Control"
	)
	visual_studio_qt_helper(${SARIBBON_EXPAMPLE_NAME})
endif()

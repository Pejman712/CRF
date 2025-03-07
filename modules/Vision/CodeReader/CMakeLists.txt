#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playan Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "CodeReader")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
    EventLogger
    ErrorHandler
    ${CURLPP_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
set(INCLUDE_DIRS ./include
    ./tests/
    ${CURLPP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
set(SRC_FILES ./tests/DownloadImageFromURL.cpp
)

if(NOT DISABLE_QRCodeCVDetector)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${OpenCV_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Detector/QRCodeCVDetector/QRCodeCVDetector.cpp
    )
endif()

if(NOT DISABLE_ArUcoCVDetector)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${OpenCV_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Detector/ArUcoCVDetector/ArUcoCVDetector.cpp
    )
endif()

if(NOT DISABLE_QRCodeCVDecoder)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${OpenCV_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Decoder/QRCodeCVDecoder/QRCodeCVCurvedDecoder.cpp
                  ./src/Decoder/QRCodeCVDecoder/QRCodeCVDecoder.cpp
    )
endif()

if(NOT DISABLE_ZBarDecoder)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${OpenCV_LIBRARIES}
                    ${ZBar_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
                     ${ZBar_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Decoder/ZBarDecoder/ZBarDecoder.cpp
    )
endif()

if(NOT DISABLE_CodeBackgroundReader)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Cameras
                    Types
                    ${OpenCV_LIBRARIES}
                    ${VISP_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS}
                     ${VISP_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Reader/CodeBackgroundReader/CodeBackgroundReader.cpp
    )
endif()

if (DISABLE_QRCodeCVDetector AND DISABLE_OpenCVDecoder AND DISABLE_ZBarDecoder AND DISABLE_CodeBackgroundReader)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_QRCodeCVDetector AND NOT DISABLE_QRCodeCVDecoder AND NOT DISABLE_CodeBackgroundReader)
    add_executable(QRCodeReaderOpenCVSample ./samples/OpenCV/QRCodeReaderOpenCVSample.cpp)
    target_link_libraries(QRCodeReaderOpenCVSample ${MODULE_NAME})
endif()

if (NOT DISABLE_ZBarDecoder AND NOT DISABLE_CodeBackgroundReader)
    add_executable(QRCodeReaderZBarSample ./samples/ZBar/QRCodeReaderZBarSample.cpp)
    target_link_libraries(QRCodeReaderZBarSample ${MODULE_NAME})
endif()

if (NOT DISABLE_ArUcoCVDetector)
    add_executable(ArUcoCVDetectorSample ./samples/OpenCV/ArUcoCVDetectorSample.cpp)
    target_link_libraries(ArUcoCVDetectorSample ${MODULE_NAME})
endif()

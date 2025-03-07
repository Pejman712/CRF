#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(CodeReader_TESTS_FILES main.cpp
)
set(MODULE_LIBS
    ${CURLPP_LIBRARIES}
    ${OpenCV_LIBRARIES}
    gmock
    CodeReader
)
set(INCLUDE_DIRS
    ./
    ${CURLPP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
set(SRC_FILES
    ./DownloadImageFromURL.cpp
)

if(NOT DISABLE_QRCodeCVDetector)
    set(CodeReader_TESTS_FILES ${CodeReader_TESTS_FILES}
                                 ./Detector/QRCodeCVDetectorTests.cpp
    )
endif()

if(NOT DISABLE_ArUcoCVDetector)
    set(CodeReader_TESTS_FILES ${CodeReader_TESTS_FILES}
                                 ./Detector/ArUcoCVDetectorTests.cpp
    )
endif()

if(NOT DISABLE_QRCodeCVDecoder)
    set(CodeReader_TESTS_FILES ${CodeReader_TESTS_FILES}
                                 ./Decoder/QRCodeCVCurvedDecoderTests.cpp
                                 ./Decoder/QRCodeCVDecoderTests.cpp
    )
endif()

if(NOT DISABLE_ZBarDecoder)
    set(CodeReader_TESTS_FILES ${CodeReader_TESTS_FILES}
                                 ./Decoder/ZBarDecoderTests.cpp
    )
endif()

if(NOT DISABLE_CodeBackgroundReader)
    set(CodeReader_TESTS_FILES ${CodeReader_TESTS_FILES}
                                 ./Reader/CodeBackgroundReaderTests.cpp
    )
endif()

add_executable(ut-codereader ${CodeReader_TESTS_FILES})
target_include_directories(ut-codereader PUBLIC ${INCLUDE_DIRS})
target_link_libraries(ut-codereader ${MODULE_LIBS})
add_test(CodeReaderTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-codereader)

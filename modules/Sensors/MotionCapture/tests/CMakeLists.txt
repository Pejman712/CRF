#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giancarlo D'Ago CERN BE/CEM/MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set(MOTION_CAPTURE_TESTS_FILES main.cpp
)

if (NOT DISABLE_Vicon)
    set(MOTION_CAPTURE_TESTS_FILES ${MOTION_CAPTURE_TESTS_FILES}
                                   ./ViconTests.cpp
    )
endif()

add_executable(ut-motioncapture ${MOTION_CAPTURE_TESTS_FILES})
target_link_libraries(ut-motioncapture gmock
                                       MotionCapture
)

add_test(MotionCaptureUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-motioncapture)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MOTION_CONTROLLER_TESTS_FILES main.cpp
)

if (NOT DISABLE_PathFollower)
    set(MOTION_CONTROLLER_TESTS_FILES ${MOTION_CONTROLLER_TESTS_FILES}
                                      PathFollowerTests/PathFollowerTests.cpp
    )
endif()

if (NOT DISABLE_Teleoperation)
    set(MOTION_CONTROLLER_TESTS_FILES ${MOTION_CONTROLLER_TESTS_FILES}
                                      TeleoperationTests/TeleoperationTests.cpp
    )
endif()

if (NOT DISABLE_MotionControllerCommunicationPoint)
    set(MOTION_CONTROLLER_TESTS_FILES ${MOTION_CONTROLLER_TESTS_FILES}
                               MotionControllerCommunicationPointTests/MotionControllerCommunicationPointTests.cpp
    )
endif()

# This test opens ports in the host machine, in the new runners this is not allowed
# We are waiting for a solution from IT, otherwise we need to change this test
if (NOT DISABLE_MotionControllerClient)
    set(MOTION_CONTROLLER_TESTS_FILES ${MOTION_CONTROLLER_TESTS_FILES}
                                # MotionControllerClientTests/MotionControllerClientTests.cpp
    )
endif()

add_executable(ut-motioncontroller ${MOTION_CONTROLLER_TESTS_FILES})
target_link_libraries(ut-motioncontroller gmock
                                    MotionController
                                    EventLogger
)

add_test(MotionControllerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-motioncontroller)

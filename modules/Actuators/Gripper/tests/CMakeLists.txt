#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(GRIPPER_TESTS_FILES main.cpp
)

if (NOT DISABLE_ECBPMiSerial)
    set(GRIPPER_TESTS_FILES ${GRIPPER_TESTS_FILES}
                            ./ECBPMiSerialTest.cpp
    )
endif()

add_executable(ut-gripper ${GRIPPER_TESTS_FILES})
target_link_libraries(ut-gripper gmock
                                 Gripper
)

add_test(GripperUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-gripper)

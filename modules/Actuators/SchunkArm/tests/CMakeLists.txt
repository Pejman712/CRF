#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(SCHUNKARM_TESTS_FILES main.cpp
                          SchunkDeviceTests.cpp
                          SchunkArmTests.cpp
                          SchunkGripperTests.cpp
)

add_executable(ut-schunkarm ${SCHUNKARM_TESTS_FILES})
target_link_libraries(ut-schunkarm gmock
                                   SchunkArm
)

add_test(SchunkArmUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-schunkarm)

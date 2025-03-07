#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(ROBOTARM_TESTS_FILES ./main.cpp
                         ./RobotArmConfigurationTests.cpp
)

add_executable(ut-robotarm ${ROBOTARM_TESTS_FILES})
target_link_libraries(ut-robotarm gmock
                                  RobotArm
)

add_test(RobotArmUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotarm)

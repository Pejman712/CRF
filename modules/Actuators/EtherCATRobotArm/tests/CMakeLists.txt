#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(ETHERCATROBOTARM_TESTS_FILES main.cpp
                                 EtherCATRobotArmTests.cpp
)

add_executable(ut-ethercatrobotarm ${ETHERCATROBOTARM_TESTS_FILES})
target_link_libraries(ut-ethercatrobotarm gmock
                                  EtherCATRobotArm
)

add_test(EtherCATRobotArmUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ethercatrobotarm)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(ROBOTARMKINEMATICS_TESTS_FILES main.cpp
)

if (NOT DISABLE_RobotArmKDLKinematics)
    set(ROBOTARMKINEMATICS_TESTS_FILES ${ROBOTARMKINEMATICS_TESTS_FILES}
                                       ./RobotArmKDLKinematics/RobotArmKDLKinematicsTests.cpp
    )
endif()

add_executable(ut-robotarmkinematics ${ROBOTARMKINEMATICS_TESTS_FILES})
target_link_libraries(ut-robotarmkinematics gmock
                                            RobotArmKinematics
)

add_test(RobotArmKinematicsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotarmkinematics)

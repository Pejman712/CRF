#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MISSION_UTILITY_TESTS main.cpp
)

if(NOT DISABLE_DeployableTIMRPWagonArm)
    set(MISSION_UTILITY_TESTS ${MISSION_UTILITY_TESTS}
                              DeployableTIMRPWagonArmTests/DeployableTIMRPWagonArmTests.cpp
    )
endif()

if(NOT DISABLE_DeployableRobotArm)
    set(MISSION_UTILITY_TESTS ${MISSION_UTILITY_TESTS}
                               DeployableRobotArmTests/DeployableRobotArmTests.cpp
    )
endif()

if(NOT DISABLE_TIMMovement)
    set(MISSION_UTILITY_TESTS ${MISSION_UTILITY_TESTS}
                              TIMMovementTests/TIMMovementTests.cpp
    )
endif()

if(NOT DISABLE_DeployableRobot)
    set(MISSION_UTILITY_TESTS ${MISSION_UTILITY_TESTS}
                              DeployableRobotTests/DeployableRobotTests.cpp
    )
endif()

add_executable(ut-missionutility ${MISSION_UTILITY_TESTS})
target_link_libraries(ut-missionutility gmock
                                        EventLogger
                                        MissionUtility
)

add_test(MissionUtilityTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-missionutility)

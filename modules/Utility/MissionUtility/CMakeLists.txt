#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN EN/SMM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "MissionUtility")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_DeployableTIMRPWagonArm)
    set(MODULE_LIBS ${MODULE_LIBS}
                    TIMRPWagon
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeployableTIMRPWagonArm/DeployableTIMRPWagonArm.cpp
    )
endif()

if(NOT DISABLE_DeployableRobotArm)
    set(MODULE_LIBS ${MODULE_LIBS}
                    RobotArmController
                    Types
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeployableRobotArm/DeployableRobotArm.cpp
    )
endif()

if(NOT DISABLE_TIMMovement)
    set(MODULE_LIBS ${MODULE_LIBS}
                    TIM
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMMovement/TIMMovement.cpp
    )
endif()

if(NOT DISABLE_DeployableRobot)
    set(MODULE_LIBS ${MODULE_LIBS}
                    MotionController
                    Types
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeployableRobot/DeployableRobot.cpp
    )
endif()

if(DISABLE_DeployableTIMRPWagonArm AND DISABLE_DeployableRobotArm AND DISABLE_TIMMovement AND DISABLE_DeployableRobot)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

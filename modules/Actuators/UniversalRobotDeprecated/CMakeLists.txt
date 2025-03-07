#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Hannes Gamper CERN BE/CEM/MRO 2021                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "UniversalRobotDeprecated")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                RobotArm
                RobotArmKinematics
                ur_rtde::rtde
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/UniversalRobotRTDE.cpp
              ./src/UniversalRobotConfiguration.cpp
              ./src/UniversalRobotRTDEInterface.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(UniversalRobotReadJointPosExample ./samples/UniversalRobotReadJointPosExample.cpp)
target_link_libraries(UniversalRobotReadJointPosExample ${MODULE_NAME})

if(NOT DISABLE_GeometricMethods)
    add_executable(UniversalRobotPosCtrlExample ./samples/UniversalRobotPosCtrlExample.cpp)
    target_link_libraries(UniversalRobotPosCtrlExample ${MODULE_NAME} GeometricMethods)
endif()

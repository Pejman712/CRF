#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "RobotArmKinematics")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Types
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_RobotArmKDLKinematics)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    RobotArm
                    TRACInverseKinematics
                    ${orocos_kdl_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotArmKDLKinematics/RobotArmKDLKinematics.cpp
    )
endif()

if (DISABLE_RobotArmKDLKinematics)
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

if(NOT DISABLE_RobotArmKDLKinematics)
    add_executable(RobotArmKDLInverseKinematicsSample ./samples/RobotArmKDLInverseKinematicsSample.cpp)
    target_link_libraries(RobotArmKDLInverseKinematicsSample ${MODULE_NAME})
endif()

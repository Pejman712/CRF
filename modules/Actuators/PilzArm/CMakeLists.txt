#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "PilzArm")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS
    ./include
    ${EIGEN3_INCLUDE_DIR}
)

set(MODULE_LIBS
    CommonInterfaces
    Gripper
    EventLogger
    RobotArm
    CANOpenDevicesDeprecated
    SchunkArm
    ${orocos_kdl_LIBRARIES}
)

set(SRC_FILES
    ./src/PilzArm.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

add_executable(PilzArmSample ./samples/PilzArmSample.cpp)
target_link_libraries(PilzArmSample PilzArm)

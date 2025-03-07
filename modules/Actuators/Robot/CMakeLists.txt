#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Robot")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                ErrorHandler
                Types
                ForwardKinematics
                Jacobian
                KinematicChain
)
set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES ./src/RobotConfiguration.cpp
)

# It's VERY IMPORTANT that KinovaGen3 links before CiARobot and EtherCATRobot
# For more info refer to SOEMLinkingError in the docs (jplayang)
if(NOT DISABLE_KinovaGen3)
    set(MODULE_LIBS ${MODULE_LIBS}
                    KortexAPI
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/KinovaGen3/KinovaGen3.cpp
                  ./src/KinovaGen3/KinovaGen3Configuration.cpp
    )
endif()

if(NOT DISABLE_CiA402Robot)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CANopenDrivers
                    EtherCATDrivers
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CiA402Robot/CiA402Robot.cpp
                  ./src/CiA402Robot/CiA402RobotConfiguration.cpp
    )
endif()

if(NOT DISABLE_UniversalRobot)
    set(MODULE_LIBS ${MODULE_LIBS}
                    UniversalRobotRTDE
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/UniversalRobot/UniversalRobot.cpp
                  ./src/UniversalRobot/UniversalRobotConfiguration.cpp
    )
endif()

if(NOT DISABLE_KinovaJaco2)
    set(MODULE_LIBS ${MODULE_LIBS}
                    KinovaJacoAPI
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                    ./src/KinovaJaco2/KinovaJaco2.cpp
                    ./src/KinovaJaco2/KinovaJaco2Configuration.cpp
    )
endif()

if(NOT DISABLE_EtherCATRobot)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EtherCATDevices
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                    ./src/EtherCATRobot/EtherCATRobot.cpp
                    ./src/EtherCATRobot/EtherCATRobotConfiguration.cpp
    )
endif()

if(NOT DISABLE_Virtuose6DTAO)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Haption
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Virtuose6DTAO/Virtuose6DTAO.cpp
                  ./src/Virtuose6DTAO/Virtuose6DTAOConfiguration.cpp
    )
endif()

if(NOT DISABLE_CombinedRobot)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CombinedRobot/CombinedRobot.cpp
                  ./src/CombinedRobot/CombinedRobotConfiguration.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_CiA402Robot)
    add_executable(CiA402RobotSample ./samples/CiA402RobotSample.cpp)
    target_link_libraries(CiA402RobotSample ${MODULE_NAME})
endif()

if(NOT DISABLE_EtherCATRobot)
    add_executable(EtherCATRobotSample ./samples/EtherCATRobotSample.cpp)
    target_link_libraries(EtherCATRobotSample ${MODULE_NAME})
endif()

if(NOT DISABLE_KinovaGen3)
    add_executable(KinovaGen3Sample ./samples/KinovaGen3Sample.cpp)
    target_link_libraries(KinovaGen3Sample ${MODULE_NAME})
endif()

if(NOT DISABLE_UniversalRobot)
    add_executable(UniversalRobotJointReadPosExample ./samples/UniversalRobotJointReadPosExample.cpp)
    target_link_libraries(UniversalRobotJointReadPosExample ${MODULE_NAME})
    add_executable(UniversalRobotJointCtrlPosExample ./samples/UniversalRobotJointCtrlPosExample.cpp)
    target_link_libraries(UniversalRobotJointCtrlPosExample ${MODULE_NAME})
    add_executable(UniversalRobotStatusReadExample ./samples/UniversalRobotStatusReadExample.cpp)
    target_link_libraries(UniversalRobotStatusReadExample ${MODULE_NAME})
    add_executable(UniversalRobotJointCtrlVelExample ./samples/UniversalRobotJointCtrlVelExample.cpp)
    target_link_libraries(UniversalRobotJointCtrlVelExample ${MODULE_NAME})
    add_executable(UniversalRobotReadAllExample ./samples/UniversalRobotReadAllExample.cpp)
    target_link_libraries(UniversalRobotReadAllExample ${MODULE_NAME})
    add_executable(UniversalRobotTaskCtrlPosExample ./samples/UniversalRobotTaskCtrlPosExample.cpp)
    target_link_libraries(UniversalRobotTaskCtrlPosExample ${MODULE_NAME})
    add_executable(UniversalRobotReadAllValuesTimed ./samples/UniversalRobotReadAllValuesTimed.cpp)
    target_link_libraries(UniversalRobotReadAllValuesTimed ${MODULE_NAME})
endif()

if(NOT DISABLE_Virtuose6DTAO)
    add_executable(Virtuose6DTAOForcePlaneSample ./samples/Virtuose6DTAOForcePlaneSample.cpp)
    target_link_libraries(Virtuose6DTAOForcePlaneSample ${MODULE_NAME})
endif()

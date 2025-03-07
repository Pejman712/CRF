#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(ROBOT_TESTS_FILES main.cpp)

if (NOT DISABLE_CiA402Robot)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
        CiA402Robot/CiA402RobotTests.cpp
    )
endif()

if (NOT DISABLE_CombinedRobot)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
        CombinedRobot/CombinedRobotTests.cpp
    )
endif()

if (NOT DISABLE_UniversalRobot)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
                          UniversalRobot/UniversalRobotTests.cpp
    )
endif()

if (NOT DISABLE_KinovaJaco2)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
                          KinovaJaco2/KinovaJaco2Tests.cpp
                          KinovaJaco2/KinovaJaco2ConfigurationTests.cpp
    )
endif()

if (NOT DISABLE_KinovaGen3)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
                          KinovaGen3/KinovaGen3Tests.cpp
                          KinovaGen3/KinovaGen3ConfigurationTests.cpp
    )
endif()

if (NOT DISABLE_EtherCATRobot)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
                          EtherCATRobot/EtherCATRobotConfigurationTests.cpp
    )
endif()

if (NOT DISABLE_Virtuose6DTAO)
    set(ROBOT_TESTS_FILES ${ROBOT_TESTS_FILES}
                          Virtuose6DTAO/Virtuose6DTAOTests.cpp
    )
endif()

add_executable(ut-robot ${ROBOT_TESTS_FILES})
target_link_libraries(ut-robot gmock
                               Robot
)

add_test(RobotUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robot)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(ROBOTBASECONTROLLER_TESTS_FILES main.cpp
)

if (NOT DISABLE_RobotBaseControllerCommunicationPoint)
    set(ROBOTBASECONTROLLER_TESTS_FILES ${ROBOTBASECONTROLLER_TESTS_FILES}
                                        RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointTests.cpp
    )
endif()

if (NOT DISABLE_RobotBaseVelocityController)
    set(ROBOTBASECONTROLLER_TESTS_FILES ${ROBOTBASECONTROLLER_TESTS_FILES}
                                        RobotBaseVelocityController/RobotBaseVelocityControllerTests.cpp
    )
endif()

add_executable(ut-robotbasecontroller ${ROBOTBASECONTROLLER_TESTS_FILES})
target_link_libraries(ut-robotbasecontroller gmock
                                            EventLogger
                                            RobotBaseController
)

add_test(RobotBaseControllerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotbasecontroller)

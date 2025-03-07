#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(ROBOTARMCONTROLLER_TESTS_FILES main.cpp
                                   RobotArmBehaviourForTests/RobotArmBehaviourForTests.cpp
)

if (NOT DISABLE_RobotArmControllerClient)
    set(ROBOTARMCONTROLLER_TESTS_FILES ${ROBOTARMCONTROLLER_TESTS_FILES}
                                       RobotArmControllerClient/RobotArmControllerClientTests.cpp
    )
endif()

if (NOT DISABLE_RobotArmControllerCommunicationPoint)
    set(ROBOTARMCONTROLLER_TESTS_FILES ${ROBOTARMCONTROLLER_TESTS_FILES}
                                       RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointTests.cpp
    )
endif()

if (NOT DISABLE_RobotArmVelocityController)
    set(ROBOTARMCONTROLLER_TESTS_FILES ${ROBOTARMCONTROLLER_TESTS_FILES}
                                       RobotArmVelocityController/RobotArmVelocityControllerTests.cpp
    )
endif()

add_executable(ut-robotarmcontroller ${ROBOTARMCONTROLLER_TESTS_FILES})
target_link_libraries(ut-robotarmcontroller gmock
                                            EventLogger
                                            RobotArmController
)

add_test(RobotArmControllerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotarmcontroller)

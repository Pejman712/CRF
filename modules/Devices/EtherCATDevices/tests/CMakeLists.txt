#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(ETHERCATDEVICES_TESTS_FILES ./main.cpp
                                ./EtherCATManagerTests.cpp
                                ./EtherCATMotorTests.cpp
)

if (NOT DISABLE_TIMRobotArmWagonMotors)
    set(ETHERCATDEVICES_TESTS_FILES ${ETHERCATDEVICES_TESTS_FILES}
                                    TIMRobotArmWagonMotorsTests.cpp
    )
endif()

add_executable(ut-ethercatdevices ${ETHERCATDEVICES_TESTS_FILES})
target_link_libraries(ut-ethercatdevices gmock
                                         EtherCATDevices
)

add_test(EtherCATDevicesUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ethercatdevices)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(CANOPENDEVICESDEPRECATED_TESTS_FILES ./main.cpp
                                         ./ObjectDictionaryRegisterTests.cpp
                                         ./ObjectDictionaryTests.cpp
                                         ./CANOpenContextTests.cpp
                                         ./CANOpenMotorTests.cpp
                                         ./CANOpenIOModuleTests.cpp
)

add_executable(ut-canopendevicesdeprecated ${CANOPENDEVICESDEPRECATED_TESTS_FILES})
target_link_libraries(ut-canopendevicesdeprecated gmock
                                                  CANOpenDevicesDeprecated
)

add_test(CANOpenDevicesDeprecatedUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-canopendevicesdeprecated)

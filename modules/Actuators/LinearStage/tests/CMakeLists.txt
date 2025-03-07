#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(LINEARSTAGE_TESTS_FILES main.cpp
                            LinearStageConfigurationTests.cpp
)

if (NOT DISABLE_CANOpenLinearStage)
    set(LINEARSTAGE_TESTS_FILES ${LINEARSTAGE_TESTS_FILES}
                                CANOpenLinearStageTests.cpp
    )
endif()

if (NOT DISABLE_EtherCATLinearActuator)
    set(LINEARSTAGE_TESTS_FILES ${LINEARSTAGE_TESTS_FILES}
                                EtherCATLinearActuatorTests/EtherCATLinearActuatorTests.cpp
    )
endif()

if (NOT DISABLE_LinearActuatorCommunicationPoint)
    set(LINEARSTAGE_TESTS_FILES ${LINEARSTAGE_TESTS_FILES}
                                LinearActuatorCommunicationPointTests.cpp
    )
endif()

add_executable(ut-linearstage ${LINEARSTAGE_TESTS_FILES})
target_link_libraries(ut-linearstage gmock
                                     LinearStage
)

add_test(LinearStageUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-linearstage)

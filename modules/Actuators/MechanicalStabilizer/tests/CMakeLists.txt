#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MECHANICALSTABILIZER_TESTS_FILES main.cpp
)

if (NOT DISABLE_RadioactiveSourceMechanicalStabilizer)
    set(MECHANICALSTABILIZER_TESTS_FILES ${MECHANICALSTABILIZER_TESTS_FILES}
                              TIMStabilizerTests.cpp
    )
endif()

add_executable(ut-mechanicalstabilizer ${MECHANICALSTABILIZER_TESTS_FILES})
target_link_libraries(ut-mechanicalstabilizer gmock
                                   EventLogger
                                   MechanicalStabilizer
)

add_test(MechanicalStabilizerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-mechanicalstabilizer)

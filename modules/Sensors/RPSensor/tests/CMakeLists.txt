#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(RPSENSOR_TESTS_FILES main.cpp
)

if (NOT DISABLE_AtomtexBDKG24)
    set(RPSENSOR_TESTS_FILES ${RPSENSOR_TESTS_FILES}
                             ./AtomtexBDKG24Tests.cpp
    )
endif()

add_executable(ut-rpsensor ${RPSENSOR_TESTS_FILES})
target_link_libraries(ut-rpsensor gmock
                                 RPSensor
                                 EventLogger
)

add_test(RPSensorUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-rpsensor)

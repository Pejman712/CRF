#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TESTS_FILES main.cpp
)

if (NOT DISABLE_FTM115)
    set(TESTS_FILES ${TESTS_FILES}
                    FTM115Tests.cpp
    )
endif()

add_executable(ut-ftsensor ${TESTS_FILES})
target_link_libraries(ut-ftsensor gmock
                                  FTSensor
)

add_test(FTSensorUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ftsensor)

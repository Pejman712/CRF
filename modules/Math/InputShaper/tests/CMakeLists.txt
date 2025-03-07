#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################


set(INPUT_SHAPER_TESTS_FILES main.cpp
)

if(NOT DISABLE_CubicPolynomialShaper)
    set(INPUT_SHAPER_TESTS_FILES ${INPUT_SHAPER_TESTS_FILES}
                                      CubicPolynomialShaperTests.cpp
    )
endif()

add_executable(ut-inputshaper ${INPUT_SHAPER_TESTS_FILES})
target_link_libraries(ut-inputshaper gmock
                                          InputShaper)

add_test(InputShaperUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-inputshaper)

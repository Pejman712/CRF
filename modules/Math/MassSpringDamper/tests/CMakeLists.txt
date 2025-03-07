#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playan Garai CERN BE/CEM/MRO 2021                                                                   ##
##         David Günter Forkel CERN BE/CEM/MRO 2021                                                                  ##
##         Jean Paul Sulca CERN BM-CEM-MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set(MASSSPRINGDAMPER_TEST_FILES ./main.cpp
)

if(NOT DISABLE_MassSpringDamperR1)
    set(MASSSPRINGDAMPER_TEST_FILES ${MASSSPRINGDAMPER_TEST_FILES}
                                    ./MSDR1ImpulseInvarianceIIRTests.cpp
                                    ./MSDR1StepInvarianceIIRTests.cpp
                                    ./MSDR1BilinearTransformIIRTests.cpp
    )
endif()

if(NOT DISABLE_MassSpringDamperRn)
    set(MASSSPRINGDAMPER_TEST_FILES ${MASSSPRINGDAMPER_TEST_FILES}
                                    ./MSDRnDecoupledTests.cpp
    )
endif()

add_executable(ut-massspringdamper ${MASSSPRINGDAMPER_TEST_FILES})
target_link_libraries(ut-massspringdamper gmock
                                          MassSpringDamper
)

add_test(MassSpringDamperUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-massspringdamper)

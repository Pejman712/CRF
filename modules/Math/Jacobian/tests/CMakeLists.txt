#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(JACOBIAN_TESTS_FILES main.cpp)

if (NOT DISABLE_URDFKinematicChain)
    set(JACOBIAN_TESTS_FILES ${JACOBIAN_TESTS_FILES}
                            JacobianTests.cpp
    )
endif()

add_executable(ut-jacobian ${JACOBIAN_TESTS_FILES})
target_link_libraries(ut-jacobian gmock
                                  Jacobian
)

add_test(JacobianUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-jacobian)

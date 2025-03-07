#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Ante Marić CERN BE/CEM/MRO 2023                                                                           ##
##                                                                                                                   ##
#######################################################################################################################


set(KINEMATIC_CHAIN_TESTS_FILES main.cpp
)

if (NOT DISABLE_URDFKinematicChain)
    set(KINEMATIC_CHAIN_TESTS_FILES ${KINEMATIC_CHAIN_TESTS_FILES}
                                      URDFKinematicChainTests.cpp
    )
endif()

add_executable(ut-kinematicchain ${KINEMATIC_CHAIN_TESTS_FILES})
target_link_libraries(ut-kinematicchain gmock
                                        KinematicChain)

add_test(KinematicChainUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-kinematicchain)

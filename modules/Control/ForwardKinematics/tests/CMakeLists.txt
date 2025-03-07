#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(FORWARDKINEMATICS_TESTS_FILES main.cpp
)

if (NOT DISABLE_MathExprForwardKinematics)
    set(FORWARDKINEMATICS_TESTS_FILES ${FORWARDKINEMATICS_TESTS_FILES}
                                      ./MathExprForwardKinematics/MathExprForwardKinematicsTests.cpp
    )
endif()

if (NOT DISABLE_KinChainForwardKinematics)
    set(FORWARDKINEMATICS_TESTS_FILES ${FORWARDKINEMATICS_TESTS_FILES}
                                      ./KinChainForwardKinematics/KinChainForwardKinematicsTests.cpp
    )
endif()

add_executable(ut-forwardkinematics ${FORWARDKINEMATICS_TESTS_FILES})
target_link_libraries(ut-forwardkinematics gmock
                                           ForwardKinematics
)

add_test(ForwardKinematicsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-forwardkinematics)

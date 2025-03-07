#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(INVERSEKINEMATICS_TESTS_FILES main.cpp
                                  ResultsIKTests.cpp
)

if (NOT DISABLE_CollisionAvoidanceSphere)
    set(INVERSEKINEMATICS_TESTS_FILES ${INVERSEKINEMATICS_TESTS_FILES}
                                      ./CollisionAvoidanceSphere/CollisionAvoidanceSphereTests.cpp
    )
endif()

if (NOT DISABLE_DesiredJointPositions)
    set(INVERSEKINEMATICS_TESTS_FILES ${INVERSEKINEMATICS_TESTS_FILES}
                                      ./DesiredJointPositions/DesiredJointPositionsTests.cpp
    )
endif()

if (NOT DISABLE_JointLimits)
    set(INVERSEKINEMATICS_TESTS_FILES ${INVERSEKINEMATICS_TESTS_FILES}
                                      ./JointLimits/JointLimitsTests.cpp
    )
endif()

if (NOT DISABLE_OptCLIK AND NOT DISABLE_OptOLIK AND NOT DISABLE_JointLimits)
    set(INVERSEKINEMATICS_TESTS_FILES ${INVERSEKINEMATICS_TESTS_FILES}
                                      ./OptCLIK/OptCLIKTests.cpp
    )
endif()

if (NOT DISABLE_OptOLIK AND NOT DISABLE_JointLimits)
    set(INVERSEKINEMATICS_TESTS_FILES ${INVERSEKINEMATICS_TESTS_FILES}
                                      ./OptOLIK/OptOLIKTests.cpp
    )
endif()

add_executable(ut-inversekinematics ${INVERSEKINEMATICS_TESTS_FILES})

if(NOT DISABLE_OptCLIK OR NOT DISABLE_OptOLIK)
    target_link_libraries(ut-inversekinematics gmock
                                               InverseKinematics
                                               GeometricMethods
    )
else()
    target_link_libraries(ut-inversekinematics gmock
                                               InverseKinematics
    )
endif()


add_test(InverseKinematicsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-inversekinematics)

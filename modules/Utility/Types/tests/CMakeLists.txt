#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Bartosz Sójka CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TYPES_TESTS_FILES main.cpp
                      JointTypes/VectorXdTests.cpp
                      JointTypes/JointPositionsTests.cpp
                      JointTypes/JointVelocitiesTests.cpp
                      JointTypes/JointAccelerationsTests.cpp
                      JointTypes/JointForceTorquesTests.cpp
                      JointTypes/JointSignalsTests.cpp
                      TaskTypes/Vector6dTests.cpp
                      TaskTypes/TaskPoseTests.cpp
                      TaskTypes/TaskVelocityTests.cpp
                      TaskTypes/TaskAccelerationTests.cpp
                      TaskTypes/TaskForceTorqueTests.cpp
                      TaskTypes/TaskSignalsTests.cpp
                      TaskTypes/TaskSpaceTests.cpp
                      ArithmeticTests.cpp
                      ComparisonTests.cpp
                      ReducedTaskSpaceComparisonTests.cpp
                      ConversionsTests.cpp
                      SignalsTests.cpp
                      JsonConvertersTests.cpp
)

add_executable(ut-types ${TYPES_TESTS_FILES})
target_link_libraries(ut-types gmock
                               Types
)

add_test(TypesUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-types)

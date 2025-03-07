#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TRAJECTORY_GENERATOR_DEPRECATED_TESTS_FILES main.cpp
                                                JointsTimeOptimalTrajectoryTests.cpp
                                                TaskLinearTrajectoryTests.cpp
                                                TaskTimeOptimalTrajectoryTests.cpp)

add_executable(ut-trajectorygeneratordeprecated ${TRAJECTORY_GENERATOR_DEPRECATED_TESTS_FILES})
target_link_libraries(ut-trajectorygeneratordeprecated gmock
                                                       TrajectoryGeneratorDeprecated
)

add_test(TrajectoryGeneratorDeprecatedUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-trajectorygeneratordeprecated)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################


set(TRAJECTORY_GENERATOR_TESTS_FILES main.cpp
)

if(NOT DISABLE_PointToPointJointsTrajectory)
    set(TRAJECTORY_GENERATOR_TESTS_FILES ${TRAJECTORY_GENERATOR_TESTS_FILES}
                                         PointToPointJointsTrajectoryTests.cpp
    )
endif()

if(NOT DISABLE_CubicJointsTrajectory)
    set(TRAJECTORY_GENERATOR_TESTS_FILES ${TRAJECTORY_GENERATOR_TESTS_FILES}
                                         CubicJointsTrajectoryTests.cpp
    )
endif()

if(NOT DISABLE_CubicTaskTrajectory)
    set(TRAJECTORY_GENERATOR_TESTS_FILES ${TRAJECTORY_GENERATOR_TESTS_FILES}
                                         CubicTaskTrajectoryTests.cpp
    )
endif()

if(NOT DISABLE_PreplannedTaskTrajectory)
    set(TRAJECTORY_GENERATOR_TESTS_FILES ${TRAJECTORY_GENERATOR_TESTS_FILES}
                                         PreplannedTaskTrajectoryTests.cpp
    )
endif()

add_executable(ut-trajectorygenerator ${TRAJECTORY_GENERATOR_TESTS_FILES})
target_link_libraries(ut-trajectorygenerator gmock TrajectoryGenerator)

add_test(TrajectoryGeneratorUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-trajectorygenerator)

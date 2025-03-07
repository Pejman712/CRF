#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TRAJECTORYPOINTGENERATOR_TESTS_FILES main.cpp
                                         ReflexxesTrajectoryPointGeneratorTests.cpp)

add_executable(ut-trajectorypointgenerator ${TRAJECTORYPOINTGENERATOR_TESTS_FILES})
target_link_libraries(ut-trajectorypointgenerator gmock
                                                  TrajectoryPointGenerator
)

add_test(TrajectoryPointGeneratorUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-trajectorypointgenerator)

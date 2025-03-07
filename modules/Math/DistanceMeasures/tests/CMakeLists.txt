#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

set(ERROR_COMPUTATION_TESTS_FILES main.cpp
                                  TaskPoseDistanceMeasuresTests.cpp
                                  RotationDistanceMeasuresTests.cpp
)

add_executable(ut-distancemeasures ${ERROR_COMPUTATION_TESTS_FILES})
target_link_libraries(ut-distancemeasures gmock
                                          DistanceMeasures)

add_test(DistanceMeasuresUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-distancemeasures)

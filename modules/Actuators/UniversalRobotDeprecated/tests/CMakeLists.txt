#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Hannes Gamper CERN BE/CEM/MRO 2021                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(UNIVERSALROBOTDEPRECATED_TESTS_FILES main.cpp
                                         UniversalRobotDeprecatedTests.cpp
)

add_executable(ut-universalrobotdeprecated ${UNIVERSALROBOTDEPRECATED_TESTS_FILES})
target_link_libraries(ut-universalrobotdeprecated gmock
                                                  UniversalRobotDeprecated
)

add_test(UniversalRobotDeprecatedUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-universalrobotdeprecated)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2017. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Pawel Ptasznik CERN EN/SMM/MRO 2017                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set(CLOSED_LOOP_CONTROLLER_TESTS_FILES main.cpp
                                       PIDControllerTests.cpp
)

add_executable(ut-closedloopcontroller ${CLOSED_LOOP_CONTROLLER_TESTS_FILES})
target_link_libraries(ut-closedloopcontroller gmock
                                              ClosedLoopController
)

add_test(ClosedLoopControllerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-closedloopcontroller)

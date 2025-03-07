#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(CERNBOT2_TESTS_FILES main.cpp
                         CERNBot2Tests.cpp
)

add_executable(ut-cernbot2 ${CERNBOT2_TESTS_FILES})
target_link_libraries(ut-cernbot2 gmock
                                  CERNBot2
)

add_test(CERNBot2UnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-cernbot2)

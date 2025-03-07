#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jean Paul Sulca CERN BM-CEM-MRO 2023                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TESTS_FILES main.cpp
                DigitalFilterTests.cpp
)

add_executable(ut-digitalfilter ${TESTS_FILES})
target_link_libraries(ut-digitalfilter gmock
                                       DigitalFilter
)

add_test(DigitalFilterUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-digitalfilter)

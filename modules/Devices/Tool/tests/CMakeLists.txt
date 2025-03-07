#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Sebastien Collomb CERN BE/CEM/MRO 2023                                                                    ##
##                                                                                                                   ##
#######################################################################################################################

set(TOOL_TESTS_FILES main.cpp

)

if (NOT DISABLE_ActiveToolCommunicationPoint)
    set(TOOL_TESTS_FILES ${TOOL_TESTS_FILES}
                         ActiveToolCommunicationPointTests.cpp
    )
endif()

add_executable(ut-tool ${TOOL_TESTS_FILES})
target_link_libraries(ut-tool gmock
                              Tool
)

add_test(ActiveToolUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-tool)

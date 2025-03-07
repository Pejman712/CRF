#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(ERROR_HANDLER_TESTS main.cpp
                               ResponseCodeTests/ResponseCodeTests.cpp
                               ExpectTests/ExpectTests.cpp
)

add_executable(ut-errorhandler ${ERROR_HANDLER_TESTS})
target_link_libraries(ut-errorhandler gmock
                                      EventLogger
                                      ErrorHandler)

add_test(ErrorHandlerTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-errorhandler)

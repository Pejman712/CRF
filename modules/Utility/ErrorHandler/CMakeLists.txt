#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "ErrorHandler")

if(DISABLE_${MODULE_NAME})
    return()
endif()

add_library(${MODULE_NAME} INTERFACE)
target_include_directories(${MODULE_NAME} INTERFACE ./include)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(CRFExpectedSample ./samples/CRFExpectedSample.cpp)
target_link_libraries(CRFExpectedSample ${MODULE_NAME})

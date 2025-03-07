#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jean Paul Sulca CERN BM-CEM-MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "DigitalFilter")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ErrorHandler
                EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/IIRFilter.cpp)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(DirectFormIIRFilterSample ./samples/DirectFormIIRFilterSample.cpp)
target_link_libraries(DirectFormIIRFilterSample ${MODULE_NAME})
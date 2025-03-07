#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "SOEMAPI")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS
    ./include
    ${SOEM_INCLUDE_DIRS}
)

set(SRC_FILES
    ./src/SOEMAPI.cpp
)

set(MODULE_LIBS
    ${SOEM_LIBRARIES}
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

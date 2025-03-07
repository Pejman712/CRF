#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "BLMTriggeringAnalysis")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/BLMTriggeringResult.cpp
)

if(NOT DISABLE_BLMTriggeringAnalysisClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/BLMTriggeringAnalysisClient/BLMTriggeringAnalysisClient.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_Sockets)
    add_executable(BLMTriggeringAnalysisClientSample ./samples/BLMTriggeringAnalysisClientSample.cpp)
    target_link_libraries(BLMTriggeringAnalysisClientSample ${MODULE_NAME} Sockets ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "CERNBot2")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CANSocket
                CANOpenDevicesDeprecated
                RobotBase
                LinearStage
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/CERNBot2.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

add_executable(CERNBot2Sample ./samples/CERNBot2Sample.cpp)
target_link_libraries(CERNBot2Sample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "FTSensor")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                Types
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_FTM115)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CANSocket
                    KalmanFilterDeprecated
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/FTM115/FTM115.cpp
    )
endif()

if(DISABLE_FTM115)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_FTM115)
    add_executable(FTM115Sample ./samples/FTM115Sample.cpp)
    target_link_libraries(FTM115Sample FTSensor)
endif()

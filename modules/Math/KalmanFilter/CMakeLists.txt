#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "KalmanFilter")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
    EventLogger
)

set(INCLUDE_DIRS
    ./include
    ${EIGEN3_INCLUDE_DIR}
)

set(SRC_FILES
    ./src/KalmanFilter.cpp
    ./src/ExtendedKalmanFilter.cpp
    ./src/StateSpace/StateSpace.cpp
    ./src/ObservationModel/ObservationModel.cpp
    ./src/ObservationModel/SPSObservationModel.cpp
    ./src/ObservationModelLinearized/ObservationModelLinearized.cpp
    ./src/SystemModel/SystemModel.cpp
    ./src/SystemModel/SPSSystemModel.cpp
    ./src/SystemModelLinearized/SystemModelLinearized.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

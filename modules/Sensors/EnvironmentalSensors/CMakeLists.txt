#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "EnvironmentalSensors")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_EnvironmentalSensorClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/EnvironmentalSensorClient/EnvironmentalSensorClient.cpp
    )
endif()

if(NOT DISABLE_EnvironmentalSensorCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorManager.cpp
                  ./src/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorCommunicationPoint.cpp
                  ./src/EnvironmentalSensorCommunicationPoint/EnvironmentalSensorCommunicationPointFactory.cpp
    )
endif()

if (DISABLE_EnvironmentalSensorClient AND DISABLE_EnvironmentalSensorCommunicationPoint)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

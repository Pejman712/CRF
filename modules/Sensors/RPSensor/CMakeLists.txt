#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "RPSensor")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_AtomtexBDKG24)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    SerialCommunication
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/AtomtexBDKG24/AtomtexBDKG24.cpp
    )
endif()

if(NOT DISABLE_RPSensorClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RPSensorClient/RPSensorClient.cpp
    )
endif()

if(NOT DISABLE_RPSensorCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RPSensorCommunicationPoint/RPSensorManager.cpp
                  ./src/RPSensorCommunicationPoint/RPSensorCommunicationPoint.cpp
                  ./src/RPSensorCommunicationPoint/RPSensorCommunicationPointFactory.cpp
    )
endif()

if (DISABLE_AtomtexBDKG24 AND DISABLE_RPSensorClient AND DISABLE_RPSensorCommunicationPoint)
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

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_RPSensorCommunicationPoint)
    if(NOT DISABLE_AtomtexBDKG24)
        add_executable(AtomtexBDKG24Point ./samples/AtomtexBDKG24Point.cpp)
        target_link_libraries(AtomtexBDKG24Point ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if(ENABLE_UnitTests)
        add_executable(MockRPSensorPoint ./samples/MockRPSensorPoint.cpp)
        target_link_libraries(MockRPSensorPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} gmock)
    endif()

endif()

if(NOT DISABLE_AtomtexBDKG24)
    add_executable(AtomtexBDKG24Sample ./samples/AtomtexBDKG24Sample.cpp)
    target_link_libraries(AtomtexBDKG24Sample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_RPSensorClient)
    add_executable(RPSensorClientSample ./samples/RPSensorClientSample.cpp)
    target_link_libraries(RPSensorClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

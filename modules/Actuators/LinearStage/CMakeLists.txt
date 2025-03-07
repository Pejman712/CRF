#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "LinearStage")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                ErrorHandler
                CommonInterfaces
                ${Boost_LIBRARIES}
                ${nlohmann_json_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIRS}
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES ./src/LinearStageConfiguration.cpp
)

if(NOT DISABLE_CANOpenLinearStage)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CANOpenDevicesDeprecated
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CANOpenLinearStage/CANOpenLinearStage.cpp
    )
endif()

if(NOT DISABLE_EtherCATLineaActuator)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EtherCATDevices
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/EtherCATLinearActuator/EtherCATLinearActuator.cpp
    )
endif()

if(NOT DISABLE_LinearActuatorCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/LinearActuatorCommunicationPoint/LinearActuatorManager.cpp
                  ./src/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPoint.cpp
                  ./src/LinearActuatorCommunicationPoint/LinearActuatorCommunicationPointFactory.cpp
    )
endif()

if (DISABLE_EtherCATLineaActuator AND DISABLE_LinearActuatorCommunicationPoint AND DISABLE_CANOpenLinearStage)
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

find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_EtherCATLineaActuator)
    add_executable(EtherCATLinearActuatorSample ./samples/EtherCATLinearActuator/EtherCATLinearActuatorSample.cpp)
    target_link_libraries(EtherCATLinearActuatorSample ${MODULE_NAME})
endif()

if(NOT DISABLE_LinearActuatorCommunicationPoint)
    add_executable(EtherCATLinearActuatorPoint ./samples/LinearActuatorCommunicationPoint/EtherCATLinearActuatorPoint.cpp)
    target_link_libraries(EtherCATLinearActuatorPoint ${MODULE_NAME}  ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

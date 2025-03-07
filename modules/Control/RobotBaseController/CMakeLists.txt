#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "RobotBaseController")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
                Types
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_RobotBaseControllerClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotBaseControllerClient/RobotBaseControllerClient.cpp
    )
endif()

if(NOT DISABLE_RobotBaseControllerCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    LinearStage
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
                    DeviceManager
                    ComponentAccessControl
                    RobotBase
                    TrajectoryPointGenerator
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPoint.cpp
                  ./src/RobotBaseControllerCommunicationPoint/RobotBaseControllerCommunicationPointFactory.cpp
                  ./src/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.cpp
    )
endif()

if(NOT DISABLE_RobotBaseVelocityController)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    RobotBase
                    TrajectoryPointGenerator
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotBaseVelocityController/RobotBaseVelocityController.cpp
    )
endif()

if (DISABLE_RobotBaseControllerClient AND DISABLE_RobotBaseControllerCommunicationPoint AND DISABLE_RobotBaseVelocityController)
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

if (NOT DISABLE_RobotBaseControllerClient)
    add_executable(RobotBaseControllerClientSample ./samples/RobotBaseControllerClient/RobotBaseControllerClientSample.cpp)
    target_link_libraries(RobotBaseControllerClientSample ${MODULE_NAME})
endif()

if (NOT DISABLE_CERNBot)
    if (NOT DISABLE_RobotBaseVelocityController)
        add_executable(CERNBotVelocityControllerSample ./samples/RobotBaseVelocityController/CERNBotVelocityControllerSample.cpp)
        target_link_libraries(CERNBotVelocityControllerSample ${MODULE_NAME} CERNBot ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
    if (NOT DISABLE_RobotBaseControllerCommunicationPoint)
        add_executable(CERNBotControllerPoint ./samples/RobotBaseControllerCommunicationPoint/CERNBotControllerPoint.cpp)
        target_link_libraries(CERNBotControllerPoint ${MODULE_NAME} CERNBot ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_CERNBot2)
    if (NOT DISABLE_RobotBaseVelocityController)
        add_executable(CERNBot2VelocityControllerSample ./samples/RobotBaseVelocityController/CERNBot2VelocityControllerSample.cpp)
        target_link_libraries(CERNBot2VelocityControllerSample ${MODULE_NAME} CERNBot2 ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
    if (NOT DISABLE_RobotBaseControllerCommunicationPoint)
        add_executable(CERNBot2ControllerPoint ./samples/RobotBaseControllerCommunicationPoint/CERNBot2ControllerPoint.cpp)
        target_link_libraries(CERNBot2ControllerPoint ${MODULE_NAME} CERNBot2 ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_CHARMBot)
    if (NOT DISABLE_RobotBaseControllerCommunicationPoint)
        add_executable(CHARMBotControllerPoint ./samples/RobotBaseControllerCommunicationPoint/CHARMBotControllerPoint.cpp)
        target_link_libraries(CHARMBotControllerPoint ${MODULE_NAME} CHARMBot ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN EN/SMM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "MissionManager")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_RPSurveyLHC)
    set(MODULE_LIBS ${MODULE_LIBS}
                    RobotArmController
                    RPSensor
                    TIM
                    TIMArm
                    TIMRPWagon
                    Types
                    MissionUtility
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RPSurveyLHC/RPSurveyLHC.cpp
                  ./src/RPSurveyLHC/Actions/Actions.cpp
                  ./src/RPSurveyLHC/Actions/BackgroundRPMeasurement.cpp
    )
endif()

if(NOT DISABLE_ScienceGateway)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Robot
                    MotionController
                    TrajectoryGenerator
                    Controller
                    KortexAPI
                    Types
                    MissionUtility
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/ScienceGateway/ScienceGateway.cpp
                  ./src/ScienceGateway/Actions/Actions.cpp
                  ./src/ScienceGateway/Actions/FigureCube.cpp
    )
endif()

if(NOT DISABLE_MissionManagerCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MissionManagerCommunicationPoint/MissionManagerCommunicationPoint.cpp
                  ./src/MissionManagerCommunicationPoint/MissionManagerCommunicationPointFactory.cpp
    )
endif()

if(NOT DISABLE_MissionManagerClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MissionManagerClient/MissionManagerClient.cpp
    )
endif()

if(DISABLE_RPSurveyLHC AND DISABLE_ScienceGateway AND DISABLE_MissionManagerCommunicationPoint AND DISABLE_MissionManagerClient)
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

if(NOT DISABLE_MissionManagerClient)
    add_executable(MissionManagerClientSample ./samples/MissionManagerClient/MissionManagerClientSample.cpp)
    target_link_libraries(MissionManagerClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_RPSurveyLHC AND NOT DISABLE_MissionManagerCommunicationPoint)
    add_executable(RPSurveyLHCPoint ./samples/MissionManagerCommunicationPoint/RPSurveyLHCPoint.cpp)
    target_link_libraries(RPSurveyLHCPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    if (ENABLE_UnitTests)
        add_executable(RPSurveyLHCSample ./samples/RPSurveyLHC/RPSurveyLHCSample.cpp)
        target_link_libraries(RPSurveyLHCSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} gmock)
    endif()
endif()

if (NOT DISABLE_ScienceGateway)
    add_executable(FigureCubeSample ./samples/ScienceGateway/FigureCubeSample.cpp)
    target_link_libraries(FigureCubeSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    add_executable(ScienceGatewaySample ./samples/ScienceGateway/ScienceGatewaySample.cpp)
    target_link_libraries(ScienceGatewaySample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY} gmock GraphPlot)
endif()

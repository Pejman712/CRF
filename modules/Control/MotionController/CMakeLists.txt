#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "MotionController")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
    ErrorHandler
    Types
    EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
    ./src/AbstractMotionController.cpp
)

if(NOT DISABLE_PathFollower)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Robot
                    Controller
                    TrajectoryGenerator
                    InverseKinematics
                    ForwardKinematics
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PathFollower/PathFollower.cpp
    )
endif()

if(NOT DISABLE_Teleoperation)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Robot
                    Controller
                    InputShaper
                    InverseKinematics
                    ForwardKinematics
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Teleoperation/InputShaperJointsController.cpp
                  ./src/Teleoperation/InputShaperTaskController.cpp
                  ./src/Teleoperation/Teleoperation.cpp
    )
endif()

if(NOT DISABLE_MotionControllerCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DataPackets
                    DataPacketSocket
                    Sockets
                    CommunicationPointServer
                    DeviceManager
                    ComponentAccessControl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MotionControllerCommunicationPoint/MotionControllerCommunicationPoint.cpp
                  ./src/MotionControllerCommunicationPoint/MotionControllerCommunicationPointFactory.cpp
                  ./src/MotionControllerCommunicationPoint/MotionControllerManager.cpp
    )
endif()

if(NOT DISABLE_MotionControllerClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DataPackets
                    DataPacketSocket
                    Sockets
                    CommunicationPointServer
                    DeviceManager
                    ComponentAccessControl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MotionControllerClient/MotionControllerClient.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

find_package(Boost COMPONENTS program_options)

if(ENABLE_UnitTests)
    add_executable(MockMotionControllerPoint ./samples/MotionControllerCommunicationPoint/MockMotionControllerPoint.cpp)
    target_link_libraries(MockMotionControllerPoint MotionController gmock ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_PathFollower AND NOT DISABLE_UniversalRobot)
    add_executable(PathFollowerSample ./samples/PathFollower/PathFollowerSample.cpp)
    target_link_libraries(PathFollowerSample MotionController)
endif()

if(NOT DISABLE_PathFollower AND NOT DISABLE_KinovaGen3)
    add_executable(PathFollowerSampleforKinovaGen3 ./samples/PathFollower/PathFollowerSampleforKinovaGen3.cpp)
    target_link_libraries(PathFollowerSampleforKinovaGen3 MotionController)
endif()

if(NOT DISABLE_PathFollower)
    if(NOT DISABLE_KinovaJaco2 AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(KinovaJaco2PathFollowerPoint ./samples/MotionControllerCommunicationPoint/KinovaJaco2PathFollowerPoint.cpp)
        target_link_libraries(KinovaJaco2PathFollowerPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if(NOT DISABLE_UniversalRobot AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(URPathFollowerPoint ./samples/MotionControllerCommunicationPoint/URPathFollowerPoint.cpp)
        target_link_libraries(URPathFollowerPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    add_executable(MIRAPathFollowerSample ./samples/PathFollower/MIRAPathFollowerSample.cpp)
    target_link_libraries(MIRAPathFollowerSample MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_Teleoperation)
    if(NOT DISABLE_UniversalRobot)
        add_executable(TeleoperationSample ./samples/Teleoperation/TeleoperationSample.cpp)
        target_link_libraries(TeleoperationSample MotionController)
    endif()

    if(NOT DISABLE_UniversalRobot AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(URTeleoperationPoint ./samples/MotionControllerCommunicationPoint/URTeleoperationPoint.cpp)
        target_link_libraries(URTeleoperationPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if(NOT DISABLE_KinovaJaco2 AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(KinovaJaco2TeleoperationPoint ./samples/MotionControllerCommunicationPoint/KinovaJaco2TeleoperationPoint.cpp)
        target_link_libraries(KinovaJaco2TeleoperationPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if (NOT DISABLE_CiA402Robot AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(PRBT6TeleoperationPoint ./samples/MotionControllerCommunicationPoint/PRBT6TeleoperationPoint.cpp)
        target_link_libraries(PRBT6TeleoperationPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if (NOT DISABLE_CiA402Robot AND NOT DISABLE_UniversalRobot AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(FCCTeleoperationPoint ./samples/MotionControllerCommunicationPoint/FCCTeleoperationPoint.cpp)
        target_link_libraries(FCCTeleoperationPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()

    if (NOT DISABLE_CiA402Robot AND NOT DISABLE_KinovaJaco2 AND NOT DISABLE_MotionControllerCommunicationPoint)
        add_executable(MIRATeleoperationPoint ./samples/MotionControllerCommunicationPoint/MIRATeleoperationPoint.cpp)
        target_link_libraries(MIRATeleoperationPoint MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_MotionControllerClient)
    add_executable(MotionControllerClientSample ./samples/MotionControllerClient/MotionControllerClientSample.cpp)
    target_link_libraries(MotionControllerClientSample MotionController ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

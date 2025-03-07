#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "RobotArmController")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
                Types
                Gripper
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_RobotArmControllerClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotArmControllerClient/RobotArmControllerClient.cpp
    )
endif()

if(NOT DISABLE_RobotArmControllerCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    Gripper
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
                    DeviceManager
                    ComponentAccessControl
                    RobotArm
                    TrajectoryGeneratorDeprecated
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPoint.cpp
                  ./src/RobotArmControllerCommunicationPoint/RobotArmControllerCommunicationPointFactory.cpp
                  ./src/RobotArmControllerCommunicationPoint/RobotArmControllerManager.cpp
    )
endif()

if(NOT DISABLE_RobotArmVelocityController)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    RobotArm
                    RobotArmKinematics
                    TrajectoryGeneratorDeprecated
                    ClosedLoopController
                    GraphPlot
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RobotArmVelocityController/RobotArmVelocityController.cpp
    )
endif()

if (DISABLE_RobotArmControllerClient AND DISABLE_RobotArmControllerCommunicationPoint AND DISABLE_RobotArmVelocityController)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if (NOT DISABLE_RobotArmControllerClient)
    add_executable(RobotArmControllerClientSample ./samples/RobotArmControllerClient/RobotArmControllerClientSample.cpp)
    target_link_libraries(RobotArmControllerClientSample ${MODULE_NAME})
endif()

if (NOT DISABLE_KinovaArm)
    add_executable(KinovaArmVelocityControllerSample ./samples/RobotArmVelocityController/KinovaArmVelocityControllerSample.cpp)
    target_link_libraries(KinovaArmVelocityControllerSample ${MODULE_NAME} KinovaArm)

    if (NOT DISABLE_RobotArmControllerCommunicationPoint)
        add_executable(KinovaArmControllerPoint ./samples/RobotArmControllerCommunicationPoint/KinovaArmControllerPoint.cpp)
        target_link_libraries(KinovaArmControllerPoint ${MODULE_NAME} KinovaArm ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_SchunkArm)
    add_executable(SchunkArmVelocityControllerSample ./samples/RobotArmVelocityController/SchunkArmVelocityControllerSample.cpp)
    target_link_libraries(SchunkArmVelocityControllerSample ${MODULE_NAME} SchunkArm)

    if (NOT DISABLE_RobotArmControllerCommunicationPoint)
        add_executable(SchunkArmControllerPoint ./samples/RobotArmControllerCommunicationPoint/SchunkArmControllerPoint.cpp)
        target_link_libraries(SchunkArmControllerPoint ${MODULE_NAME} SchunkArm ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_SchunkPowerCube)
    if (NOT DISABLE_RobotArmControllerCommunicationPoint)
        add_executable(SchunkPowerCubeControllerPoint ./samples/RobotArmControllerCommunicationPoint/SchunkPowerCubeControllerPoint.cpp)
        target_link_libraries(SchunkPowerCubeControllerPoint ${MODULE_NAME} SchunkPowerCube RobotiqGripper ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_EtherCATRobotArm AND NOT DISABLE_TIMRobotArmWagonMotors)
    if (NOT DISABLE_RobotArmControllerCommunicationPoint)
        add_executable(EtherCATRobotArmControllerPoint ./samples/RobotArmControllerCommunicationPoint/EtherCATRobotArmControllerPoint.cpp)
        target_link_libraries(EtherCATRobotArmControllerPoint ${MODULE_NAME} EtherCATRobotArm ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if (NOT DISABLE_TIMArm)
    add_executable(TIMArmVelocityControllerSample ./samples/RobotArmVelocityController/TIMArmVelocityControllerSample.cpp)
    target_link_libraries(TIMArmVelocityControllerSample ${MODULE_NAME} TIMArm)

    if (NOT DISABLE_RobotArmControllerCommunicationPoint)
        add_executable(TIMArmControllerPoint ./samples/RobotArmControllerCommunicationPoint/TIMArmControllerPoint.cpp)
        target_link_libraries(TIMArmControllerPoint ${MODULE_NAME} TIMArm ${Boost_PROGRAM_OPTIONS_LIBRARY})

        check_module_dependencies(MechanicalStabilizer)
        check_module_dependencies(Shielding)
        if(NOT DISABLE_MechanicalStabilizer AND NOT DISABLE_Shielding)
            add_executable(TIMArmWagonActuatorsCommunicationPoint ./samples/MultipleCommunicationPoints/TIMArmWagonActuatorsCommunicationPoint.cpp)
            target_link_libraries(TIMArmWagonActuatorsCommunicationPoint ${MODULE_NAME} TIMArm MechanicalStabilizer Shielding ${Boost_PROGRAM_OPTIONS_LIBRARY})
        endif()
    endif()
endif()

if (BUILD_PilzArm)
    if (BUILD_Sockets AND BUILD_ComponentAccessControl)
        add_executable(PilzArmControllerPoint ./samples/RobotArmControllerCommunicationPoint/PilzArmControllerPoint.cpp)
        target_link_libraries(PilzArmControllerPoint ${MODULE_NAME} PilzArm ${Boost_LIBRARIES})
    endif()
endif()

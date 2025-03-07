#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "Shielding")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_ShieldingClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/ShieldingClient/ShieldingClient.cpp
    )
endif()

if(NOT DISABLE_ShieldingCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
                    DeviceManager
                    ComponentAccessControl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/ShieldingCommunicationPoint/ShieldingCommunicationPoint.cpp
                  ./src/ShieldingCommunicationPoint/ShieldingCommunicationPointFactory.cpp
                  ./src/ShieldingCommunicationPoint/ShieldingManager.cpp
    )
endif()

if(NOT DISABLE_RadioactiveSourceShielding)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    EtherCATDevices
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RadioactiveSourceShielding/RadioactiveSourceShielding.cpp
    )
endif()

if (DISABLE_ShieldingClient AND DISABLE_ShieldingCommunicationPoint AND DISABLE_RadioactiveSourceShielding)
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

if (NOT DISABLE_ShieldingClient)
    add_executable(ShieldingClientSample ./samples/ShieldingClientSample.cpp)
    target_link_libraries(ShieldingClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if (NOT DISABLE_RadioactiveSourceShielding AND NOT DISABLE_TIMRobotArmWagonMotors)
    add_executable(RadioactiveSourceShieldingSample ./samples/RadioactiveSourceShieldingSample.cpp)
    target_link_libraries(RadioactiveSourceShieldingSample ${MODULE_NAME})

    if (NOT DISABLE_ShieldingCommunicationPoint)
        add_executable(RadioactiveSourceShieldingPoint ./samples/RadioactiveSourceShieldingPoint.cpp)
        target_link_libraries(RadioactiveSourceShieldingPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

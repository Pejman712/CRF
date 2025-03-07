#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "MechanicalStabilizer")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommonInterfaces
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_MechanicalStabilizerClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    DataPackets
                    DataPacketSocket
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MechanicalStabilizerClient/MechanicalStabilizerClient.cpp
    )
endif()

if(NOT DISABLE_MechanicalStabilizerCommunicationPoint)
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
                  ./src/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPoint.cpp
                  ./src/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerCommunicationPointFactory.cpp
                  ./src/MechanicalStabilizerCommunicationPoint/MechanicalStabilizerManager.cpp
    )
endif()

if(NOT DISABLE_TIMStabilizer)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    EtherCATDevices
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMStabilizer/TIMStabilizer.cpp
    )
endif()

if (DISABLE_MechanicalStabilizerClient AND DISABLE_MechanicalStabilizerCommunicationPoint AND DISABLE_TIMStabilizer)
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

if (NOT DISABLE_MechanicalStabilizerClient)
    add_executable(MechanicalStabilizerClientSample ./samples/MechanicalStabilizerClient/MechanicalStabilizerClientSample.cpp)
    target_link_libraries(MechanicalStabilizerClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if (NOT DISABLE_TIMStabilizer AND NOT DISABLE_TIMRobotArmWagonMotors)
    add_executable(TIMStabilizerSample ./samples/TIMStabilizer/TIMStabilizerSample.cpp)
    target_link_libraries(TIMStabilizerSample ${MODULE_NAME})

    if (NOT DISABLE_MechanicalStabilizerCommunicationPoint)
        add_executable(TIMStabilizerPoint ./samples/MechanicalStabilizerCommunicationPoint/TIMStabilizerPoint.cpp)
        target_link_libraries(TIMStabilizerPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

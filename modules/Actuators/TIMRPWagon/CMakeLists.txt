#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "TIMRPWagon")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(MODULE_LIBS EventLogger
                CommonInterfaces
)
set(SRC_FILES ./src/TIMRPWagonConfiguration.cpp
)

if(NOT DISABLE_TIMS300RPWagon)
    set(MODULE_LIBS ${MODULE_LIBS}
                    SiemensPLC
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMRPWagonCommands.cpp
                  ./src/TIMRPWagonStatus.cpp
                  ./src/TIMS300RPWagon/TIMS300RPWagon.cpp
    )
endif()

if(NOT DISABLE_TIMRPWagonCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMRPWagonCommunicationPoint/TIMRPWagonManager.cpp
                  ./src/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPoint.cpp
                  ./src/TIMRPWagonCommunicationPoint/TIMRPWagonCommunicationPointFactory.cpp
    )
endif()

if(NOT DISABLE_TIMRPWagonClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
                    DeviceManager
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMRPWagonClient/TIMRPWagonClient.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

if (ENABLE_UnitTests)
    # add_subdirectory(tests)
endif()

find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_TIMRPWagonClient)
    add_executable(TIMRPWagonClientSample ./samples/TIMRPWagonClientSample.cpp)
    target_link_libraries(TIMRPWagonClientSample ${MODULE_NAME})
endif()

if(NOT DISABLE_TIMS300RPWagon)
    if(NOT DISABLE_TIMRPWagonCommunicationPoint)
        add_executable(TIMS300RPWagonPoint ./samples/TIMS300RPWagonPoint.cpp)
        target_link_libraries(TIMS300RPWagonPoint ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

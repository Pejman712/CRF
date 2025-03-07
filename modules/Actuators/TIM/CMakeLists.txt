#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN EN/SMM/MRO                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "TIM")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(MODULE_LIBS EventLogger
                CommonInterfaces
)
set(SRC_FILES ./src/TIMConfiguration.cpp
              ./src/LHCObstacle.cpp
              ./src/TIMAlarms.cpp
              ./src/TIMCommands.cpp
              ./src/TIMSettings.cpp
              ./src/TIMStatus.cpp
)

if(NOT DISABLE_TIMS300)
    set(MODULE_LIBS ${MODULE_LIBS}
                    SiemensPLC
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMS300/TIMS300.cpp
    )
endif()

if(NOT DISABLE_TIMCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
                    DeviceManager
                    CommunicationUtility
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMCommunicationPoint/TIMManager.cpp
                  ./src/TIMCommunicationPoint/TIMCommunicationPoint.cpp
                  ./src/TIMCommunicationPoint/TIMCommunicationPointFactory.cpp
    )
endif()

if(NOT DISABLE_TIMClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    CommunicationPointServer
                    DataPackets
                    DataPacketSocket
                    DeviceManager
                    CommunicationUtility
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMClient/TIMClient.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

if (ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_TIMClient)
    add_executable(TIMClientSample ./samples/TIMClientSample.cpp)
    target_link_libraries(TIMClientSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_TIMS300)
    add_executable(TIMS300Sample ./samples/TIMS300Sample.cpp)
    target_link_libraries(TIMS300Sample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    if(NOT DISABLE_TIMCommunicationPoint)
        add_executable(TIMS300Point ./samples/TIMS300Point.cpp)
        target_link_libraries(TIMS300Point ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

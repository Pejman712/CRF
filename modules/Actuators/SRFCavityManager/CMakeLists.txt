#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Adrien Luthi CERN EN/SMM/MRO 2023                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "SRFCavityManager")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
)
set(MODULE_LIBS EventLogger
                CommonInterfaces
                ErrorHandler
)
set(SRC_FILES
)

if(NOT DISABLE_EtherCATSRFCavityManager)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EtherCATDevices
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${nlohmann_json_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/EtherCATSRFCavityManager/EtherCATSRFCavityManager.cpp
    )
endif()


if (DISABLE_EtherCATSRFCavityManager)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
endif()

if (ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(testCavityMotorInVel ./samples/testCavityMotorInVel.cpp)
target_link_libraries(testCavityMotorInVel ${MODULE_NAME})

add_executable(testCavityMotorInPosAbsolute ./samples/testCavityMotorInPosAbsolute.cpp)
target_link_libraries(testCavityMotorInPosAbsolute ${MODULE_NAME})

add_executable(testCavityMotorInPosRelative ./samples/testCavityMotorInPosRelative.cpp)
target_link_libraries(testCavityMotorInPosRelative ${MODULE_NAME})

add_executable(testCavityMotorSwitchingMOp ./samples/testCavityMotorSwitchingMOp.cpp)
target_link_libraries(testCavityMotorSwitchingMOp ${MODULE_NAME})

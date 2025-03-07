#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "EtherCATDevices")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                CommonInterfaces
                ${SOEM_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${SOEM_INCLUDE_DIRS}
)
set(SRC_FILES ./src/SoemApi.cpp
              ./src/EtherCATManager.cpp
              ./src/EtherCATMotor.cpp
)

if(NOT DISABLE_TIMRobotArmWagonMotors)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/TIMRobotArmWagonMotors/TIMRobotArmWagonMotors.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(ProfilePositionMode ./samples/ProfilePositionMode.cpp)
target_link_libraries(ProfilePositionMode ${MODULE_NAME})

add_executable(ProfileVelocityMode ./samples/ProfileVelocityMode.cpp)
target_link_libraries(ProfileVelocityMode ${MODULE_NAME})

add_executable(ProfileTorqueMode ./samples/ProfileTorqueMode.cpp)
target_link_libraries(ProfileTorqueMode ${MODULE_NAME})

if(NOT DISABLE_TIMRobotArmWagonMotors)
    add_executable(TIMRobotArmWagonMotorsSample ./samples/TIMRobotArmWagonMotorsSample.cpp)
    target_link_libraries(TIMRobotArmWagonMotorsSample ${MODULE_NAME})
endif()

# © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
# Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
#
# Author: Victor Mtsimbe Norrild CERN EN/SMM/MRO
#
# ==================================================================================================
#

set(MODULE_NAME "DataLogger")

find_package(CURLPP REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LZ4 1.9.2)

set(MODULE_LIBS EventLogger
                Types
                CANSocket
                CommonInterfaces
                Gripper
                IPC
                RobotArm
                SchunkArm
                RobotArmControllersDeprecated
                VideoCodecs
                ${CURLPP_LIBRARIES}
                ${LZ4_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${CURLPP_INCLUDE_DIRS}
                 ${LZ4_INCLUDE_DIRS}
)
set(SRC_FILES ./src/InfluxLogger.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(NumericLoggerExample ./samples/NumericLoggerExample.cpp)
target_link_libraries(NumericLoggerExample ${MODULE_NAME})

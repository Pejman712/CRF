#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO                                                                            ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "SchunkPowerCube")

find_package(nlohmann_json REQUIRED)

set(MODULE_LIBS EventLogger
                CANSocket
                RobotArm
)

set(INCLUDE_DIRS ./include
)

set(SRC_FILES ./src/SchunkPowerCubeDevice.cpp
              ./src/SchunkPowerCubeConfiguration.cpp
              ./src/SchunkPowerCube.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(SchunkPowerCubeExample ./samples/SchunkPowerCubeExample.cpp)
target_link_libraries(SchunkPowerCubeExample SchunkPowerCube)

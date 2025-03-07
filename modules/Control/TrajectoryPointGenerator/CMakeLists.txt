#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "TrajectoryPointGenerator")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                Types
                ReflexxesMotionLibrary
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/ReflexxesTrajectoryPointGenerator.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

add_executable(VelocityTrajectoryGeneratorExample ./samples/VelocityTrajectoryGeneratorExample.cpp)
target_link_libraries(VelocityTrajectoryGeneratorExample ${MODULE_NAME})

add_executable(PositionTrajectoryGeneratorExample ./samples/PositionTrajectoryGeneratorExample.cpp)
target_link_libraries(PositionTrajectoryGeneratorExample ${MODULE_NAME})

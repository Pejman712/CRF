#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2021                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "DistanceMeasures")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                Types
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)
set(SRC_FILES ./src/TaskPose.cpp
              ./src/Rotation.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

add_executable(TaskPoseDistanceMeasureSample ./samples/TaskPoseDistanceMeasureSample.cpp)
target_link_libraries(TaskPoseDistanceMeasureSample DistanceMeasures)

add_executable(RotationDistanceMeasureSample ./samples/RotationDistanceMeasureSample.cpp)
target_link_libraries(RotationDistanceMeasureSample DistanceMeasures)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

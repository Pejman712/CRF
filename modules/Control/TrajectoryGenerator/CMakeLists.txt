#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "TrajectoryGenerator")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Types
                EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_CubicJointsTrajectory)
    set(MODULE_LIBS ${MODULE_LIBS}
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CubicJointsTrajectory/CubicJointsTrajectory.cpp
    )
endif()

if(NOT DISABLE_CubicTaskTrajectory)
    set(MODULE_LIBS ${MODULE_LIBS}
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CubicTaskTrajectory/CubicTaskTrajectory.cpp
    )
endif()

if(NOT DISABLE_PointToPointJointsTrajectory)
    set(MODULE_LIBS ${MODULE_LIBS}
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PointToPointJointsTrajectory/PointToPointJointsTrajectory.cpp
    )
endif()

if(NOT DISABLE_PreplannedTaskTrajectory)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PreplannedTaskTrajectory/PreplannedTaskTrajectory.cpp
    )
endif()

if (DISABLE_CubicJointsTrajectory AND DISABLE_PointToPointJointsTrajectory AND DISABLE_PreplannedTaskTrajectory)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_OnlineCubicJointsTrajectory)
    add_executable(CubicJointsTrajectorySample ./samples/CubicJointsTrajectorySample.cpp)
    target_link_libraries(CubicJointsTrajectorySample TrajectoryGenerator)
endif()

if(NOT DISABLE_CubicTaskTrajectory)
    add_executable(CubicTaskTrajectorySample ./samples/CubicTaskTrajectorySample.cpp)
    target_link_libraries(CubicTaskTrajectorySample TrajectoryGenerator)
endif()

if(NOT DISABLE_PointToPointJointsTrajectory)
    add_executable(PointToPointJointsTrajectorySample ./samples/PointToPointJointsTrajectorySample.cpp)
    target_link_libraries(PointToPointJointsTrajectorySample TrajectoryGenerator)
endif()

if(NOT DISABLE_PreplannedTaskTrajectory)
    add_executable(PreplannedTaskTrajectorySample ./samples/PreplannedTaskTrajectorySample.cpp)
    target_link_libraries(PreplannedTaskTrajectorySample TrajectoryGenerator)
endif()

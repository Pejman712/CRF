#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "InverseKinematics")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Types
                Jacobian
                KinematicChain
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)
set(SRC_FILES ./src/ResultsIK.cpp
)

if(NOT DISABLE_CollisionAvoidanceSphere)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CollisionAvoidanceSphere/CollisionAvoidanceSphere.cpp
    )
endif()

if(NOT DISABLE_DesiredJointPositions)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DesiredJointPositions/DesiredJointPositions.cpp
    )
endif()

if(NOT DISABLE_JointLimits)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/JointLimits/JointLimits.cpp
    )
endif()

if(NOT DISABLE_OptCLIK)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DistanceMeasures
                    EventLogger
                    ForwardKinematics
                    Robot
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/OptCLIK/OptCLIK.cpp
    )
endif()

if(NOT DISABLE_OptOLIK)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    Robot
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/OptOLIK/OptOLIK.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_CollisionAvoidanceSphere)
    add_executable(CollisionAvoidanceSphereSample ./samples/CollisionAvoidanceSphere/CollisionAvoidanceSphereSample.cpp)
    target_link_libraries(CollisionAvoidanceSphereSample ${MODULE_NAME})
endif()

if(NOT DISABLE_DesiredJointPositions)
    add_executable(DesiredJointPositionsSample ./samples/DesiredJointPositions/DesiredJointPositionsSample.cpp)
    target_link_libraries(DesiredJointPositionsSample ${MODULE_NAME})
endif()

if(NOT DISABLE_JointLimits)
    add_executable(JointLimitsSample ./samples/JointLimits/JointLimitsSample.cpp)
    target_link_libraries(JointLimitsSample ${MODULE_NAME})
endif()

if(NOT DISABLE_OptCLIK)
    add_executable(BasicOptCLIKFunctionsSample ./samples/OptCLIK/BasicOptCLIKFunctionsSample.cpp)
    target_link_libraries(BasicOptCLIKFunctionsSample ${MODULE_NAME})

    add_executable(BasicOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample ./samples/OptCLIK/BasicOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample.cpp)
    target_link_libraries(BasicOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample ${MODULE_NAME} GeometricMethods)

    add_executable(BasicOptCLIKTrajectoryReduced2DTaskSpace2DOFARISArmRobotSample ./samples/OptCLIK/BasicOptCLIKTrajectoryReduced2DTaskSpace2DOFARISArmRobotSample.cpp)
    target_link_libraries(BasicOptCLIKTrajectoryReduced2DTaskSpace2DOFARISArmRobotSample ${MODULE_NAME} GeometricMethods)

    add_executable(ExpertOptCLIKFunctionsSample ./samples/OptCLIK/ExpertOptCLIKFunctionsSample.cpp)
    target_link_libraries(ExpertOptCLIKFunctionsSample ${MODULE_NAME})
endif()

if(NOT DISABLE_OptCLIK AND NOT DISABLE_CollisionAvoidanceSphere AND NOT DISABLE_DesiredJointPositions AND NOT DISABLE_JointLimits)
    add_executable(BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobotSample ./samples/OptCLIKAndObjectiveFunctions/BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobotSample.cpp)
    target_link_libraries(BasicOptCLIKTrajectoryReduced2DTaskSpace3DOFRobotSample ${MODULE_NAME} GeometricMethods)

    add_executable(ExpertOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample ./samples/OptCLIKAndObjectiveFunctions/ExpertOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample.cpp)
    target_link_libraries(ExpertOptCLIKTrajectory6DTaskSpace6DOFUR10eRobotSample ${MODULE_NAME} GeometricMethods)

    add_executable(ExpertOptCLIKTrajectoryReduced3DTaskSpace6DOFUR10eRedundantRobotSample ./samples/OptCLIKAndObjectiveFunctions/ExpertOptCLIKTrajectoryReduced3DTaskSpace6DOFUR10eRedundantRobotSample.cpp)
    target_link_libraries(ExpertOptCLIKTrajectoryReduced3DTaskSpace6DOFUR10eRedundantRobotSample ${MODULE_NAME} GeometricMethods)
endif()

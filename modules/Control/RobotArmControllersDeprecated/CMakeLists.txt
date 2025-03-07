set (MODULE_NAME "RobotArmControllersDeprecated")

set (MODULE_LIBS EventLogger
                 RobotArm
                 RobotArmKinematics
                 Types
                 TrajectoryGenerator
                 ClosedLoopController
)

set(SRC_FILES ./src/RobotArmSlaveController.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC include)
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

if (BUILD_KinovaArm AND BUILD_SchunkArm)
    add_executable(SlaveControllerExample ./samples/SlaveControllerExample.cpp)
    target_link_libraries(SlaveControllerExample ${MODULE_NAME} KinovaArm SchunkArm)
endif()

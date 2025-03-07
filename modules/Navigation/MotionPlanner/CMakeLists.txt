set (MODULE_NAME "MotionPlanner")

set(MODULE_LIBS EventLogger
                Types
                PathPlanner
                CollisionDetector
                RobotBase
                RobotBaseController
                TrajectoryGenerator
                Laser
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/MotionPlannerLaser.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

find_package(fcl QUIET)
if (fcl_FOUND AND BUILD_CHARMBot AND BUILD_CANSocket)
    add_executable(MotionPlannerCHARMBotExample ./samples/MotionPlannerCHARMBotExample.cpp)
    target_link_libraries(MotionPlannerCHARMBotExample ${MODULE_NAME} CHARMBot CANSocket)
endif()
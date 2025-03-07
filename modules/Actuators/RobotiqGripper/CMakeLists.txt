set(MODULE_NAME "RobotiqGripper")
set(INCLUDE_DIRS ./include)
set(MODULE_LIBS
    Gripper
    EventLogger
    modbus
)
set(SRC ./src/RobotiqGripper.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

set(MODULE_NAME "RobotiqGripper140")
set(INCLUDE_DIRS ./include)
set(MODULE_LIBS
    modbus
    Gripper
)
set(SRC ./src/RobotiqGripper140.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

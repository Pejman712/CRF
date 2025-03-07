set(MODULE_NAME "YoubotBase")


include(./YouBot.cmake)
set(INCLUDE_DIRS ./include)
set(MODULE_LIBS
    RobotBase
    ${Boost_LIBRARIES}
    ${YOUBOT_LIBRARIES}
)
set(SRC ./src/YoubotBase.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
SET_TARGET_PROPERTIES(${MODULE_NAME} PROPERTIES COMPILE_FLAGS "-fPIC")
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(youbot ./samples/youbot.cpp)
target_link_libraries(youbot YoubotBase)

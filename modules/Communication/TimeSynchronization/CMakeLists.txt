set(MODULE_NAME "TimeSynchronization")

set(INCLUDE_DIRS ./include)
set(MODULE_LIBS
    IPC
    CommUtility
    CommonInterfaces
)
set(SRC ./src/TimeSynchronizationServer.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(TimeSyncServer ./samples/TimeSyncServer.cpp)
target_link_libraries(TimeSyncServer ${MODULE_NAME})

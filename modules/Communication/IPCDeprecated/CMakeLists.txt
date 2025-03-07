set(MODULE_NAME "IPC")
set(MODULE_LIBS rt CommUtility EventLogger)

set(INCLUDE_DIRS ./include)
set(MODULE_LIBS
    rt
    CommUtility
    NetworkServer
    EventLogger
)
set(SRC
    ./src/FIFO.cpp
    ./src/MMAP.cpp
    ./src/NetworkIPC.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})


add_executable(NetworkIPCExample ./samples/NetworkIPCExample.cpp)
target_link_libraries(NetworkIPCExample ${MODULE_NAME})
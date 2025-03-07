set(MODULE_NAME "LeakDetector")
set(MODULE_LIBS IPC EventLogger SerialCommunication)

add_library(${MODULE_NAME} SHARED src/PhoenixL300i.cpp src/PhoenixTelegram.cpp)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

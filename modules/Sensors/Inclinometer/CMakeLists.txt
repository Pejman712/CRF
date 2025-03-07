set(MODULE_NAME "Inclinometer")
set(MODULE_LIBS EventLogger SerialCommunication)

add_library(${MODULE_NAME} SHARED src/Zerotronic.cpp)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

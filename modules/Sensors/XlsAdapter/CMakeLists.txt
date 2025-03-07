set(MODULE_NAME "XlsAdapter")
set(MODULE_LIBS
    EventLogger
    NetworkClient
    CommUtility
    CommonInterfaces
)

add_library(${MODULE_NAME} SHARED src/XlsAdapter.cpp src/XlsMessage.cpp)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

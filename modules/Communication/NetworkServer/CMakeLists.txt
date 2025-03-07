set (MODULE_NAME "NetworkServer")
set (MODULE_LIBS EventLogger CommUtility)

add_library(${MODULE_NAME} SHARED
    src/TcpServer.cpp
    src/UdpServer.cpp
)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

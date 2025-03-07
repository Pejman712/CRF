set (MODULE_NAME "NetworkClient")
set (MODULE_LIBS EventLogger CommUtility)

add_library(${MODULE_NAME} SHARED
    src/FetchWriteClient.cpp
    src/TcpClient.cpp
    src/XlsTcpClient.cpp
)

target_include_directories(${MODULE_NAME} PUBLIC include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

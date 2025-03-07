set (MODULE_NAME "SqlAdapter")

find_package(ODBC REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(MYSQL REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(nlohmann_json REQUIRED)

set (MODULE_LIBS
    EventLogger
    ${ODBC_LIBRARIES}
    ${MYSQL_LIBRARIES})

add_library(${MODULE_NAME} SHARED
    src/MicrosoftSqlServerAdapter.cpp
    src/MySqlAdapter.cpp
    src/OdbcInterface.cpp
    src/SqlResult.cpp
)

target_include_directories(${MODULE_NAME} PUBLIC include ${ODBC_INCLUDE_DIRS})

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

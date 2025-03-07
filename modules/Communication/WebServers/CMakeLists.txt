set (MODULE_NAME "WebServers")

find_package(RESTBED REQUIRED)

set (MODULE_LIBS EventLogger
                 TIM
                 Cameras
                 ${RESTBED_LIBRARIES}
)

set(INCLUDE_DIRS ./include
                  ${RESTBED_INCLUDE_DIRS}
)

set(SRC_FILES ./src/TIMWebServer.cpp
              ./src/TIMWebPoint.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

add_executable(timwebserver ./samples/timwebserver)
target_link_libraries(timwebserver ${MODULE_NAME})
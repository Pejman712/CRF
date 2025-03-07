set(MODULE_NAME "TeltonikaRUT")
find_package(nlohmann_json REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${nlohmann_json_INCLUDE_DIRS}
)
set(MODULE_LIBS
    curl
    CommonInterfaces
    CommUtility
    IPC
)
set(SRC ./src/TeltonikaRUT.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(4ginfograbber ./samples/4ginfograbber.cpp)
target_link_libraries(4ginfograbber TeltonikaRUT IPC)

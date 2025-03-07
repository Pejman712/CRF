set(MODULE_NAME "Tools")
find_package(nlohmann_json REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${nlohmann_json_INCLUDE_DIRS}
)
set(MODULE_LIBS
    ComponentAccessControl
    CanOpenDevices
    CommonInterfaces
    CommUtility
    IPC
    EtherCATDevices
)

set(SRC
    ./src/ToolCommunicationPoint.cpp
    ./src/CuttingScissors.cpp
    ./src/Screwdriver.cpp)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

find_package(Boost 1.56 COMPONENTS program_options)

add_executable(CuttingScissorsPoint ./samples/CuttingScissorsPoint.cpp)
target_link_libraries(CuttingScissorsPoint ${MODULE_NAME} ${Boost_LIBRARIES})

add_executable(ScrewdriverPoint ./samples/ScrewdriverPoint.cpp)
target_link_libraries(ScrewdriverPoint ${MODULE_NAME} ${Boost_LIBRARIES})


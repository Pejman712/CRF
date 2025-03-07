set(MODULE_NAME "WallDetector")

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(nlohmann_json REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${nlohmann_json_INCLUDE_DIRS}
)

set(MODULE_LIBS
    EventLogger
    ${PCL_LIBRARIES}
    RobotBase
    Types
)

set(WallDetector_SRC ./src/WallDetector.cpp)

add_library(${MODULE_NAME} SHARED ${WallDetector_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

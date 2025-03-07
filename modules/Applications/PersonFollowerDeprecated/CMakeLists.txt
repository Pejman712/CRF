set(MODULE_NAME "PersonFollowerDeprecated")

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(PCL 1.8 REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${PCL_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)

set(MODULE_LIBS
    IPC
    EventLogger
    ObjectDetection
    RobotBaseControllers
    CommonInterfaces
    Types
    Cameras
    LaserCommunicationPoint
    WallDetector
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
)

set(PersonFollower_SRC
    ./src/PersonDetector.cpp
	./src/PersonTracker.cpp
    ./src/PersonFollower.cpp
)

add_library(${MODULE_NAME} SHARED ${PersonFollower_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(PersonFollowerSample ./samples/PersonFollowerSample.cpp)
target_link_libraries(PersonFollowerSample PersonFollower CHARMBot ZMQSocket)

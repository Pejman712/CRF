set(MODULE_NAME "DepthEstimation")
find_package(OpenCV 3.4.0 REQUIRED)
find_package(Eigen3 REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
set(MODULE_LIBS
    ${OpenCV_LIBS}
    EventLogger
    CommUtility
    RobotArm
    SchunkArm
	IPC
)

set(DepthEstimation_SRC
    ./src/Homography.cpp
    ./src/InsulateFrameWorks.cpp
    ./src/NormalDistribution.cpp
    ./src/ScrewDetection.cpp
    ./src/SelectArea.cpp
    ./src/TrackingArea.cpp
)

add_library(${MODULE_NAME} SHARED ${DepthEstimation_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(screw_detection ./samples/screw_detection.cpp)
target_link_libraries(screw_detection DepthEstimation)

find_package(orocos_kdl REQUIRED)
add_executable(depth_estimation ./samples/depth_estimation.cpp)
target_include_directories(depth_estimation SYSTEM PUBLIC ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(depth_estimation DepthEstimation IPC ${orocos_kdl_LIBRARIES})

#it uses an environmental variable to interact with the GUI
find_package(orocos_kdl REQUIRED)
add_executable(selectArea ./samples/selectArea.cpp)
target_link_libraries(selectArea DepthEstimation IPC ${orocos_kdl_LIBRARIES})

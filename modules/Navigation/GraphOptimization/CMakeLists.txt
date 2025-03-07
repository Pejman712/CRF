set(MODULE_NAME "GraphOptimization")

set(BUILD_SAMPLES "ON")

find_package(nlohmann_json REQUIRED)
find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost 1.9 COMPONENTS program_options REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(PythonLibs REQUIRED)

set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIRS}
                 ${PYTHON_INCLUDE_DIRS}
                 /usr/include/suitesparse
                 ${EIGEN3_INCLUDE_DIR}
                 ${nlohmann_json_INCLUDE_DIRS}
                 ${G2O_INCLUDE_DIRS}
                 ${SUITESPARSE_INCLUDE_DIRS}
                 ${PCL_INCLUDE_DIRS}
)

# set is for classes programs
set(MODULE_LIBS EventLogger
                #Mapper3d
                ${Boost_LIBRARIES}
                ${PYTHON_LIBRARIES}
                ${G2O_CORE_LIBRARY}
                ${G2O_STUFF_LIBRARY}
                ${G2O_SOLVER_CHOLMOD}
                ${G2O_SOLVER_CSPARSE}
                ${G2O_SOLVER_DENSE}
                ${G2O_SOLVER_PCG}
                ${G2O_SOLVER_EIGEN}
                ${G2O_TYPES_SBA}
                ${G2O_TYPES_SLAM3D}
                ${SUITESPARSE_LIBRARIES}
)

#LINK_DIRECTORIES(/usr/include/suitesparse)

set(SRC_FILES 
        #./src/GraphOptimization.cpp
        ./src/GraphOptimizationConfiguration.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Werror -Wall -Wextra -Wpedantic)


add_executable(mira3_graph_slam ./samples/mira3_graph_slam.cpp)
target_link_libraries(mira3_graph_slam
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    IMU
) 



add_executable(Unilidar_graph_SLAM ./samples/Unilidar_graph_SLAM.cpp)
target_link_libraries(Unilidar_graph_SLAM
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 

add_executable(Edge_graph_slam ./samples/Edge_graph_slam.cpp)
target_link_libraries(Edge_graph_slam
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    IMU
) 

add_executable(graph_test ./samples/graph_test.cpp)
target_link_libraries(graph_test
    ${MODULE_NAME}
    ${Boost_LIBRARIES}
    g2o_core
    g2o_stuff
    g2o_types_slam2d
)

add_executable(graph_test_offline ./samples/graph_test_offline.cpp)
target_link_libraries(graph_test_offline
    Laser
    IMU
    ${MODULE_NAME}
    ${Boost_LIBRARIES}
)

add_executable(Unilidar_graph_SLAM_IE ./samples/Unilidar_graph_SLAM_IE.cpp)
target_link_libraries(Unilidar_graph_SLAM_IE
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    VisionUtility
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 

add_executable(Unilidar_graph_SLAM_csv ./samples/Unilidar_graph_SLAM_csv.cpp)
target_link_libraries(Unilidar_graph_SLAM_csv
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    VisionUtility
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 


add_executable(Graph_SLAM_Unilidar_Wheel_Camera_visualize ./samples/Graph_SLAM_Unilidar_Wheel_Camera_visualize.cpp)
target_link_libraries(Graph_SLAM_Unilidar_Wheel_Camera_visualize
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    VisionUtility
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 


add_executable(Graph_ROS ./samples/Graph_ROS.cpp)
target_link_libraries(Graph_ROS
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    VisionUtility
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 
add_executable(Graph_SLAM_Unilidar_Wheel_Camera ./samples/Graph_SLAM_Unilidar_Wheel_Camera.cpp)
target_link_libraries(Graph_SLAM_Unilidar_Wheel_Camera
    ${MODULE_NAME}
    KalmanFilter
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
    Laser
    VisionUtility
    IMU
    g2o_core
    g2o_stuff
    g2o_types_slam2d
) 




# SAMPLES EXECUTABLE GENERATION (for samples programs, remove the included in the classes programs)

# The modules RGBDCamera are deprecated

#if (BUILD_RGBDCamera AND BUILD_Mapper3d AND BUILD_SAMPLES)
    #add_executable(MultiGraphOptimization ./samples/MultiGraphOptimization.cpp)
    #target_link_libraries(MultiGraphOptimization
    #    ${MODULE_NAME}
    #    RGBDCamera
    #    Mapper3d
    #    RGBDVisionUtility
    #    ${Boost_PROGRAM_OPTIONS_LIBRARY}
    #)

    #add_executable(MultiGraphOptimizationVelodyne ./samples/MultiGraphOptimizationVelodyne.cpp)
    #target_link_libraries(MultiGraphOptimizationVelodyne
    #    ${MODULE_NAME}
    #    Mapper3d
    #    RGBDVisionUtility
    #    Laser
    #)

    #add_executable(RobotSimulationVelodyne ./samples/RobotSimulationVelodyne.cpp)
    #target_link_libraries(RobotSimulationVelodyne
    #    ${MODULE_NAME}
    #    Mapper3d
    #    RGBDVisionUtility
    #    Laser
    #)

    #add_executable(RobotPositionOptimizationComplet ./samples/RobotPositionOptimizationComplet.cpp)
    #target_link_libraries(RobotPositionOptimizationComplet
    #    ${MODULE_NAME}
    #    RGBDCamera
    #    Mapper3d
    #    RGBDVisionUtility
    #    RobotPoseEstimators
    #    CHARMBot
    #    IPC
    #    RobotBaseCommunicationPoint
    #    Types
    #)

   # add_executable(SlamWithVelodyne ./samples/SlamWithVelodyne.cpp)
    #target_link_libraries(SlamWithVelodyne
     #   ${MODULE_NAME}
    #    Mapper3d
     #   RGBDVisionUtility
     #   RobotPoseEstimators
     #   CHARMBot
    #    IPC
     #   RobotBaseCommunicationPoint
     #   Laser
    #    Types
    #)

    #add_executable(DatasetGenerator ./samples/DatasetGenerator.cpp)
    #target_link_libraries(DatasetGenerator
    #    RobotPoseEstimators
    #    CHARMBot
    #    IPC
    #    RobotBaseCommunicationPoint
    #    Laser
    #    Types
    #)

    #add_executable(OfflineSlamFromDataset ./samples/OfflineSlamFromDataset.cpp)
    #target_link_libraries(OfflineSlamFromDataset
    #    ${MODULE_NAME}
    #    Mapper3d
    #    RGBDVisionUtility
    #    Types
    #)

#endif()

set(MODULE_NAME "ObjectPoseEstimator")

set(BUILD_SAMPLES "ON")
set(BUILD_VISUALIZATION "ON")

find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)

set(ObjectPoseEstimator_SRCS
    ./src/ObjectPoseEstimator.cpp
    ./src/ObjectPoseEstimatorConfiguration.cpp
)

set(MODULE_LIBS
    EventLogger
    RGBDVisionUtility
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
)

add_library(${MODULE_NAME} SHARED ${ObjectPoseEstimator_SRCS})


target_include_directories(${MODULE_NAME} PUBLIC include)
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${nlohmann_json_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

# The modules CameraViewer, RGBDCamera are deprecated

#if(BUILD_SAMPLES STREQUAL "ON")
#    add_executable(BlmModelScan ./samples/BlmModelScan.cpp)
#    target_link_libraries(BlmModelScan 
#        ObjectPoseEstimator
#        Mapper3d
#        ObjectDetection
#        ZMQSocket
#        RGBDVisionUtility
#        PclTypes
#        RGBDCamera
#        CameraViewer
#        ClosedLoopController
#        RobotArm
#        CANSocket
#        SchunkArm
#        RobotArmControllersDeprecated)
#
#    add_executable(BLMPoseEstimatorAndEndEffectorAligning ./samples/BLMPoseEstimatorAndEndEffectorAligning.cpp)
#    target_link_libraries(BLMPoseEstimatorAndEndEffectorAligning 
#        ObjectPoseEstimator
#        Mapper3d
#        ObjectDetection
#        ZMQSocket
#        RGBDVisionUtility
#        RGBDCamera
#        CameraViewer
#        ClosedLoopController
#        RobotArm
#        CANSocket
#        SchunkArm
#        RobotArmControllersDeprecated
#        PclTypes)
#
#    add_executable(OpenDaysmapping ./samples/OpenDaysMapping.cpp)
#    target_link_libraries(OpenDaysmapping 
#        ObjectPoseEstimator
#        Mapper3d
#        ObjectDetection
#        ZMQSocket
#        RGBDVisionUtility
#        RGBDCamera
#        CameraViewer
#        ClosedLoopController
#        RobotArm
#        CANSocket
#        SchunkArm
#        RobotArmControllersDeprecated)
#endif()

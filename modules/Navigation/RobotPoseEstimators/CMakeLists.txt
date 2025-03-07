set(MODULE_NAME "RobotPoseEstimators")

find_package(Boost 1.56 REQUIRED COMPONENTS
             program_options)
find_package(nlohmann_json REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${Boost_INCLUDE_DIRS}
)

set (MODULE_LIBS
    CommonInterfaces
    ${Boost_LIBRARIES}
    EventLogger
    Types
    RobotBase
    StateEstimator
    IMU
)

set(ROBOT_POSE_ESTIMATOR_SRC
    ./src/RobotPoseEstimator.cpp
)


if(BUILD_TrackingCamera)
    set (MODULE_LIBS 
        ${MODULE_LIBS}
        TrackingCamera
    )
    set(ROBOT_POSE_ESTIMATOR_SRC ${ROBOT_POSE_ESTIMATOR_SRC}
        ./src/RobotPoseEstimatorTrackingCam.cpp
    )
endif()


add_library(${MODULE_NAME} SHARED ${ROBOT_POSE_ESTIMATOR_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})



if (BUILD_RobotBaseController AND BUILD_TrackingCamera)
    add_executable(RobotPoseEstimatorTrackingCamera ./samples/RobotPoseEstimatorTrackingCamera.cpp)
    target_link_libraries(RobotPoseEstimatorTrackingCamera ${MODULE_NAME} CHARMBot RobotBaseController)
endif()

if (BUILD_RobotBaseController)
    add_executable(RobotPoseEstimatorCHARMBOT ./samples/RobotPoseEstimatorCHARMBOT.cpp)
    target_link_libraries(RobotPoseEstimatorCHARMBOT ${MODULE_NAME} CHARMBot RobotBaseController)
endif()


set(MODULE_NAME "Mapper3d")

set(BUILD_SAMPLES "ON")

find_package(Eigen3 REQUIRED)
find_package(VISP REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(octomap REQUIRED)

set(Mapper3d_SRCS
    ./src/Mapper3d.cpp
    ./src/HandHeldMapper3d.cpp
    ./src/NormalsColorOctree.cpp
)

set(MODULE_LIBS
    EventLogger
    ${VISP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
)

add_library(${MODULE_NAME} SHARED ${Mapper3d_SRCS})


target_include_directories(${MODULE_NAME} PUBLIC include)
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${VISP_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${nlohmann_json_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${OCTOMAP_INCLUDE_DIRS})

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

# The modules CameraViewer, RGBDCamera are deprecated

#if(BUILD_SAMPLES STREQUAL "ON")
#    add_executable(HandHeldScannerOffline ./samples/HandHeldScannerOffline.cpp)
#    target_link_libraries(HandHeldScannerOffline
#        Mapper3d
#        RGBDCamera
#        CameraViewer
#    )
#
#    add_executable(ImageSaver ./samples/ImageSaver.cpp)
#    target_link_libraries(ImageSaver
#        RGBDCamera
#        CameraViewer
#    )
#
#    add_executable(MapWhileMovingSchunk ./samples/MapWhileMovingSchunk.cpp)
#    target_link_libraries(MapWhileMovingSchunk 
#        Mapper3d
#        RGBDCamera
#        CameraViewer
#        RobotArm
#        CANSocket
#        SchunkArm
#        ClosedLoopController
#        RobotArmControllersDeprecated)
#endif()

#if (BUILD_KinovaArm AND BUILD_RGBDCamera AND BUILD_ClosedLoopController AND BUILD_TrajectoryGenerator)
#    add_executable(MapWhileMovingKinova ./samples/MapWhileMovingKinova.cpp)
#    target_link_libraries(MapWhileMovingKinova Mapper3d
#                                               RGBDCamera
#                                               Types
#                                               RobotArm
#                                               KinovaArm
#                                               ClosedLoopController
#                                               RobotArmControllersDeprecated)
#endif()

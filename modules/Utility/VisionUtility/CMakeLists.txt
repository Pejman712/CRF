#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Carlos Prados Sesmero && Alejandro Diaz Rosales CERN && Pejman Habibiroudkenar EN/SMM/MRO                 ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "VisionUtility")
if(DISABLE_${MODULE_NAME})
    return()
endif()

find_package(small_gicp)
find_package(Boost REQUIRED COMPONENTS program_options)
#find_package(Boost)


set(MODULE_LIBS EventLogger
                ${PCL_LIBRARIES}
                ${OpenCV_LIBRARIES}
                ${small_gicp_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${OpenCV_INCLUDE_DIRS}
                 ${EIGEN3_INDCLUDE_DIRS}
                 ${PCL_INCLUDE_DIRS}
                 ${Boost_INCLUDE_DIRS}
                 ${small_gicp_INCLUDE_DIRS}
)
set(SRC_FILES ./src/Image/ImageJSONConverter.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)


add_executable(visualize_keypoints ./samples/visualize_keypoints.cpp)
target_link_libraries(visualize_keypoints ${MODULE_NAME})

add_executable(visualize_edge ./samples/visualize_edge.cpp)
target_link_libraries(visualize_edge ${MODULE_NAME})

add_executable(edge_detection_tracking ./samples/edge_detection_tracking.cpp)
target_link_libraries(edge_detection_tracking ${MODULE_NAME})

add_executable(gicp_registeration ./samples/gicp_registeration.cpp)
target_link_libraries(gicp_registeration ${MODULE_NAME})

add_executable(convexhull ./samples/convexhull.cpp)
target_link_libraries(convexhull ${MODULE_NAME})

add_executable(convexline ./samples/convexline.cpp)
target_link_libraries(convexline ${MODULE_NAME})

add_executable(gicp_registeration_active_unilidar ./samples/gicp_registeration_active_unilidar.cpp)
target_link_libraries(gicp_registeration_active_unilidar ${MODULE_NAME})

add_executable(edge_test ./samples/edge_test.cpp)
target_link_libraries(edge_test ${MODULE_NAME})

add_executable(keypoints_test ./samples/keypoints_test.cpp)
target_link_libraries(keypoints_test ${MODULE_NAME})

add_executable(gicp_key_test ./samples/keypoints_test.cpp)
target_link_libraries(keypoints_test ${MODULE_NAME})

add_executable(ndt_slam ./samples/ndt_slam.cpp)
target_link_libraries(ndt_slam ${MODULE_NAME})

add_executable(gicp_test ./samples/gicp_test.cpp)
target_link_libraries(gicp_test ${MODULE_NAME})

add_executable(gicp_ndt_registeration ./samples/gicp_ndt_registeration.cpp)
target_link_libraries(gicp_ndt_registeration ${MODULE_NAME})

add_executable(gicp_fpcs_registeration ./samples/gicp_fpcs_registeration.cpp)
target_link_libraries(gicp_fpcs_registeration ${MODULE_NAME} ) 

add_executable(gicp_sac_ia_registeration ./samples/gicp_sac_ia_registeration.cpp)
target_link_libraries(gicp_sac_ia_registeration ${MODULE_NAME}) 

add_executable(gicp_walldetector ./samples/gicp_walldetector.cpp)
target_link_libraries(gicp_walldetector ${MODULE_NAME}) 

add_executable(wall_corner ./samples/wall_corner.cpp)
target_link_libraries(wall_corner ${MODULE_NAME}) 

add_executable(dynahull ./samples/dynahull.cpp)
target_link_libraries(dynahull 
${MODULE_NAME}
pcl_ml
KalmanFilter
SPSRobot
RobotBase
EtherCATDevices
EtherCATDrivers
${Boost_LIBRARIES}
Laser
IMU
)


add_executable(mira3_kalman_filter ./samples/mira3_kalman_filter.cpp)
target_link_libraries(mira3_kalman_filter
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

add_executable(mira3_kalman_filter_teleop ./samples/mira3_kalman_filter_teleop.cpp)
target_link_libraries(mira3_kalman_filter_teleop
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


add_executable(SPS_teleop ./samples/SPS_teleop.cpp)
target_link_libraries(SPS_teleop
    ${MODULE_NAME}
    SPSRobot
    RobotBase
    EtherCATDevices
    EtherCATDrivers
    ${Boost_LIBRARIES}
) 

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

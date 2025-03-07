set(MODULE_NAME "HandEyeCalibration")

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)

set(MODULE_LIBS EventLogger
                IPC
                ${Boost_LIBRARIES}
                ${OpenCV_LIBRARIES}
                ${VISP_LIBRARIES}
)

set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIRS}
                 ${OpenCV_INCLUDE_DIRS}
                 ${VISP_INCLUDE_DIRS}   
)

set(SRC_FILES ./src/HandEye.cpp
              ./src/FixedHandeye.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

if (BUILD_KinovaArm AND BUILD_Cameras AND BUILD_RobotArmController)
    add_executable(Rs2KinovaHandEye ./samples/Rs2KinovaHandEye.cpp
                                    ./src/DataAcquisitionRobotArmRealSense.cpp)
    target_link_libraries(Rs2KinovaHandEye HandEyeCalibration
                                           Cameras
                                           KinovaArm
                                           RobotArmController)
endif()

if (BUILD_SchunkArm AND BUILD_Cameras AND BUILD_RobotArmController)
    add_executable(Rs2SchunkHandEye ./samples/Rs2SchunkHandEye.cpp
                                    ./src/DataAcquisitionRobotArmRealSense.cpp)
    target_link_libraries(Rs2SchunkHandEye HandEyeCalibration
                                           Cameras                                           
                                           SchunkArm
                                           CANSocket
                                           RobotArmController)
endif()

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO 2020                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "RGBDCameras")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Cameras
                EventLogger
                ${OpenCV_LIBRARIES}
                ${PCL_LIBRARIES}
                ${nlohmann_json_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${OpenCV_INCLUDE_DIRS}
                 ${PCL_INCLUDE_DIRS}
                 ${nlohmann_json_INCLUDE_DIRS}
)
set(SRC_FILES
)

if(NOT DISABLE_RealSenseCamera)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${realsense2_LIBRARY}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${realsense2_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RealSenseCamera/RealSenseCamera.cpp
                  ./src/RealSenseCamera/D405RealSenseCamera.cpp
    )
endif()

if(NOT DISABLE_RGBDCameraClient)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DataPackets
                    DataPacketSocket
                    VideoCodecs
                    VisionUtility
                    CommunicationUtility
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RGBDCameraClient/RGBDCameraClient.cpp
    )
endif()


if(NOT DISABLE_RGBDCameraCommunicationPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DataPackets
                    DataPacketSocket
                    CommunicationPointServer
                    VideoCodecs
                    VisionUtility
                    ${LZ4_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${LZ4_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/RGBDCameraCommunicationPoint/RGBDCameraManager.cpp
                  ./src/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPoint.cpp
                  ./src/RGBDCameraCommunicationPoint/RGBDCameraCommunicationPointFactory.cpp
    )
endif()

if (DISABLE_RealSenseCamera AND DISABLE_RGBDCameraCommunicationPoint AND DISABLE_RGBDCameraClient)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)
endif()

# There is an error in the GitLab pipeline that makes the test fail in ubuntu20
# For more information check the issue:
#   - RGBD Cameras unit testing is not able to find the RealSense library in the Ubuntu20 pipeline
# (jplayang)

execute_process(
    COMMAND lsb_release -r -s
    OUTPUT_VARIABLE UBUNTU_VERSION
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Convert the version string to an integer
string(REPLACE "." "" UBUNTU_VERSION_INT "${UBUNTU_VERSION}")
if ("${UBUNTU_VERSION_INT}" VERSION_GREATER 2004)
    if (ENABLE_UnitTests)
        add_subdirectory(tests)
    endif()
endif()

# Optional packages for the different samples
find_package(Boost COMPONENTS program_options)

if(NOT DISABLE_RGBDCameraCommunicationPoint)
    if(NOT DISABLE_RealSenseCamera)
        add_executable(RealSenseCameraPoint ./samples/RealSenseCameraPoint.cpp)
        target_link_libraries(RealSenseCameraPoint VisionUtility ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
    endif()
endif()

if(NOT DISABLE_RealSenseCamera)
    add_executable(RealSenseCameraSample ./samples/RealSenseCameraSample.cpp)
    target_link_libraries(RealSenseCameraSample VisionUtility ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})

    add_executable(RealSenseCameraViewer ./samples/RealSenseCameraViewer.cpp)
    target_link_libraries(RealSenseCameraViewer VisionUtility ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

if(NOT DISABLE_RGBDCameraClient)
   add_executable(RGBDCameraClientSample ./samples/RGBDCameraClientSample.cpp)
   target_link_libraries(RGBDCameraClientSample VisionUtility ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

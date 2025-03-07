set(MODULE_NAME "TrackingCamera")

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Boost REQUIRED)

set(TRACKING_CAM_SRC
    ./src/IntelT265Grabber.cpp
    ./src/TrackingCameraUtils.cpp
    ./src/TrackingCameraUtils.cpp
    ./src/ContextFactory.cpp
)
set(SYSTEM_INCLUDE_DIRS
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIR}
)

set(MODULE_LIBS
    EventLogger
    CommonInterfaces
    ${OpenCV_LIBRARIES}
    ${realsense2_LIBRARY}
    ${Boost_LIBRARIES}
)

add_library(${MODULE_NAME} SHARED
    ${TRACKING_CAM_SRC}
)

target_include_directories(${MODULE_NAME} SYSTEM PUBLIC ${SYSTEM_INCLUDE_DIRS})
target_include_directories(${MODULE_NAME} PUBLIC ./include)

target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

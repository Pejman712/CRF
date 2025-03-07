set(MODULE_NAME "ThermalCamera")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Ebus REQUIRED)

set(INCLUDE_DIRS
    ./include
    ${Ebus_INCLUDE_DIRS}
)
set(MODULE_LIBS
    IPC
    CommUtility
    EventLogger
    CommonInterfaces
    irdirectsdk
    opencv_core
    opencv_highgui
    ${Ebus_LIBRARIES}
)
set(SRC
    ./src/FLIRCam.cpp
    ./src/FLIRDeviceParams.cpp
    ./src/FLIRFactory.cpp
    ./src/FLIRTempConverter.cpp
    ./src/OptrisDeviceFactory.cpp
    ./src/OptrisPi.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(optris_thermal ./samples/optris_thermal.cpp)
target_link_libraries(optris_thermal ThermalCamera)

find_package(OpenCV REQUIRED )
add_executable(flir ./samples/flir_test.cpp)
target_include_directories(flir PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(flir ThermalCamera ${OpenCV_LIBRARIES})

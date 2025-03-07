set(MODULE_NAME "SonyDSCCamera")
find_package( OpenCV REQUIRED )
include(${CMAKE_SOURCE_DIR}/cmake/FindLibUSB.cmake)

set(INCLUDE_DIRS
    ./include
    ./include/SonyDSCCamera/libsoccptp
    ./include/SonyDSCCamera/libsoccptp/ports/
    ${LibUSB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
set(MODULE_LIBS
    ${LibUSB_LIBRARIES}
    Camera
    udev
    IPC
    ${OpenCV_LIBS}
)
set(SRC
    ./src/SonyDSCCamera.cpp
    ./src/SOCCManager.cpp
    ./src/parser.cpp
    ./src/ports_ptp_impl.cpp
    ./src/ports_usb_impl.cpp
    ./src/socc_ptp.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(sony_camera ./samples/sony_camera.cpp)
target_link_libraries(sony_camera SonyDSCCamera)


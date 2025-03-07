set (MODULE_NAME "PanTilt")

find_package(nlohmann_json REQUIRED)
find_package(OpenCV 3.4 QUIET)

set (MODULE_LIBS EventLogger
                 CommonInterfaces
                 Dynamixel
)

set(INCLUDE_DIRS ./include
)

set(SRC_FILES src/DynamixelPanTilt.cpp)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

if(OpenCV_FOUND)
    add_executable(panTilt ./samples/PanTilt.cpp)
    target_link_libraries(panTilt PanTilt ${OpenCV_LIBRARIES})
endif()
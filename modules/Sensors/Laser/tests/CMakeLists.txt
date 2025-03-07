set(LASER_TESTS_FILES main.cpp
                      LaserConfigurationTests.cpp
                      LaserUtilsTests.cpp
                      VelodyneHDLLaserTests/VelodyneHDLLaserPCLTests.cpp
                      VelodyneHDLLaserTests/VelodyneHDLLaserTests.cpp
                      VelodyneHDLLaserTests/VelodyneHDLLaserConfigurationTests.cpp)

include(${CMAKE_SOURCE_DIR}/modules/Sensors/Laser/cmake/FindHokuyoLib.cmake)
if(HokuyoLib_FOUND)
    set(LASER_TESTS_FILES ${LASER_TESTS_FILES} HokuyoLaserTests/HokuyoLaserTests.cpp)
endif()

add_executable(ut-laser ${LASER_TESTS_FILES})
target_link_libraries(ut-laser gmock
                               CRFMocks
                               EventLogger
                               Laser)

add_test(LaserTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-laser)

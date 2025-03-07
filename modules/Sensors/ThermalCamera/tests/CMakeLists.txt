set(THERMALCAMERA_TESTS_FILES main.cpp
                              OptrisPiTests.cpp
                              FLIRCamTests.cpp)

add_executable(ut-thermalcamera ${THERMALCAMERA_TESTS_FILES})
target_link_libraries(ut-thermalcamera gmock
                                       CRFMocks
                                       ThermalCamera)

add_test(ThermalCameraUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-thermalcamera)

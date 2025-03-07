set(HANDEYECALIBRATION_TESTS_FILES main.cpp
                                   HandEyeTests.cpp
                                   FixedHandeyeTests.cpp)

add_executable(ut-handeyecalibration ${HANDEYECALIBRATION_TESTS_FILES})
target_link_libraries(ut-handeyecalibration gmock
                                            CRFMocks
                                            HandEyeCalibration)

add_test(HandEyeCalibrationTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-handeyecalibration)

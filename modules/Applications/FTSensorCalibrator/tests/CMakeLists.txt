set(FTSENSORCALIBRATOR_TESTS_FILES main.cpp
                                    FTSensorCalibrationConfigurationTests.cpp
                                   SensorCalibratorTests.cpp
                                   )

add_executable(ut-ftsensorcalibrator ${FTSENSORCALIBRATOR_TESTS_FILES})
target_link_libraries(ut-ftsensorcalibrator gmock
                                            CRFMocks
                                            FTSensorCalibrator)

add_test(FTSensorCalibratorTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ftsensorcalibrator)

set(HEALTHDETECTION_TESTS_FILES main.cpp
                                HealthDetectionTests.cpp
)

add_executable(ut-healthdetection ${HEALTHDETECTION_TESTS_FILES})
target_link_libraries(ut-healthdetection gmock
                                         CRFMocks
                                         HealthDetection
                                         Radars
                                         PeakDetection
                                         StateEstimator
                                         EventLogger)

add_test(HealthDetectionTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-healthdetection)

execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tests/Configurations/Sensors/Radar/TestData/RadarBreathingPersonInFrameDownload.sh [${CMAKE_SOURCE_DIR}])
execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tests/Configurations/Sensors/Radar/TestData/RadarHighVarianceDownload.sh [${CMAKE_SOURCE_DIR}])
execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tests/Configurations/Sensors/Radar/TestData/RadarNoPersonInFrameDownload.sh [${CMAKE_SOURCE_DIR}])

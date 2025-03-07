set(PEAK_DETECTION_TESTS_FILES main.cpp
                               CFARTests.cpp
                               GradientPeakDetectionTests.cpp
)

add_executable(ut-peakdetection ${PEAK_DETECTION_TESTS_FILES})
target_link_libraries(ut-peakdetection gmock
                                       CRFMocks
                                       PeakDetection
                                       EventLogger
)

add_test(PeakDetectionTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-peakdetection)

execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tests/Configurations/Sensors/Radar/TestData/MatrixPeakPresentImagDownload.sh [${CMAKE_SOURCE_DIR}])
execute_process(COMMAND bash ${CMAKE_SOURCE_DIR}/tests/Configurations/Sensors/Radar/TestData/MatrixPeakPresentRealDownload.sh [${CMAKE_SOURCE_DIR}])

set(LEAK_DETECTOR_TESTS_FILES main.cpp
                              PhoenixL300iTests.cpp
                              PhoenixTelegramTests.cpp
                              CrcCalcForTest.cpp)

add_executable(ut-leakdetector ${LEAK_DETECTOR_TESTS_FILES})
target_link_libraries(ut-leakdetector gmock
                                      CRFMocks
                                      LeakDetector)

add_test(LeakDetectorTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-leakdetector)

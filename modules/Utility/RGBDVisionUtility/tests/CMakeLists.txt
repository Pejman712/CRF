set(RGBDVisionUtility_TESTS_FILES main.cpp
                                  PCLUtilsTests.cpp)

add_executable(ut-rgbdvisionutility ${RGBDVisionUtility_TESTS_FILES})
target_link_libraries(ut-rgbdvisionutility gmock
                                           CRFMocks
                                           RGBDVisionUtility)

add_test(RGBDVisionUtilityTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-rgbdvisionutility)

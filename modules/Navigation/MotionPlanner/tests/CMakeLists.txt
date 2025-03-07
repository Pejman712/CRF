set(AUTONOMOUS_NAVIGATION_TESTS_FILES main.cpp	
                    MotionPlannerLaserTests.cpp)

set(AUTONOMOUS_NAVIGATION_TEST_LIBS gmock MotionPlanner)

add_executable(ut-motionplannerlaser ${AUTONOMOUS_NAVIGATION_TESTS_FILES})
target_link_libraries(ut-motionplannerlaser gmock
                             CRFMocks
                             MotionPlanner)

add_test(MotionPlannerTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-motionplannerlaser)

set(STATE_ESTIMATOR_TESTS_FILES main.cpp
                                StateEstimatorTests.cpp)

add_executable(ut-stateestimator ${STATE_ESTIMATOR_TESTS_FILES})
target_link_libraries(ut-stateestimator gmock 
                                        CRFMocks
                                        StateEstimator)

add_test(StateEstimatorTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-stateestimator)

set(OBJECTPOSEESTIMATOR_TESTS_FILES main.cpp
                        ObjectPoseEstimatorTest.cpp
                        ObjectPoseEstimatorConfigurationTest.cpp)

add_executable(ut-objectPoseEstimator ${OBJECTPOSEESTIMATOR_TESTS_FILES})
target_link_libraries(ut-objectPoseEstimator gmock
                                  CRFMocks
                                  ObjectPoseEstimator)

add_test(ObjectPoseEstimatorTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-objectPoseEstimator)

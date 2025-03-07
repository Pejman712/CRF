set(POSE_ESTIMATOR_TESTS_FILES main.cpp
                               RobotPoseEstimatorTests.cpp
                               RobotPoseEstimatorTrackingCamTests.cpp
			       			  )

add_executable(ut-robotposeestimator ${POSE_ESTIMATOR_TESTS_FILES})
target_link_libraries(ut-robotposeestimator gmock
                                            CRFMocks
                                            RobotPoseEstimators)

add_test(RobotPoseEstimatorsTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotposeestimator)

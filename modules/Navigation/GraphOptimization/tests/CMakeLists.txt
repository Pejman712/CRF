set(GRAPHOPTIMIZATION_TESTS_FILES main.cpp
                         GraphOptimizationTest.cpp
                         SlamTest.cpp)

add_executable(ut-graphoptimization ${GRAPHOPTIMIZATION_TESTS_FILES})

target_link_libraries(ut-graphoptimization gmock
                                  CRFMocks
                                  GraphOptimization
                                  RobotPoseEstimators)

add_test(GraphOptimizationTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-graphoptimization)

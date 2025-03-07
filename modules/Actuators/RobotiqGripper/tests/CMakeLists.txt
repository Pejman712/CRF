set(ROBOTIQ_GRIPPER_TESTS_FILES main.cpp
                                RobotiqGripperTests.cpp)

add_executable(ut-robotiqgripper ${ROBOTIQ_GRIPPER_TESTS_FILES})
target_link_libraries(ut-robotiqgripper gmock
                                        CRFMocks
                                        RobotiqGripper)

add_test(RobotiqGripperTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-robotiqgripper)

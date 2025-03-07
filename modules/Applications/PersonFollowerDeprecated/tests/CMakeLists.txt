set(PERSONDETECTOR_TESTS_FILES main.cpp
    PersonDetectorTests.cpp
    PersonTrackerTests.cpp
    PersonFollowerTests.cpp
)
add_executable(ut-personfollower ${PERSONDETECTOR_TESTS_FILES})
target_link_libraries(ut-personfollower gmock CRFMocks PersonFollower)

add_test(PersonFollowingTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-personfollower)
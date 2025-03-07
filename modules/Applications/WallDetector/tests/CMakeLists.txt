set(WALLDETECTOR_TESTS_FILES main.cpp
    WallDetectorTests.cpp
)

add_executable(ut-walldetector ${WALLDETECTOR_TESTS_FILES})
target_link_libraries(ut-walldetector gmock CRFMocks WallDetector)
add_test(WallDetectorTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-walldetector)

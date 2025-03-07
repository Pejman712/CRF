set(Dynamixel_TESTS_FILES
    main.cpp
    DynamixelTests.cpp
)

add_executable(ut-dynamixel ${Dynamixel_TESTS_FILES})
target_link_libraries(
    ut-dynamixel
    gmock
    CRFMocks
    Dynamixel
    EventLogger)

add_test(DynamixelTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-dynamixel)

set(TrackingCameraLive_TESTS_FILES main.cpp
                                   TrackingCameraLiveTests.cpp)

add_executable(ut-trackingcamlive ${TrackingCameraLive_TESTS_FILES})
target_link_libraries(ut-trackingcamlive gmock
                                         CRFMocks
                                         TrackingCamera
                                         EventLogger)

add_test(TrackingCameraTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-trackingcamlive)

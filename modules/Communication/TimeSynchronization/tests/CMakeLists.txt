set(TIME_SYNCHRONIZATION_TESTS_FILES main.cpp
                                     TimeSynchronizationServerTests.cpp)

add_executable(ut-timesynchronization ${TIME_SYNCHRONIZATION_TESTS_FILES})
target_link_libraries(ut-timesynchronization gmock
                                             CRFMocks
                                             TimeSynchronization)

add_test(TimeSynchronizationTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-timesynchronization)

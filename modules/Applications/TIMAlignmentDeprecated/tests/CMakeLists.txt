set(TIMAlignment_TESTS_FILES main.cpp
                             TIMAlignmentTests.cpp)

add_executable(ut-timalignment ${TIMAlignment_TESTS_FILES})
target_link_libraries(ut-timalignment gmock
                                      CRFMocks
                                      TIMAlignment)

add_test(TIMAlignmentTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-timalignment)

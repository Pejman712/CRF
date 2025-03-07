set(INCLINOMETER_TESTS_FILES main.cpp
                             InclinometerTests.cpp)

add_executable(ut-inclinometer ${INCLINOMETER_TESTS_FILES})
target_link_libraries(ut-inclinometer gmock
                                      CRFMocks
                                      Inclinometer)

add_test(InclinometerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-inclinometer)

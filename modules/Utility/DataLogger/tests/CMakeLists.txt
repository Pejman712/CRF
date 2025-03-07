set(DATALOGGER_TESTS_FILES main.cpp
                           DataLoggerTests.cpp)

add_executable(ut-datalogger ${DATALOGGER_TESTS_FILES})

target_link_libraries(ut-datalogger gmock
                                   CRFMocks
                                   DataLogger
                                   )

add_test(DataLoggerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-datalogger)

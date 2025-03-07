set(DSO_TESTS_FILES main.cpp
                    DsoTests.cpp)

set(DSO_TEST_LIBS gmock DsoDevel)

add_executable(ut-dso ${DSO_TESTS_FILES})
target_link_libraries(ut-dso gmock
                             CRFMocks
                             DsoDevel)

add_test(DsoTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-dso)

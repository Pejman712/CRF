set(SQLADAPTER_TESTS_FILES main.cpp
                           SqlAdapterTests.cpp
                           SqlResultTests.cpp
                           MySqlAdapterTests.cpp)

add_executable(ut-sqladapter ${SQLADAPTER_TESTS_FILES})
target_link_libraries(ut-sqladapter gmock
                                    CRFMocks
                                    SqlAdapter)

add_test(SqlAdapterTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-sqladapter)

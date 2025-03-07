set(XLS_ADAPTER_TESTS_FILES main.cpp
                            XlsAdapterTests.cpp
                            XlsMessageTests.cpp)

add_executable(ut-xlsadapter ${XLS_ADAPTER_TESTS_FILES})
target_link_libraries(ut-xlsadapter gmock
                                    CRFMocks
                                    XlsAdapter)

add_test(XlsAdapterTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-xlsadapter)

set(NETWORK_INTEGRATION_TESTS_FILES main.cpp
                                    NetworkIntegrationTests.cpp)

add_executable(ut-networkintegration ${NETWORK_INTEGRATION_TESTS_FILES})
target_link_libraries(ut-networkintegration gmock
                                            CRFMocks
                                            NetworkServer
                                            NetworkClient)

add_test(NetworkIntegrationTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-networkintegration)

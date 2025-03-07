set(NETWORK_SERVER_TESTS_FILES main.cpp
                               TcpServerTests.cpp
                               UdpServerTests.cpp)

add_executable(ut-networkserver ${NETWORK_SERVER_TESTS_FILES})
target_link_libraries(ut-networkserver gmock
                                       CRFMocks
                                       NetworkServer)

add_test(NetworkServerTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-networkserver)

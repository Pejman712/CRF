find_package(nlohmann_json REQUIRED)

set(NETWORK_CLIENT_TESTS_FILES main.cpp
                               FetchWriteClientTests.cpp
                               TcpClientTests.cpp
                               XlsTcpClientTests.cpp)

add_executable(ut-networkclient ${NETWORK_CLIENT_TESTS_FILES})
target_link_libraries(ut-networkclient gmock
                                       CRFMocks
                                       NetworkClient)

add_test(NetworkClientTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-networkclient)

set(WEBSERVERS_TESTS_FILES main.cpp
    TIMWebPointTests.cpp
    TIMWebServerTests.cpp)
add_executable(ut-webservers ${WEBSERVERS_TESTS_FILES})
target_link_libraries(ut-webservers gmock CRFMocks curlpp curl WebServers)

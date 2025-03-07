set(TOOLS_TESTS_FILES main.cpp
                      ToolCommunicationPointTests.cpp
                      CuttingScissorsTests.cpp
                      ScrewdriverTests.cpp)

add_executable(ut-tools ${TOOLS_TESTS_FILES})
target_link_libraries(ut-tools gmock
                               CRFMocks
                               Tools)

add_test(ToolsTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-tools)

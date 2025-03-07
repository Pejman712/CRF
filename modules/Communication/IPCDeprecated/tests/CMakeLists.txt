set(IPC_TESTS_FILES main.cpp
                    FifoAndMmapTests.cpp
                    NetworkIPCTests.cpp)

add_executable(ut-ipc ${IPC_TESTS_FILES})
target_link_libraries(ut-ipc gmock
                             CRFMocks
                             IPC)

add_test(IpcUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-ipc)

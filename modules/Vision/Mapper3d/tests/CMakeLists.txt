set(MAPPER3D_TESTS_FILES main.cpp
                         Mapper3dTest.cpp
                         HandHeldMapper3dTest.cpp)

add_executable(ut-mapper3d ${MAPPER3D_TESTS_FILES})
target_link_libraries(ut-mapper3d gmock
                                  CRFMocks
                                  Mapper3d)

add_test(Mapper3dTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-mapper3d)

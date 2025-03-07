set(RADAR_TESTS_FILES main.cpp
                      FraunhoferRadarTests.cpp)

find_package(ModuleConnector 1.5)

if (ModuleConnector_FOUND)
    set(RADAR_TESTS_FILES ${RADAR_TESTS_FILES}
                          ./XethruRadarTests.cpp
    )
endif()

add_executable(ut-radar ${RADAR_TESTS_FILES})
target_link_libraries(ut-radar gmock
                               CRFMocks
                               Radars
                               EventLogger)

add_test(RadarTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-radar)

set(MODULE_NAME "Radars")

find_package(nlohmann_json REQUIRED)

find_package(ModuleConnector 1.5)

set(INCLUDE_DIRS
    ./include
    ${nlohmann_json_INCLUDE_DIRS}
)

set(MODULE_LIBS
    EventLogger
    SerialCommunication
    CommonInterfaces
)

set(RADAR_SRC ./src/FraunhoferRadar.cpp)


if (ModuleConnector_FOUND)
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${ModuleConnector_INCLUDE_DIRS}
    )
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${ModuleConnector_LIBRARY}

    )
    set(RADAR_SRC ${RADAR_SRC}
                  ./src/XethruAdapter.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${RADAR_SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

add_executable(RadarSample ./samples/RadarSample.cpp)
target_link_libraries(RadarSample Radars)

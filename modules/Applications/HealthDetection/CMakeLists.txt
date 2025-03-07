set(MODULE_NAME "HealthDetection")

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
                 ${Boost_INCLUDE_DIRS}
)

set(MODULE_LIBS EventLogger
                Radars
                PeakDetection
                FourierTransform
                StateEstimator
                ${Boost_FILESYSTEM_LIBRARY}
                ${Boost_SYSTEM_LIBRARY}
)

set(SRC_FILES ./src/HealthDetection.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

find_package(nlohmann_json)
if(nlohmann_json_FOUND AND BUILD_SerialCommunication)
    add_executable(HealthDetectionSample ./samples/HealthDetectionSample.cpp)
    target_link_libraries(HealthDetectionSample HealthDetection ${nlohmann_json_LIBRARY})
endif()

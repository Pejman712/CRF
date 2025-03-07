set (MODULE_NAME "StateEstimator")

find_package(Eigen3 REQUIRED)

add_library(${MODULE_NAME} INTERFACE)

set (MODULE_LIBS
    EventLogger
    ${EIGEN3_LIBRARIES}
)
target_link_libraries(${MODULE_NAME} INTERFACE ${MODULE_LIBS})
target_include_directories(${MODULE_NAME} INTERFACE include)
target_include_directories(${MODULE_NAME} INTERFACE SYSTEM  ${EIGEN3_INCLUDE_DIRS})

add_executable(StateEstimatorExample samples/StateEstimatorExample.cpp)
target_link_libraries(StateEstimatorExample ${MODULE_NAME})

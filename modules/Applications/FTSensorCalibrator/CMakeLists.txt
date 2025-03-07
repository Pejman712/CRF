set (MODULE_NAME "FTSensorCalibrator")

find_package(Eigen3 REQUIRED)

set(INCLUDE_DIRS
        ./include
        ${EIGEN3_INCLUDE_DIR}
)

set (MODULE_LIBS
    EventLogger
    SchunkArm
    RobotArmControllersDeprecated
    Types
    FTSensor
    CommonInterfaces
)

set(SRC
    ./src/FTSensorCalibrator.cpp
    ./src/SinusFunctor.cpp
    ./src/FTSensorCalibratorConfig.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})
target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)

add_executable(CalibrateSchunk ./samples/calibrateSchunk.cpp)
target_link_libraries(CalibrateSchunk ${MODULE_NAME})

add_executable(SchunkAdmittanceControllerIKpos ./samples/SchunkAdmittanceControlInverseKinematics.cpp)
target_link_libraries(SchunkAdmittanceControllerIKpos ${MODULE_NAME})

add_executable(SchunkAdmittanceControllerIKvel ./samples/SchunkAdmittanceControllInverseVelocity.cpp)
target_link_libraries(SchunkAdmittanceControllerIKvel ${MODULE_NAME})

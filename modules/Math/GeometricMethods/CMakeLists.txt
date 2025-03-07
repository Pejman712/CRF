#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "GeometricMethods")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_DeBoor)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DeBoor/DeBoor.cpp
    )
endif()

if(NOT DISABLE_Sinusoid)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/Sinusoid/Sinusoid.cpp
    )
endif()

if(NOT DISABLE_CubicPolynomial)
    set(MODULE_LIBS ${MODULE_LIBS}
                    nlopt
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CubicPolynomial/CubicPolynomial.cpp
    )
endif()

if(NOT DISABLE_CubicOrientationSpline)
    set(MODULE_LIBS ${MODULE_LIBS}
                    Rotation
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CubicOrientationSpline/CubicOrientationSpline.cpp
    )
endif()

if(NOT DISABLE_PointToPoint)
    set(MODULE_LIBS ${MODULE_LIBS}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PointToPoint/PointToPoint.cpp
    )
endif()

if (DISABLE_PointToPoint AND DISABLE_CubicPolynomial AND DISABLE_DeBoor AND DISABLE_Sinusoid)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_DeBoor)
    add_executable(DeBoorSample ./samples/DeBoorSample.cpp)
    target_link_libraries(DeBoorSample GeometricMethods)
endif()

if(NOT DISABLE_Sinusoid)
    add_executable(SinusoidSample ./samples/SinusoidSample.cpp)
    target_link_libraries(SinusoidSample GeometricMethods)
endif()

if(NOT DISABLE_CubicPolynomial)
    add_executable(CubicPolynomialSample ./samples/CubicPolynomialSample.cpp)
    target_link_libraries(CubicPolynomialSample GeometricMethods)
endif()

if(NOT DISABLE_PointToPoint)
    add_executable(PointToPointSample ./samples/PointToPointSample.cpp)
    target_link_libraries(PointToPointSample GeometricMethods)
endif()

if(NOT DISABLE_CubicOrientationSpline)
    add_executable(CubicOrientationSplineSample ./samples/CubicOrientationSplineSample.cpp)
    target_link_libraries(CubicOrientationSplineSample GeometricMethods)
endif()

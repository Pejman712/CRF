#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################


set(GEOMETRIC_METHODS_TESTS_FILES main.cpp
)

if (NOT DISABLE_DeBoor)
    set(GEOMETRIC_METHODS_TESTS_FILES ${GEOMETRIC_METHODS_TESTS_FILES}
                                      DeBoorTests.cpp
    )
endif()

if (NOT DISABLE_Sinusoid)
    set(GEOMETRIC_METHODS_TESTS_FILES ${GEOMETRIC_METHODS_TESTS_FILES}
                                      SinusoidTests.cpp
    )
endif()

if(NOT DISABLE_CubicPolynomial)
    set(GEOMETRIC_METHODS_TESTS_FILES ${GEOMETRIC_METHODS_TESTS_FILES}
                                      CubicPolynomialTests.cpp
    )
endif()

if(NOT DISABLE_PointToPoint)
    set(GEOMETRIC_METHODS_TESTS_FILES ${GEOMETRIC_METHODS_TESTS_FILES}
                                      PointToPointTests.cpp
    )
endif()

if(NOT DISABLE_CubicOrientationSpline)
    set(GEOMETRIC_METHODS_TESTS_FILES ${GEOMETRIC_METHODS_TESTS_FILES}
                                      CubicOrientationSplineTests.cpp
    )
endif()

add_executable(ut-geometricmethods ${GEOMETRIC_METHODS_TESTS_FILES})
target_link_libraries(ut-geometricmethods gmock
                                          GeometricMethods)

add_test(GeometricMethodsUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-geometricmethods)

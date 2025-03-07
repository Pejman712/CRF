#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Bartosz Sójka CERN BE/CEM/MRO 2023                                                                        ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Rotation")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS CommunicationUtility
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)
set(SRC_FILES ./src/Arithmetic.cpp
              ./src/Comparison.cpp
              ./src/Conversions.cpp
              ./src/EulerAngles.cpp
              ./src/IsValidRotation.cpp
              ./src/JsonConverters.cpp
              ./src/Representations.cpp
              ./src/RotationClass.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

add_executable(RotationArithmeticSample ./samples/ArithmeticSample.cpp)
target_link_libraries(RotationArithmeticSample ${MODULE_NAME})

add_executable(RotationComparisonSample ./samples/ComparisonSample.cpp)
target_link_libraries(RotationComparisonSample ${MODULE_NAME})

add_executable(RotationConversionsSample ./samples/ConversionsSample.cpp)
target_link_libraries(RotationConversionsSample ${MODULE_NAME})

add_executable(EulerAnglesSample ./samples/EulerAnglesSample.cpp)
target_link_libraries(EulerAnglesSample ${MODULE_NAME})

add_executable(IsValidRotationSample ./samples/IsValidRotationSample.cpp)
target_link_libraries(IsValidRotationSample ${MODULE_NAME})

add_executable(RotationJsonConvertersSample ./samples/JsonConvertersSample.cpp)
target_link_libraries(RotationJsonConvertersSample ${MODULE_NAME})

add_executable(RotationClassSample ./samples/RotationClassSample.cpp)
target_link_libraries(RotationClassSample ${MODULE_NAME})

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

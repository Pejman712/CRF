#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Giacomo Lunghi CERN EN/SMM/MRO 2019                                                                       ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Types")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Rotation
                ErrorHandler
                EventLogger
                CommunicationUtility
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)
set(SRC_FILES ./src/TaskTypes/TaskPose.cpp
              ./src/TaskTypes/Vector6d.cpp
              ./src/TaskTypes/TaskSpace.cpp
              ./src/JointTypes/VectorXd.cpp
              ./src/Arithmetic.cpp
              ./src/Comparison.cpp
              ./src/Conversions.cpp
              ./src/JsonConverters.cpp
)

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

add_executable(TypesArithmeticSample samples/ArithmeticSample.cpp)
target_link_libraries(TypesArithmeticSample ${MODULE_NAME})

add_executable(TypesComparisonSample samples/ComparisonSample.cpp)
target_link_libraries(TypesComparisonSample ${MODULE_NAME})

add_executable(TypesConversionsSample samples/ConversionsSample.cpp)
target_link_libraries(TypesConversionsSample ${MODULE_NAME})

add_executable(JointTypesSample samples/JointTypesSample.cpp)
target_link_libraries(JointTypesSample ${MODULE_NAME})

add_executable(TypesJsonConvertersSample samples/JsonConvertersSample.cpp)
target_link_libraries(TypesJsonConvertersSample ${MODULE_NAME})

add_executable(SignalsSample samples/SignalsSample.cpp)
target_link_libraries(SignalsSample ${MODULE_NAME})

add_executable(TaskTypesSample samples/TaskTypesSample.cpp)
target_link_libraries(TaskTypesSample ${MODULE_NAME})

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

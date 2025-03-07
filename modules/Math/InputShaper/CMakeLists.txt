#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "InputShaper")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_CubicPolynomialShaper)
    set(MODULE_LIBS ${MODULE_LIBS}
                    GeometricMethods
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/CubicPolynomialShaper/CubicPolynomialShaper.cpp
    )
endif()

if (DISABLE_CubicPolynomialShaper)
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

if(NOT DISABLE_CubicPolynomialShaper)
    add_executable(CubicPolynomialShaperSample ./samples/CubicPolynomialShaperSample.cpp)
    target_link_libraries(CubicPolynomialShaperSample InputShaper)
endif()

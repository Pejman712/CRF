#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playan Garai CERN BE/CEM/MRO 2021                                                                   ##
##         David Günter Forkel CERN BE/CEM/MRO 2021                                                                  ##
##         Jean Paul Sulca CERN BM-CEM-MRO 2023                                                                      ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "MassSpringDamper")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_MassSpringDamperR1)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DigitalFilter
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MSDR1/MSDR1DirectForm2Abstract.cpp
                  ./src/MSDR1/MSDR1ImpulseInvarianceIIR.cpp
                  ./src/MSDR1/MSDR1StepInvarianceIIR.cpp
                  ./src/MSDR1/MSDR1BilinearTransformIIR.cpp
    )
endif()

if(NOT DISABLE_MassSpringDamperRn)
    set(MODULE_LIBS ${MODULE_LIBS}
                    DigitalFilter
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
    )
    set(SRC_FILES ${SRC_FILES}
    )
endif()

if (DISABLE_MassSpringDamperR1)
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

if(NOT DISABLE_MassSpringDamperR1)
    add_executable(MSDR1ImpulseInvarianceIIRSample ./samples/MSDR1ImpulseInvarianceIIRSample.cpp)
    target_link_libraries(MSDR1ImpulseInvarianceIIRSample ${MODULE_NAME})
endif()

if(NOT DISABLE_MassSpringDamperRn)
    find_package(Boost COMPONENTS program_options)
    add_executable(MSDRnDecoupledSample ./samples/MSDRnDecoupledSample.cpp)
    target_link_libraries(MSDRnDecoupledSample ${MODULE_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})
endif()

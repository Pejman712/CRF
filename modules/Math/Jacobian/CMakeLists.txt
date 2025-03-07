#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Laura Rodrigo Pérez CERN BE/CEM/MRO 2022                                                                  ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "Jacobian")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                Types
                Rotation
                ErrorHandler
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)

if(NOT DISABLE_MathExprJacobian)
    set(MODULE_LIBS ${MODULE_LIBS}
                    MathExprTk
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/MathExprJacobian/MathExprJacobian.cpp
    )
endif()

if(NOT DISABLE_KinChainJacobian)
    set(MODULE_LIBS ${MODULE_LIBS}
                    KinematicChain
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/KinChainJacobian/KinChainJacobian.cpp
    )
endif()

if (DISABLE_MathExprJacobian AND DISABLE_KinChainJacobian)
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

if(NOT DISABLE_KinChainJacobian AND NOT DISABLE_URDFKinematicChain)
    add_executable(KinChainJacobianSample ./samples/KinChainJacobianSample.cpp)
    target_link_libraries(KinChainJacobianSample ${MODULE_NAME})
endif()


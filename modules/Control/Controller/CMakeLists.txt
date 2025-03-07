#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "Controller")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ErrorHandler
                Types
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES ./src/PIDController.cpp
)

if(NOT DISABLE_DirectOpenLoopVelocity)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    InverseKinematics
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/DirectOpenLoopVelocity/DirectOpenLoopVelocity.cpp
    )
endif()

if(NOT DISABLE_PositionCtrlVelocityFF)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    InverseKinematics
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/PositionCtrlVelocityFF/PositionCtrlVelocityFF.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall -Wpedantic -pthread)

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_PositionCtrlVelocityFF)
    add_executable(PositionCtrlVelocityFFSample ./samples/PositionCtrlVelocityFF/PositionCtrlVelocityFFSample.cpp)
    target_link_libraries(PositionCtrlVelocityFFSample Controller)
endif()

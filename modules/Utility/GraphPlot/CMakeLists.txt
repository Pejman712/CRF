#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playán Garai CERN BE/CEM/MRO 2021                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "GraphPlot")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS EventLogger
                Types
                ${VTK_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${VTK_INCLUDE_DIRS}
)
set(SRC_FILES ./src/GraphPlot.cpp
)

if (NOT DISABLE_MotionControllerPlotter)
    set(MODULE_LIBS ${MODULE_LIBS}
                    MotionController
    )
    set(SRC_FILES ${SRC_FILES}
                ./src/MotionControllerPlotter/MotionControllerPlotter.cpp
    )
endif()

add_library(${MODULE_NAME} SHARED ${SRC_FILES})
target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

target_compile_options(${MODULE_NAME} PRIVATE -Wall)

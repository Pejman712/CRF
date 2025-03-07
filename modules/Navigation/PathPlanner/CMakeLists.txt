#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playan Garai CERN EN/SMM/MRO 2023                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "PathPlanner")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ErrorHandler
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_OMPLGeometricPlanner)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    ${OMPL_LIBRARIES}
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${OMPL_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/OMPLGeometricPlanner/OMPLGeometricPlanner.cpp
                  ./src/OMPLGeometricPlanner/OMPLStateValidatorBridge.cpp
                  ./src/OMPLGeometricPlanner/OMPLMotionValidatorBridge.cpp
                  ./src/OMPLGeometricPlanner/OMPLStateSpaceConfiguration.cpp
    )
endif()

if (DISABLE_OMPLGeometricPlanner)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall)
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_OMPLGeometricPlanner)
    add_executable(OMPLGeometricPathPlannerSample ./samples/OMPLGeometricPathPlannerSample.cpp)
    target_link_libraries(OMPLGeometricPathPlannerSample PathPlanner )
endif()

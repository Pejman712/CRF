#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Ante Marić CERN BE/CEM/MRO 2022                                                                           ##
##                                                                                                                   ##
#######################################################################################################################

set(MODULE_NAME "KinematicChain")

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS Types
)
set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIR}
)
set(SRC_FILES
)

if(NOT DISABLE_URDFKinematicChain)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${urdfdom_LIBRARIES}
                    EventLogger
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${urdfdom_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/URDFKinematicChain/URDFKinematicChain.cpp
    )
endif()

if (DISABLE_URDFKinematicChain)
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

if(NOT DISABLE_URDFKinematicChain)
    add_executable(URDFKinematicChainSample ./samples/URDFKinematicChainSample.cpp)
    target_link_libraries(URDFKinematicChainSample ${MODULE_NAME})
endif()

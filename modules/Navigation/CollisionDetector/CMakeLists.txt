#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2018                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set (MODULE_NAME "CollisionDetector")

# TODO: The PathPLanner module depends of this module, and if the PCL dependency is added it enters
#       in conflict with OMPL in some computers, and it's not passing the test. 
#       - Issue: https://bitbucket.org/ompl/ompl/issues/480/segmentation-fault-problem-with-eigen

if(DISABLE_${MODULE_NAME})
    return()
endif()

set(MODULE_LIBS ${Boost_SYSTEM_LIBRARY}
                ${OCTOMAP_LIBRARIES}
)
set(INCLUDE_DIRS ./include
)
set(SRC_FILES
)

if(NOT DISABLE_FCLCollisionDetector)
    set(MODULE_LIBS ${MODULE_LIBS}
                    EventLogger
                    RobotBase
                    fcl
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/FCLCollisionDetector/FCLCollisionDetector.cpp
                  ./src/FCLCollisionDetector/FCLCollisionDetectorUtility.cpp
    )
endif()

if(NOT DISABLE_GPUVoxelsCollisionDetector)
    set(MODULE_LIBS ${MODULE_LIBS}
                    ${CUDA_LIBRARIES}
                    ${icl_core_LIBRARIES}
                    ${gpu_voxels_LIBRARIES}
                    ${PCL_LIBRARIES}
                    RobotArm 
    )
    set(INCLUDE_DIRS ${INCLUDE_DIRS}
                     ${CUDA_INCLUDE_DIRS}
                     ${icl_core_INCLUDE_DIRS}
                     ${gpu_voxels_INCLUDE_DIRS}
                     ${PCL_INCLUDE_DIRS}
    )
    set(SRC_FILES ${SRC_FILES}
                  ./src/GPUVoxelsCollisionDetector/GPUVoxelsCollisionDetector.cpp
    )
endif()

if (DISABLE_FCLCollisionDetector AND DISABLE_GPUVoxelsCollisionDetector)
    add_library(${MODULE_NAME} INTERFACE)
    target_include_directories(${MODULE_NAME} INTERFACE ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} INTERFACE)
else()
    add_library(${MODULE_NAME} SHARED ${SRC_FILES})
    target_include_directories(${MODULE_NAME} PUBLIC ${INCLUDE_DIRS})
    target_link_libraries(${MODULE_NAME} ${MODULE_LIBS})

    target_compile_options(${MODULE_NAME} PRIVATE -Wall)
endif()

# Inherited environment definitions required for the libraries
if (NOT DISABLE_GPUVoxelsCollisionDetector)
    target_compile_definitions(${MODULE_NAME} PUBLIC ${gpu_voxels_DEFINITIONS} ${icl_core_DEFINITIONS})
endif()

if(ENABLE_UnitTests)
    add_subdirectory(tests)
endif()

if (NOT DISABLE_FCLCollisionDetector)
    add_executable(FCLCollisionDetectorExample ./samples/FCLCollisionDetectorExample.cpp)
    target_link_libraries(FCLCollisionDetectorExample CollisionDetector)
endif()

if (NOT DISABLE_GPUVoxelsCollisionDetector)
    add_executable(GPUVoxelsCollisionDetectorExample ./samples/GPUVoxelsCollisionDetectorExample.cpp)
    target_link_libraries(GPUVoxelsCollisionDetectorExample CollisionDetector)

    if (NOT DISABLE_KinovaArm)
        add_executable(GPUVoxelsCollisionDetectorForKinovaExample ./samples/GPUVoxelsCollisionDetectorForKinovaExample.cpp)
        target_link_libraries(GPUVoxelsCollisionDetectorForKinovaExample CollisionDetector KinovaArm)
    endif()
endif()

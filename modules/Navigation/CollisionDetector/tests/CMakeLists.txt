#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN EN/SMM/MRO 2018                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(COLLISIONDETECTOR_TESTS_FILES main.cpp
)

if (NOT DISABLE_FCLCollisionDetector)
    set(COLLISIONDETECTOR_TESTS_FILES ${COLLISIONDETECTOR_TESTS_FILES}
                                      FCLCollisionDetectorTests.cpp
    )
endif()

if (NOT DISABLE_GPUVoxelsCollisionDetector)
    set(COLLISIONDETECTOR_TESTS_FILES ${COLLISIONDETECTOR_TESTS_FILES}
                                      GPUVoxelsCollisionDetectorTests.cpp
    )
endif()

add_executable(ut-collisiondetector ${COLLISIONDETECTOR_TESTS_FILES})
target_link_libraries(ut-collisiondetector gmock
                                           EventLogger
                                           CollisionDetector
)

add_test(CollisionDetectorUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-collisiondetector)

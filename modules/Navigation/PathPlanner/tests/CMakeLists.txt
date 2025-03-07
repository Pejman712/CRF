#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Jorge Playan Garai CERN EN/SMM/MRO 2023                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(PATHPLANNER_TESTS_FILES main.cpp
)

if(NOT DISABLE_OMPLGeometricPlanner)
    set(PATHPLANNER_TESTS_FILES ${PATHPLANNER_TESTS_FILES}
                                OMPLGeometricPlannerTests/OMPLGeometricPlannerTests.cpp
    )
endif()

add_executable(ut-pathplanner ${PATHPLANNER_TESTS_FILES})
target_link_libraries(ut-pathplanner gmock
                                     PathPlanner
)

add_test(PathPlannerUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-pathplanner)

#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license. ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales and CERN EN/SMM/MRO                                                                ##
##                                                                                                                   ##
#######################################################################################################################

set(LIBRARY_NAME "TimeOptimalTrajectoryGenerator")

if(DISABLE_${LIBRARY_NAME})
    return()
endif()

set(INCLUDE_DIRS ./include
                 ${EIGEN3_INCLUDE_DIRS}
)
set(SRC_FILES ./src/Path.cpp
              ./src/TrajectoryHelper.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRC_FILES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})

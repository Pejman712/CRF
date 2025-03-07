#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(LIBRARY_NAME "TRACInverseKinematics")

if(DISABLE_${LIBRARY_NAME})
    return()
endif()

set(MODULE_LIBS ${Boost_LIBRARIES}
                ${NLOPT_LIBRARIES}
                ${orocos_kdl_LIBRARIES}
)
set(INCLUDE_DIRS ./include
                 ${Boost_INCLUDE_DIRS}
                 ${EIGEN3_INCLUDE_DIRS}
                 ${NLOPT_INCLUDE_DIRS}
)
set(SRC_FILES ./src/kdl_tl.cpp
              ./src/nlopt_ik.cpp
              ./src/trac_ik.cpp
)

add_library(${LIBRARY_NAME} SHARED ${SRC_FILES})
target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})
target_link_libraries(${LIBRARY_NAME} ${MODULE_LIBS})

add_executable(TRACInverseKinematicsSample samples/TrackIkTestSchunk.cpp)
target_link_libraries(TRACInverseKinematicsSample ${LIBRARY_NAME})

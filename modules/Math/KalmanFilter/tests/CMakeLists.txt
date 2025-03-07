#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2021                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TESTS_FILES
    main.cpp
    KalmanFilterTests.cpp
    ExtendedKalmanFilterTests.cpp
)

add_executable(ut-kalmanfilter ${TESTS_FILES})
target_link_libraries(ut-kalmanfilter
    gmock
    KalmanFilter
)

add_test(KalmanFilterUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-kalmanfilter)

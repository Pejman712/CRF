#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2023. All rights reserved. This software is released under a CERN proprietary software licence.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2023                                                               ##
##                                                                                                                   ##
#######################################################################################################################

set(TESTS_FILES main.cpp
)

if (NOT DISABLE_VMU931)
    set(TESTS_FILES ${TESTS_FILES}
    )
endif()

if (NOT DISABLE_XsensMT)
    set(TESTS_FILES ${TESTS_FILES}
                    XsensMti200AdapterTests.cpp
    )
endif()

if (NOT DISABLE_IMUCommunicationPoint)
    set(TESTS_FILES ${TESTS_FILES}
                    IMUCommunicationPointTests.cpp
    )
endif()

if (NOT DISABLE_Gable)
    set(TESTS_FILES ${TESTS_FILES}
                    GableSE1Tests.cpp
                    GableSE2SE3Tests.cpp
    )
endif()

add_executable(ut-imu ${TESTS_FILES})
target_link_libraries(ut-imu gmock
                             IMU
)

add_test(IMUUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-imu)

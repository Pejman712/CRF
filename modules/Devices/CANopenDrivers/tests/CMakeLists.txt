#######################################################################################################################
##                                                                                                                   ##
## © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.  ##
## Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch ##
##                                                                                                                   ##
## Author: Natalija Topalovic CERN BE/CEM/MRO 2022                                                                   ##
##                                                                                                                   ##
#######################################################################################################################

set(CANOPENDEVICES_TESTS_FILES
    main.cpp
)

if (NOT DISABLE_CoE)
    set(CANOPENDEVICES_TESTS_FILES
        ${CANOPENDEVICES_TESTS_FILES}
        CoEMasterTests.cpp
    )
endif()

if (NOT DISABLE_CANopen AND NOT DISABLE_CiA402)
    set(CANOPENDEVICES_TESTS_FILES
        ${CANOPENDEVICES_TESTS_FILES}
        CiA402CANDriverTests.cpp
    )
endif()

if (NOT DISABLE_CoE AND NOT DISABLE_CiA402)
    set(CANOPENDEVICES_TESTS_FILES
        ${CANOPENDEVICES_TESTS_FILES}
        CiA402CoEDriverTests.cpp
    )
endif()

if (NOT DISABLE_CiA402CommunicationPointTests)
    set(CANOPENDEVICES_TESTS_FILES
        ${CANOPENDEVICES_TESTS_FILES}
        CiA402CommunicationPointTests.cpp
    )
endif()

add_executable(ut-canopendrivers ${CANOPENDEVICES_TESTS_FILES})
target_link_libraries(ut-canopendrivers
    gmock
    CANopenDrivers
)

add_test(CANopenDriversUnitTests ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/ut-canopendrivers)
